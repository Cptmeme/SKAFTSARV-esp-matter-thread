# SKAFTSÄRV hardware test (ESP32-C6 SuperMini)

Standalone ESP-IDF app that only exercises the lamp hardware — 30 WS2812 LEDs on
GPIO14 and the two buttons on GPIO19/GPIO18 — so you can prove every solder
joint before dealing with Matter. Builds in ~1 minute instead of ~15.

## Build & flash

Your shell `python3` is 3.14 but the ESP-IDF venv was installed for 3.13, so
plain `source ~/esp/esp-idf/export.sh` fails. Put 3.13 first:

```sh
export PATH="/opt/homebrew/opt/python@3.13/bin:$PATH"
source ~/esp/esp-idf/export.sh
cd ~/esp/SKAFTSARV-esp-matter-thread/led_test
idf.py -p /dev/cu.usbmodem* flash monitor      # exit the monitor with Ctrl-]
```

Target is pinned to `esp32c6` in `sdkconfig.defaults`, and the console runs over
the SuperMini's native USB port (there is no USB-UART bridge on that board).

## macOS + USB-Serial-JTAG quirk

On this board RTS is the EN (reset) line and DTR is GPIO9 (BOOT). macOS asserts
both for a moment whenever a program opens the port, so *connecting* a serial
monitor resets the chip into the ROM download loader and you see:

```
rst:0x15 (USB_UART_HPSYS),boot:0x0 (USB_BOOT)
wait usb download
```

That is not a fault and the firmware is fine. Attach the monitor first, then tap
the RST button on the SuperMini: the monitor already has DTR deasserted by then,
so the chip boots normally with BOOT high and the monitor reconnects when the
USB device re-enumerates.

## Wiring

| SuperMini | Lamp                                                       |
|-----------|------------------------------------------------------------|
| GPIO14    | LED DIN (330–470 Ω in series, close to the pin)             |
| GPIO19    | Button 1, other side to GND                                 |
| GPIO18    | Button 2, other side to GND                                 |
| GND       | Strip GND **and** button common — the #1 cause of "nothing works" |
| 5V        | Strip +5V. Fine for a few LEDs over USB; 30 at full white is ~1.8 A, so use an external supply for real brightness. |

Buttons use the ESP32-C6's internal pull-ups, so no external resistors are
needed — a press just shorts the pin to GND. A 470–1000 µF capacitor across
+5V/GND at the strip end absorbs the inrush.

## Using it

The LED sequence starts by itself at boot and button edges are logged the whole
time, so you can press them whenever. Everything is adjustable live from the
`led>` prompt — no reflashing while you probe:

```
info                 show current LED config
count 30             LED count
pin 14               move the data line to another GPIO
bright 40            brightness cap 0-255 (low by default: USB gives ~500 mA)
maxma 850            current budget in mA, WLED-style; 0 disables the limiter
model ws2812         chip timing: ws2812 / sk6812 / ws2811 / ws2816
order grb            byte order: grb, rgb, grbw, ...
auto                 loop the full 6-step LED diagnostic
walk 200             one lit pixel crawling down the strip, index logged
pixel 0 255 0 0      light exactly one LED
solid 255 255 255    all LEDs one colour
chase / rainbow / off
probe 2              drive GPIO14 as a 2 Hz square wave (meter/scope the joint)

btn                  button levels, state and press counts
btnpin 1 19          move a button to another GPIO
btnactive 0          level the pin reads while pressed (0 = switch to GND)
buttons              LEDs mirror the buttons: BTN1 lights the first half green,
                     BTN2 the second half blue — tests both at once, hands-free
```

## The current limiter

Every frame is rendered into a buffer, then scaled twice on the way to the
strip: once by the `bright` cap, then again by a current limiter that estimates
the frame's draw (~20 mA per lit channel plus ~1 mA of quiescent per chip) and
scales the whole frame down if it exceeds `maxma`. This is what WLED's
Automatic Brightness Limiter does, and it is why the reference SKAFTSÄRV
conversions survive on USB power: 30 LEDs at full white want ~1.8 A, and the
limiter simply never lets them ask for it.

Set the budget to what the supply can really deliver:

| Powering from | Sensible `maxma` |
|---|---|
| A laptop USB port | `maxma 400` — leaves headroom for the board itself |
| A 2–3 A USB charger | `maxma 1500` |
| A dedicated 5 V bench supply | `maxma 2000` or `maxma 0` |

The limiter logs once when it engages and once when it releases, and `info`
reports the budget, the last frame's estimated draw, and the all-on worst case.

It stops the firmware asking for more current than the budget — it cannot
conjure current the supply does not have. If the far end of the strip still
dies with a sane budget set, that is the wiring, not the firmware.

## Reading the LED result

| Symptom | Cause |
|---|---|
| Nothing lights at all | No common GND, no 5V at the strip, or the DIN joint is open. Run `probe 2` and meter GPIO14 at the far end of the data wire — if it does not swing, the joint is the problem. |
| First LED lights (often white/random), rest dark | Data reaches LED 1 but its DOUT→next-DIN link is broken, or LED 1 is dead. |
| `walk` stops or freezes at index N | The chain breaks right after LED N — resolder that joint. |
| Colours named in the log don't match what you see | Not a solder fault. Try `order rgb`, `order brg`, … until RED shows red. |
| Works dim, flickers or reboots on white | Power, not data. The USB 5V rail is sagging — external supply, inject at both ends. |
| Far end shifts yellow/orange on white | Voltage drop along the strip. Same fix. |
| Random flicker, occasional wrong pixels | Data line too long or unterminated: series resistor at the ESP, short wire, GND running alongside it. |

Sanity check: `pin 8` + `count 1` drives the SuperMini's own on-board RGB LED.
If that works, the firmware, RMT and timing are all fine and the fault is
somewhere on the GPIO14 side.

3.3 V data into a 5 V strip is marginally out of spec. It usually works; if the
first LED is glitchy, either add a level shifter or run the strip at ~4.5 V (a
diode in series with +5V) so its logic threshold drops.

## Reading the button result

Each edge prints the press number, how long it was held, and how many raw edges
of contact bounce were seen — that bounce count is the tell:

| What you see | Cause |
|---|---|
| No log line at all when pressed, `btn` shows `raw=1` always | Open joint, or the button's other leg never reaches GND. |
| `raw=0` always, "PRESSED" straight after boot | Shorted joint, or the switch is wired to 3V3 instead of GND (then use `btnactive 1`). |
| 0–5 edges of bounce per press | Normal, healthy joint. |
| Dozens of edges, or presses logged while you touch nothing | Cold joint or a floating wire picking up noise. Reflow it, and keep the button wires away from the LED data line. |
| Both buttons fire together | The two lines are bridged, or they share a wire that is not actually connected to GND. |

`btnpin 1 <gpio>` lets you retarget a button without reflashing, which is the
quickest way to tell a bad joint from a bad pin.
