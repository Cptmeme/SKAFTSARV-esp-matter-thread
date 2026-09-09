# SKAFTSÄRV → Matter over Thread

Firmware that converts an **IKEA SKAFTSÄRV** lamp into a native **Matter over Thread**
colour light, running on an **ESP32-C6 SuperMini**.

No Wi-Fi, no cloud, no bridge. The lamp joins your Thread network directly and works in
Apple Home, Google Home, Alexa or Home Assistant like any other Matter accessory —
including Adaptive Lighting.

## What it does

| Feature | Detail |
|---|---|
| On/off, dimming | Full Matter Level Control |
| Colour | Hue/saturation, colour temperature and CIE xy |
| Adaptive Lighting | Works in Apple Home |
| Rainbow effect | A moving hue cycle across the strip, on its own on/off switch |
| Physical buttons | The lamp's original two buttons: on/off and brightness |
| Factory reset | Hold the on/off button 10 s, release — the strip flashes red to confirm |
| Current limiting | WLED-style automatic brightness limiter, so the strip can't brown out its supply |
| Identity | Reports as `IKEA` / `SKAFTSARV` with a MAC-derived serial number |

## Hardware

- **IKEA SKAFTSÄRV** lamp (30 × WS2812, GRB, 5 V)
- **ESP32-C6 SuperMini**
- The lamp's original 5 V USB cable and supply

The original control board has to go. Leaving it connected to the LED data line makes two
drivers fight over it, which produces flicker, wrong colours and a strip that dies
partway along.

### Keeping the original buttons

The buttons sit on that same board and are aligned with the housing, so they are worth
salvaging rather than replacing.

**Cut the original PCB in half.** Keep the half carrying the two buttons and discard the
half with the controller and the microphone — that removes the old controller from the
LED data line while leaving the buttons where the housing expects them.

**Three wires in total**, because the two buttons share a ground on the salvaged half:

| Wire | From | To |
|---|---|---|
| 1 | Shared ground pad | SuperMini **GND** |
| 2 | Button 1 positive pad | **GPIO18** (on/off) |
| 3 | Button 2 positive pad | **GPIO19** (brightness) |

The firmware enables the C6's internal pull-ups and treats a press as the pin being
pulled to ground, so no external resistors are needed.

### Wiring

| SuperMini | Lamp | Notes |
|---|---|---|
| GPIO14 | LED data (brown) | 330–470 Ω in series, close to the pin |
| GPIO18 | Button 1 (on/off) | Other side to GND; internal pull-up, no resistor needed |
| GPIO19 | Button 2 (brightness) | As above |
| GND | Strip GND (black) **and** button common | Must be common — the usual cause of "nothing works" |
| 5V | Strip +5 V (white) | See below |

Tap the strip's +5 V **directly from the power cable**, in parallel with the board's 5V
pin, rather than routing strip current through the board. The SuperMini's trace and its
Schottky are typically rated around 1 A.

Do **not** feed 5 V into the battery pads. That pad is the LiPo charger's output and is
regulated to 4.2 V.

### Power budget

30 WS2812s at full white draw about **1.8 A**, while the lamp is rated 1.5 W (300 mA).
`LAMP_MAX_MA` in [`main/lamp_config.h`](main/lamp_config.h) defaults to **900 mA** —
measured stable, and roughly what a stock WLED install allows itself.

This caps *current*, not brightness. A frame is scaled only if it would exceed the
budget, so a saturated colour (~630 mA) runs at full output and only near-white frames
get pulled back. Set it to 0 to disable.

Perceived brightness goes roughly as the cube root of luminance, so 900 mA looks only
~44 % brighter than the stock lamp, and dropping to 600 mA costs about 13 % while
halving the heat in a housing designed for 1.5 W. Worth a closed-housing soak test
before settling on a value.

## Building

Requires [esp-matter](https://github.com/espressif/esp-matter) and its ESP-IDF.

```sh
export ESP_MATTER_PATH=~/esp-matter          # wherever yours lives
. $ESP_MATTER_PATH/export.sh

idf.py set-target esp32c6
idf.py build
idf.py -p /dev/ttyUSB0 erase-flash
idf.py -p /dev/ttyUSB0 flash monitor
```

`sdkconfig` is intentionally not committed — `sdkconfig.defaults.esp32c6` reproduces it
exactly, and it carries the settings that matter (Thread on, Wi-Fi off, USB console,
CHIP shell **off**).

`build_thread.sh` is a wrapper for installs where `export.sh` doesn't work; ignore it if
yours is healthy.

## Commissioning

Uses the standard CHIP **test** credentials:

| | |
|---|---|
| Manual pairing code | `34970112332` |
| Discriminator | 3840 |
| Passcode | 20202021 |

Fixed in firmware, so they survive resets and reflashes — worth a sticker inside the
housing. You need a **Thread Border Router** (Apple TV/HomePod, Nest Hub, Echo 4th gen,
or a self-hosted OTBR).

These are test values shared with every stock esp-matter example. If another such device
is powered nearby, your phone may try to commission that one instead.

## Things that cost me a lot of time

**The CHIP shell must stay off.** `CONFIG_ENABLE_CHIP_SHELL=n`, explicitly, because the
shared `sdkconfig.defaults` sets it to `y`. Its main loop calls `linenoise()` and, when
that returns NULL, loops back with no delay. With a USB host attached the read blocks and
the task sleeps; with **no** host it returns immediately and the task spins at priority 5
— the same priority as the OpenThread task — starving Thread and Matter. The lamp then
advertises for commissioning and pairs fine over USB, but hangs at "connecting" and times
out whenever it runs standalone. Every attempt to observe it by attaching USB makes it
disappear.

**esp-matter's `extended_color_light` has no hue/saturation feature.** It installs only
colour temperature and xy, so Apple Home shows a white slider and no colour wheel. The
feature is added explicitly in `app_main.cpp`.

**Seed every colour representation at startup.** The driver learns values from change
callbacks, so anything not seeded keeps its default until a controller writes a
*different* value. Booting in CT mode left saturation at 0 while Matter's attribute was
already 254 — so asking for a saturated colour changed nothing, fired no callback, and
every hue rendered as white.

**Basic Information is served two ways.** Vendor and product names come from the
`DeviceInstanceInfoProvider`; the serial number is read from esp-matter's own data model,
where `basic_information::create()` never creates that attribute. It has to be added
explicitly.

**Don't fail a Matter write because the strip hiccuped.** An RMT timeout — the ISR can be
stalled by an NVS commit — used to make esp-matter reject the whole attribute update and
lose the command.

**`espressif/led_strip` can't be used.** esp-matter's `device_hal/led_driver` pins it to
1.x, which can't coexist with 3.x. `strip_driver.c` drives the WS2812 waveform straight
from RMT instead.

## led_test/

A standalone bench app for checking the strip and buttons before dealing with Matter —
builds in a minute rather than fifteen. Walking pixel, solid colours, colour-order
testing, a current-budget sweep, button bounce counting, and a square-wave mode for
metering a suspect solder joint. See [`led_test/README.md`](led_test/README.md).

## Credits

- [simoneluconi/SKAFTSARV-to-WLED](https://github.com/simoneluconi/SKAFTSARV-to-WLED) — original conversion and pinout
- [isarrider/SKAFTSARV-to-WLED](https://github.com/isarrider/SKAFTSARV-to-WLED) — variant that removes the original board
- Based on the `light` example from [espressif/esp-matter](https://github.com/espressif/esp-matter)

Not affiliated with or endorsed by IKEA. The vendor and product IDs are Espressif's test
values, deliberately left alone — using a real manufacturer's Matter vendor ID would fail
device attestation and claim a certification this lamp does not hold.

## Licence

GPL-3.0. See [LICENSE](LICENSE).

Copyright (C) 2026 Cptmeme.

The files under `main/` that began life as Espressif's `light` example keep their
original "Public Domain (or CC0)" headers — that is accurate about where they came from,
and the unmodified originals remain available under those terms from
[espressif/esp-matter](https://github.com/espressif/esp-matter). esp-matter and
connectedhomeip are Apache-2.0, which is compatible with GPL-3.0 in this direction.
