/*
   Hardware configuration for the IKEA SKAFTSÄRV / ESP32-C6 SuperMini conversion.
   Plain defines only, so this is safe to include from both C and C++.
*/

#pragma once

/* LED strip: 30 x WS2812, GRB byte order, data on GPIO14. */
#define LAMP_LED_GPIO           14
#define LAMP_LED_COUNT          30

/* Current budget for the whole strip, in mA.
 *
 * Measured on this lamp: 933 mA (level 128/255 on full white) ran stable with
 * no colour shift, with the strip fed directly from the power wire rather than
 * through the board. 900 mA sits just under that and just above WLED's 850 mA
 * default, which is what the reference conversions have effectively been
 * running at all along.
 *
 * This does NOT cap brightness. A frame is scaled only if it would exceed the
 * budget, so a saturated colour (~630 mA at full output) is never touched and
 * only near-white frames get pulled back. Set to 0 to disable the limiter.
 */
#define LAMP_MAX_MA             900

/* Buttons salvaged from the original control board, each shorting to GND.
 * Internal pull-ups are used, so no external resistors are needed. */
#define LAMP_BTN_ONOFF_GPIO     18
#define LAMP_BTN_LEVEL_GPIO     19

/* Hold the on/off button this long, then release, to wipe the Matter fabrics
 * and start advertising for commissioning again. Long enough that it cannot
 * happen by accident; the strip flashes red once the hold is registered. */
#define LAMP_FACTORY_RESET_MS   10000

/* Moving rainbow effect: how often a frame is pushed, and how far the hue
   ramp rotates each frame. 2 degrees every 40 ms walks the whole wheel past a
   given LED in about 7 seconds. */
#define LAMP_EFFECT_STEP_MS     40
#define LAMP_EFFECT_HUE_STEP    2
