/*
   Single-light driver for the lamp's WS2812 strip.

   The whole strip is treated as one light: every pixel shows the same colour,
   which is what the Matter data model expects from a colour light endpoint.
*/

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <esp_err.h>

#include "lamp_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Bring up the RMT channel and the strip. Safe to call once, at boot. */
esp_err_t strip_driver_init(void);

/** On/off. Brightness and colour are remembered while off. */
esp_err_t strip_driver_set_power(bool power);

/** Brightness, 0-100, matching STANDARD_BRIGHTNESS in app_priv.h. */
esp_err_t strip_driver_set_brightness(uint8_t percent);

/** Hue 0-360 and saturation 0-100; either switches the light to HS colour mode. */
esp_err_t strip_driver_set_hue(uint16_t hue);
esp_err_t strip_driver_set_saturation(uint8_t saturation);

/** Colour temperature in kelvin; switches the light to CT mode. */
esp_err_t strip_driver_set_temperature(uint32_t kelvin);

/** CIE xy as sent by Matter (0-65535 per axis); switches the light to xy mode. */
esp_err_t strip_driver_set_xy(uint16_t x, uint16_t y);

/** Turn the moving rainbow effect on or off. While it is on the strip animates
 *  a hue ramp travelling along its length; the light's on/off state and
 *  brightness still apply. Turning it off restores the Matter colour. */
esp_err_t strip_driver_set_effect(bool enabled);
bool strip_driver_get_effect(void);

/** Flash the strip red a few times, then go back to what it was showing.
 *  Used to confirm a factory-reset hold has registered. Blocks for about a
 *  second. */
esp_err_t strip_driver_indicate(void);

/** Estimated draw of the frame currently on the strip, in mA. */
int strip_driver_last_ma(void);

#ifdef __cplusplus
}
#endif
