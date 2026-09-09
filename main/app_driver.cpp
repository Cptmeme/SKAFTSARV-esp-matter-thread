/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   Glue between the Matter data model and the SKAFTSÄRV hardware: a 30-LED
   WS2812 strip on GPIO14 and the lamp's own two buttons on GPIO19/GPIO18.
*/

#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

#include <esp_matter.h>
#include <app_priv.h>
#include <app_reset.h>
#include <common_macros.h>

#include <device.h>
#include <button_gpio.h>

#include "lamp_config.h"
#include "strip_driver.h"

using namespace chip::app::Clusters;
using namespace esp_matter;

static const char *TAG = "app_driver";
extern uint16_t light_endpoint_id;
extern uint16_t effect_endpoint_id;

/* Matter sends x and y as two separate attribute writes, so the other half has
   to be remembered. */
static uint16_t current_x = 0;
static uint16_t current_y = 0;

/* The strip driver is a singleton, but the endpoint still wants a non-null
   private-data pointer, so it gets the address of this token. */
static uint8_t s_light_token;

/* Do any conversions/remapping for the actual value here */
static esp_err_t app_driver_light_set_power(esp_matter_attr_val_t *val)
{
    return strip_driver_set_power(val->val.b);
}

static esp_err_t app_driver_light_set_brightness(esp_matter_attr_val_t *val)
{
    int value = REMAP_TO_RANGE(val->val.u8, MATTER_BRIGHTNESS, STANDARD_BRIGHTNESS);
    return strip_driver_set_brightness(value);
}

static esp_err_t app_driver_light_set_hue(esp_matter_attr_val_t *val)
{
    int value = REMAP_TO_RANGE(val->val.u8, MATTER_HUE, STANDARD_HUE);
    return strip_driver_set_hue(value);
}

static esp_err_t app_driver_light_set_saturation(esp_matter_attr_val_t *val)
{
    int value = REMAP_TO_RANGE(val->val.u8, MATTER_SATURATION, STANDARD_SATURATION);
    return strip_driver_set_saturation(value);
}

static esp_err_t app_driver_light_set_temperature(esp_matter_attr_val_t *val)
{
    uint32_t value = REMAP_TO_RANGE_INVERSE(val->val.u16, STANDARD_TEMPERATURE_FACTOR);
    return strip_driver_set_temperature(value);
}

static esp_err_t app_driver_light_set_xy(uint16_t x, uint16_t y)
{
    return strip_driver_set_xy(x, y);
}

/* ------------------------------------------------------------------------ */
/* Buttons                                                                   */
/* ------------------------------------------------------------------------ */

/* Both button handlers drive the Matter attributes rather than the strip
   directly, so the controller and any smart-home app stay in step with what
   somebody just did by hand at the lamp. */
static void app_driver_button_toggle_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "Toggle button pressed");
    uint16_t endpoint_id = light_endpoint_id;
    uint32_t cluster_id = OnOff::Id;
    uint32_t attribute_id = OnOff::Attributes::OnOff::Id;

    attribute_t *attribute = attribute::get(endpoint_id, cluster_id, attribute_id);

    esp_matter_attr_val_t val;
    attribute::get_val(attribute, &val);
    val.val.b = !val.val.b;
    attribute::update(endpoint_id, cluster_id, attribute_id, &val);
}

static void app_driver_button_level_cb(void *arg, void *data)
{
    /* 25 / 50 / 75 / 100 % of the Matter level range, cycling round. */
    static const uint8_t k_levels[] = { 64, 127, 190, 254 };

    attribute_t *level_attribute =
        attribute::get(light_endpoint_id, LevelControl::Id, LevelControl::Attributes::CurrentLevel::Id);
    esp_matter_attr_val_t level_val;
    attribute::get_val(level_attribute, &level_val);

    uint8_t next = k_levels[0];
    for (size_t i = 0; i < sizeof(k_levels); i++) {
        if (level_val.val.u8 < k_levels[i]) {
            next = k_levels[i];
            break;
        }
    }
    ESP_LOGI(TAG, "Brightness button: level %u -> %u", level_val.val.u8, next);
    level_val.val.u8 = next;
    attribute::update(light_endpoint_id, LevelControl::Id, LevelControl::Attributes::CurrentLevel::Id, &level_val);

    /* Reaching for the brightness button on a lamp that is off means you want
       it on. */
    attribute_t *onoff_attribute = attribute::get(light_endpoint_id, OnOff::Id, OnOff::Attributes::OnOff::Id);
    esp_matter_attr_val_t onoff_val;
    attribute::get_val(onoff_attribute, &onoff_val);
    if (!onoff_val.val.b) {
        onoff_val.val.b = true;
        attribute::update(light_endpoint_id, OnOff::Id, OnOff::Attributes::OnOff::Id, &onoff_val);
    }
}

/* Factory reset: hold the on/off button for LAMP_FACTORY_RESET_MS, then let go.
   Arming and firing are split so that letting go early cancels it, and so the
   strip can confirm the hold registered - without that feedback there is no way
   to tell when 10 seconds have passed on a sealed lamp. */
static bool s_factory_reset_armed = false;

static void app_driver_factory_reset_armed_cb(void *arg, void *data)
{
    if (!s_factory_reset_armed) {
        s_factory_reset_armed = true;
        ESP_LOGW(TAG, "Factory reset armed - release the button to wipe the Matter pairing");
        strip_driver_indicate();
    }
}

static void app_driver_factory_reset_released_cb(void *arg, void *data)
{
    if (s_factory_reset_armed) {
        s_factory_reset_armed = false;
        ESP_LOGW(TAG, "Starting factory reset");
        esp_matter::factory_reset();
    }
}

static button_handle_t lamp_button_create(int gpio)
{
    button_handle_t handle = NULL;
    const button_config_t btn_cfg = {0};
    const button_gpio_config_t gpio_cfg = {
        .gpio_num          = gpio,
        .active_level      = 0,      /* the switch shorts the pin to GND */
        .enable_power_save = false,
        .disable_pull      = false,  /* use the internal pull-up */
    };

    if (iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create lamp button on gpio %d", gpio);
        return NULL;
    }
    return handle;
}

/* ------------------------------------------------------------------------ */

esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val)
{
    esp_err_t err = ESP_OK;
    if (endpoint_id == light_endpoint_id) {
        if (cluster_id == OnOff::Id) {
            if (attribute_id == OnOff::Attributes::OnOff::Id) {
                err = app_driver_light_set_power(val);
            }
        } else if (cluster_id == LevelControl::Id) {
            if (attribute_id == LevelControl::Attributes::CurrentLevel::Id) {
                err = app_driver_light_set_brightness(val);
            }
        } else if (cluster_id == ColorControl::Id) {
            if (attribute_id == ColorControl::Attributes::CurrentHue::Id) {
                err = app_driver_light_set_hue(val);
            } else if (attribute_id == ColorControl::Attributes::CurrentSaturation::Id) {
                err = app_driver_light_set_saturation(val);
            } else if (attribute_id == ColorControl::Attributes::ColorTemperatureMireds::Id) {
                err = app_driver_light_set_temperature(val);
            } else if (attribute_id == ColorControl::Attributes::CurrentX::Id) {
                current_x = val->val.u16;
                err = app_driver_light_set_xy(current_x, current_y);
            } else if (attribute_id == ColorControl::Attributes::CurrentY::Id) {
                current_y = val->val.u16;
                err = app_driver_light_set_xy(current_x, current_y);
            }
        }
        /* Never fail the attribute write because the strip hiccuped. An RMT
           timeout (the ISR can be stalled by an NVS commit) would otherwise make
           esp-matter reject the whole update and lose the change. */
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "light update failed: %s - keeping the attribute anyway",
                     esp_err_to_name(err));
            err = ESP_OK;
        }
    } else if (effect_endpoint_id != 0 && endpoint_id == effect_endpoint_id) {
        /* The rainbow switch. Endpoint 0 is the root node, so the id is only
           trusted once app_main has actually created the endpoint. */
        if (cluster_id == OnOff::Id && attribute_id == OnOff::Attributes::OnOff::Id) {
            err = strip_driver_set_effect(val->val.b);
        }
    }
    return err;
}

esp_err_t app_driver_light_set_defaults(uint16_t endpoint_id)
{
    esp_err_t err = ESP_OK;
    esp_matter_attr_val_t val;

    /* Setting brightness */
    attribute_t *attribute = attribute::get(endpoint_id, LevelControl::Id, LevelControl::Attributes::CurrentLevel::Id);
    attribute::get_val(attribute, &val);
    err |= app_driver_light_set_brightness(&val);

    /* Seed every colour representation, not just the one the active mode uses.
       The driver only ever learns values from change callbacks, so anything not
       seeded here keeps its startup default until a controller happens to write
       a *different* value. That is what left saturation at 0 while the lamp
       booted in colour-temperature mode: Matter already had CurrentSaturation
       at 254, so asking for a fully saturated colour changed nothing, fired no
       callback, and every hue rendered as white. */
    attribute = attribute::get(endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentHue::Id);
    attribute::get_val(attribute, &val);
    err |= app_driver_light_set_hue(&val);
    attribute = attribute::get(endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentSaturation::Id);
    attribute::get_val(attribute, &val);
    err |= app_driver_light_set_saturation(&val);
    attribute = attribute::get(endpoint_id, ColorControl::Id,
                               ColorControl::Attributes::ColorTemperatureMireds::Id);
    attribute::get_val(attribute, &val);
    err |= app_driver_light_set_temperature(&val);

    /* Setting color - this runs last so the active mode wins. */
    attribute = attribute::get(endpoint_id, ColorControl::Id, ColorControl::Attributes::ColorMode::Id);
    attribute::get_val(attribute, &val);
    if (val.val.u8 == (uint8_t)ColorControl::ColorMode::kCurrentHueAndCurrentSaturation) {
        /* Setting hue */
        attribute = attribute::get(endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentHue::Id);
        attribute::get_val(attribute, &val);
        err |= app_driver_light_set_hue(&val);
        /* Setting saturation */
        attribute = attribute::get(endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentSaturation::Id);
        attribute::get_val(attribute, &val);
        err |= app_driver_light_set_saturation(&val);
    } else if (val.val.u8 == (uint8_t)ColorControl::ColorMode::kColorTemperature) {
        /* Setting temperature */
        attribute = attribute::get(endpoint_id, ColorControl::Id, ColorControl::Attributes::ColorTemperatureMireds::Id);
        attribute::get_val(attribute, &val);
        err |= app_driver_light_set_temperature(&val);
    } else if (val.val.u8 == (uint8_t)ColorControl::ColorMode::kCurrentXAndCurrentY) {
        /* Setting XY coordinates */
        attribute = attribute::get(endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentX::Id);
        attribute::get_val(attribute, &val);
        current_x = val.val.u16;
        attribute = attribute::get(endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentY::Id);
        attribute::get_val(attribute, &val);
        current_y = val.val.u16;
        err |= app_driver_light_set_xy(current_x, current_y);
    } else {
        ESP_LOGE(TAG, "Color mode not supported");
    }

    /* Setting power */
    attribute = attribute::get(endpoint_id, OnOff::Id, OnOff::Attributes::OnOff::Id);
    attribute::get_val(attribute, &val);
    err |= app_driver_light_set_power(&val);

    return err;
}

app_driver_handle_t app_driver_light_init()
{
    esp_err_t err = strip_driver_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialise the LED strip: %s", esp_err_to_name(err));
        return NULL;
    }
    return (app_driver_handle_t)&s_light_token;
}

app_driver_handle_t app_driver_button_init()
{
    /* The board's own button (BOOT on the SuperMini) keeps its upstream role:
       toggle on press, factory reset on a long press. It is only reachable with
       the lamp open, which is why the lamp's own buttons are wired up below. */
    button_handle_t handle = NULL;
    const button_config_t btn_cfg = {0};
    const button_gpio_config_t btn_gpio_cfg = button_driver_get_config();

    if (iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create button device");
        return NULL;
    }
    iot_button_register_cb(handle, BUTTON_PRESS_DOWN, NULL, app_driver_button_toggle_cb, NULL);

    /* The lamp's original buttons. */
    button_handle_t onoff_button = lamp_button_create(LAMP_BTN_ONOFF_GPIO);
    if (onoff_button) {
        iot_button_register_cb(onoff_button, BUTTON_SINGLE_CLICK, NULL, app_driver_button_toggle_cb, NULL);

        /* A long press on this button is the factory reset. A single click is
           unaffected: it only fires on a short press and release. */
        button_event_args_t hold_args = {};
        hold_args.long_press.press_time = LAMP_FACTORY_RESET_MS;
        iot_button_register_cb(onoff_button, BUTTON_LONG_PRESS_START, &hold_args,
                               app_driver_factory_reset_armed_cb, NULL);
        iot_button_register_cb(onoff_button, BUTTON_PRESS_UP, NULL,
                               app_driver_factory_reset_released_cb, NULL);
    }

    button_handle_t level_button = lamp_button_create(LAMP_BTN_LEVEL_GPIO);
    if (level_button) {
        iot_button_register_cb(level_button, BUTTON_SINGLE_CLICK, NULL, app_driver_button_level_cb, NULL);
    }

    ESP_LOGI(TAG, "Lamp buttons: on/off on gpio %d (hold %d s to factory reset), brightness on gpio %d",
             LAMP_BTN_ONOFF_GPIO, LAMP_FACTORY_RESET_MS / 1000, LAMP_BTN_LEVEL_GPIO);
    return (app_driver_handle_t)handle;
}
