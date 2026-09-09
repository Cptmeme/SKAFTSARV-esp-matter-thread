/*
   LED strip driver for the SKAFTSÄRV conversion.

   Every pixel shows the same colour, so the 30-LED strip presents itself to
   Matter as a single colour light.

   The WS2812 waveform is generated straight from the RMT peripheral rather
   than through the espressif/led_strip component, because esp-matter's own
   device_hal/led_driver already pins led_strip to 1.x and the two versions
   cannot coexist in one build.

   The other non-obvious part is the current limiter: brightness is never
   capped, but a frame that would draw more than LAMP_MAX_MA is scaled down as
   a whole. That is what WLED's automatic brightness limiter does, and it is
   what keeps a strip this size from browning out its own supply - a starved
   WS2812 chain loses green and blue before red, and a chip that browns out
   stops passing data, so everything after it goes dark too.
*/

#include "strip_driver.h"

#include <stdlib.h>
#include <string.h>

#include <driver/gpio.h>
#include <driver/rmt_encoder.h>
#include <driver/rmt_tx.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <color_format.h>

static const char *TAG = "strip";

/* 10 MHz => one RMT tick is 0.1 us. */
#define WS2812_RESOLUTION_HZ    (10 * 1000 * 1000)
#define WS2812_T0H              3     /* 0.3 us */
#define WS2812_T0L              9     /* 0.9 us */
#define WS2812_T1H              9     /* 0.9 us */
#define WS2812_T1L              3     /* 0.3 us */
#define WS2812_RESET_TICKS      2800  /* 280 us of low latches the frame */

/* One 48-symbol block is the C6 default; two doubles the time the ISR has to
   refill, so a late interrupt cannot tear a frame and strand the tail. */
#define RMT_MEM_SYMBOLS         96

/* ~20 mA per fully lit colour channel, plus ~1 mA of quiescent draw per chip. */
#define MA_PER_CHANNEL          20

/* ------------------------------------------------------------------------ */
/* WS2812 RMT encoder: the pixel bytes, then the reset gap                   */
/* ------------------------------------------------------------------------ */
typedef struct {
    rmt_encoder_t      base;      /* must stay first; we cast back to it */
    rmt_encoder_t     *bytes;
    rmt_encoder_t     *copy;
    rmt_symbol_word_t  reset_code;
    int                step;
} ws2812_encoder_t;

static size_t ws2812_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                            const void *primary_data, size_t data_size,
                            rmt_encode_state_t *ret_state)
{
    ws2812_encoder_t *enc = (ws2812_encoder_t *)encoder;
    rmt_encode_state_t session = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t symbols = 0;

    if (enc->step == 0) {
        symbols += enc->bytes->encode(enc->bytes, channel, primary_data, data_size, &session);
        if (session & RMT_ENCODING_COMPLETE) {
            enc->step = 1;
        }
        if (session & RMT_ENCODING_MEM_FULL) {
            *ret_state = state | RMT_ENCODING_MEM_FULL;
            return symbols;
        }
    }
    if (enc->step == 1) {
        symbols += enc->copy->encode(enc->copy, channel, &enc->reset_code,
                                     sizeof(enc->reset_code), &session);
        if (session & RMT_ENCODING_COMPLETE) {
            enc->step = 0;
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
        }
    }
    *ret_state = state;
    return symbols;
}

static esp_err_t ws2812_encoder_del(rmt_encoder_t *encoder)
{
    ws2812_encoder_t *enc = (ws2812_encoder_t *)encoder;
    rmt_del_encoder(enc->bytes);
    rmt_del_encoder(enc->copy);
    free(enc);
    return ESP_OK;
}

static esp_err_t ws2812_encoder_reset(rmt_encoder_t *encoder)
{
    ws2812_encoder_t *enc = (ws2812_encoder_t *)encoder;
    rmt_encoder_reset(enc->bytes);
    rmt_encoder_reset(enc->copy);
    enc->step = 0;
    return ESP_OK;
}

static esp_err_t ws2812_encoder_new(rmt_encoder_handle_t *ret_encoder)
{
    ws2812_encoder_t *enc = calloc(1, sizeof(ws2812_encoder_t));
    if (!enc) {
        return ESP_ERR_NO_MEM;
    }
    enc->base.encode = ws2812_encode;
    enc->base.del    = ws2812_encoder_del;
    enc->base.reset  = ws2812_encoder_reset;

    rmt_bytes_encoder_config_t bytes_cfg = {
        .bit0 = { .level0 = 1, .duration0 = WS2812_T0H, .level1 = 0, .duration1 = WS2812_T0L },
        .bit1 = { .level0 = 1, .duration0 = WS2812_T1H, .level1 = 0, .duration1 = WS2812_T1L },
        .flags = { .msb_first = 1 },
    };
    esp_err_t err = rmt_new_bytes_encoder(&bytes_cfg, &enc->bytes);
    if (err != ESP_OK) {
        free(enc);
        return err;
    }

    rmt_copy_encoder_config_t copy_cfg = {};
    err = rmt_new_copy_encoder(&copy_cfg, &enc->copy);
    if (err != ESP_OK) {
        rmt_del_encoder(enc->bytes);
        free(enc);
        return err;
    }

    enc->reset_code.level0    = 0;
    enc->reset_code.duration0 = WS2812_RESET_TICKS / 2;
    enc->reset_code.level1    = 0;
    enc->reset_code.duration1 = WS2812_RESET_TICKS / 2;

    *ret_encoder = &enc->base;
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/* Light state                                                               */
/* ------------------------------------------------------------------------ */
typedef enum {
    COLOR_MODE_HS,
    COLOR_MODE_CT,
    COLOR_MODE_XY,
} color_mode_t;

static rmt_channel_handle_t s_chan;
static rmt_encoder_handle_t s_encoder;
static SemaphoreHandle_t    s_lock;
static uint8_t              s_pixels[LAMP_LED_COUNT * 3];   /* GRB wire bytes */
static uint8_t              s_rgb[LAMP_LED_COUNT][3];       /* logical RGB frame */

static bool         s_power;
static uint8_t      s_level  = 100;          /* 0-100 */
static HS_color_t   s_hs     = { 0, 0 };
static uint32_t     s_kelvin = 2700;
static XY_color_t   s_xy     = { 0, 0 };
static color_mode_t s_mode   = COLOR_MODE_CT;

static bool     s_limiting;
static int      s_last_ma;
static bool     s_effect;      /* moving rainbow running */
static uint16_t s_phase;       /* where the hue ramp currently sits */

/* Caller must hold s_lock. Applies the current limiter to whatever is in
   s_rgb and pushes it out. Works for a whole frame, so the rainbow is limited
   on its real content rather than on a worst case. */
static esp_err_t frame_push(void)
{
    if (!s_chan) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t sum = 0;
    for (int i = 0; i < LAMP_LED_COUNT; i++) {
        sum += (uint32_t)s_rgb[i][0] + s_rgb[i][1] + s_rgb[i][2];
    }
    uint32_t lit_ma  = sum * MA_PER_CHANNEL / 255;
    uint32_t idle_ma = LAMP_LED_COUNT;
    uint32_t scale   = 256;                  /* Q8 */

    if (LAMP_MAX_MA > 0 && lit_ma + idle_ma > (uint32_t)LAMP_MAX_MA) {
        uint32_t budget = ((uint32_t)LAMP_MAX_MA > idle_ma) ? (LAMP_MAX_MA - idle_ma) : 0;
        scale = lit_ma ? (budget * 256) / lit_ma : 256;
        if (scale > 256) {
            scale = 256;
        }
    }

    bool limiting = (scale < 256);
    if (limiting != s_limiting) {
        s_limiting = limiting;
        ESP_LOGI(TAG, "current limiter %s (%lu mA wanted, %d mA budget)",
                 limiting ? "engaged" : "released",
                 (unsigned long)(lit_ma + idle_ma), LAMP_MAX_MA);
    }
    s_last_ma = (int)((lit_ma * scale) / 256 + idle_ma);

    for (int i = 0; i < LAMP_LED_COUNT; i++) {
        s_pixels[i * 3 + 0] = (uint8_t)(((uint32_t)s_rgb[i][1] * scale) >> 8);   /* G */
        s_pixels[i * 3 + 1] = (uint8_t)(((uint32_t)s_rgb[i][0] * scale) >> 8);   /* R */
        s_pixels[i * 3 + 2] = (uint8_t)(((uint32_t)s_rgb[i][2] * scale) >> 8);   /* B */
    }

    rmt_transmit_config_t tx_cfg = { .loop_count = 0 };
    esp_err_t err = rmt_transmit(s_chan, s_encoder, s_pixels, sizeof(s_pixels), &tx_cfg);
    if (err != ESP_OK) {
        return err;
    }
    /* Generous: a frame takes about 1.2 ms, but the RMT ISR can be stalled far
       longer than that by an NVS commit writing flash. */
    return rmt_tx_wait_all_done(s_chan, pdMS_TO_TICKS(1000));
}

static void fill_uniform(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < LAMP_LED_COUNT; i++) {
        s_rgb[i][0] = r;
        s_rgb[i][1] = g;
        s_rgb[i][2] = b;
    }
}

/* Caller must hold s_lock. A full hue wheel spread along the strip and rotated
   by s_phase, so the colours travel. Brightness and on/off still apply. */
static esp_err_t render_rainbow(void)
{
    uint8_t level = s_power ? s_level : 0;

    for (int i = 0; i < LAMP_LED_COUNT; i++) {
        HS_color_t hs;
        hs.hue        = (uint16_t)((s_phase + (i * 360) / LAMP_LED_COUNT) % 360);
        hs.saturation = 100;

        RGB_color_t rgb;
        hsv_to_rgb(hs, level, &rgb);
        s_rgb[i][0] = rgb.red;
        s_rgb[i][1] = rgb.green;
        s_rgb[i][2] = rgb.blue;
    }
    return frame_push();
}

/* Caller must hold s_lock. */
static esp_err_t render(void)
{
    /* While the effect runs it owns the strip, but power and brightness
       changes still have to take effect, so re-render it here too. */
    if (s_effect) {
        return render_rainbow();
    }

    RGB_color_t rgb = { 0, 0, 0 };
    uint8_t level = s_power ? s_level : 0;

    switch (s_mode) {
    case COLOR_MODE_XY:
        /* xy_to_rgb takes brightness on a 0-255 scale, unlike hsv_to_rgb. */
        xy_to_rgb(s_xy, (uint8_t)((uint16_t)level * 255 / 100), &rgb);
        break;
    case COLOR_MODE_CT: {
        HS_color_t hs;
        temp_to_hs(s_kelvin, &hs);
        hsv_to_rgb(hs, level, &rgb);
        break;
    }
    case COLOR_MODE_HS:
    default:
        hsv_to_rgb(s_hs, level, &rgb);
        break;
    }

    /* One line per colour change, so a misbehaving colour can be traced to
       either the values arriving from Matter or the conversion here. Kept at
       DEBUG so it costs nothing normally - raise the log level for this tag to
       see it. Skipped while the effect runs, which would otherwise log 25 times
       a second. */
    if (!s_effect) {
        ESP_LOGD(TAG, "render mode=%s power=%d level=%d hue=%d sat=%d ct=%luK -> rgb %d,%d,%d",
                 (s_mode == COLOR_MODE_HS) ? "hs" : (s_mode == COLOR_MODE_CT) ? "ct" : "xy",
                 (int)s_power, (int)s_level, (int)s_hs.hue, (int)s_hs.saturation,
                 (unsigned long)s_kelvin, (int)rgb.red, (int)rgb.green, (int)rgb.blue);
    }

    fill_uniform(rgb.red, rgb.green, rgb.blue);
    return frame_push();
}

static void effect_task(void *arg)
{
    while (1) {
        if (!s_effect) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (s_effect) {                       /* may have been turned off */
            s_phase = (uint16_t)((s_phase + LAMP_EFFECT_HUE_STEP) % 360);
            render_rainbow();
        }
        xSemaphoreGive(s_lock);
        vTaskDelay(pdMS_TO_TICKS(LAMP_EFFECT_STEP_MS));
    }
}

esp_err_t strip_driver_set_effect(bool enabled)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_effect = enabled;
    /* Turning it off has to put the Matter colour back straight away;
       turning it on can wait for the next tick of the effect task. */
    esp_err_t err = enabled ? render_rainbow() : render();
    xSemaphoreGive(s_lock);

    ESP_LOGI(TAG, "rainbow effect %s", enabled ? "on" : "off");
    return err;
}

esp_err_t strip_driver_indicate(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < 3; i++) {
        fill_uniform(255, 0, 0);
        frame_push();
        vTaskDelay(pdMS_TO_TICKS(120));
        fill_uniform(0, 0, 0);
        frame_push();
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    /* Put back whatever the lamp is supposed to be showing. */
    esp_err_t err = render();
    xSemaphoreGive(s_lock);
    return err;
}

bool strip_driver_get_effect(void)
{
    return s_effect;
}

esp_err_t strip_driver_init(void)
{
    if (s_chan) {
        return ESP_OK;
    }
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) {
        return ESP_ERR_NO_MEM;
    }

    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num          = LAMP_LED_GPIO,
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = WS2812_RESOLUTION_HZ,
        .mem_block_symbols = RMT_MEM_SYMBOLS,
        .trans_queue_depth = 4,
    };
    esp_err_t err = rmt_new_tx_channel(&tx_cfg, &s_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_tx_channel failed: %s", esp_err_to_name(err));
        s_chan = NULL;
        return err;
    }

    err = ws2812_encoder_new(&s_encoder);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ws2812 encoder failed: %s", esp_err_to_name(err));
        rmt_del_channel(s_chan);
        s_chan = NULL;
        return err;
    }

    err = rmt_enable(s_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_enable failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Strongest pad drive: the run from the board to the strip is longer than
       the original in-lamp harness. */
    gpio_set_drive_capability(LAMP_LED_GPIO, GPIO_DRIVE_CAP_3);

    /* Start dark so the strip is not left showing whatever it powered up with. */
    xSemaphoreTake(s_lock, portMAX_DELAY);
    fill_uniform(0, 0, 0);
    frame_push();
    xSemaphoreGive(s_lock);

    if (xTaskCreate(effect_task, "lamp_effect", 3072, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "could not start the effect task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "%d x WS2812 on gpio %d, current budget %d mA",
             LAMP_LED_COUNT, LAMP_LED_GPIO, LAMP_MAX_MA);
    return ESP_OK;
}

esp_err_t strip_driver_set_power(bool power)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_power = power;
    esp_err_t err = render();
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t strip_driver_set_brightness(uint8_t percent)
{
    if (percent > 100) {
        percent = 100;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    /* Matter uses level 0 as a way of expressing off; keep the remembered level
       so the light comes back at the brightness it had. */
    if (percent != 0) {
        s_level = percent;
    }
    esp_err_t err = render();
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t strip_driver_set_hue(uint16_t hue)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_hs.hue = hue;
    s_mode   = COLOR_MODE_HS;
    esp_err_t err = render();
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t strip_driver_set_saturation(uint8_t saturation)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_hs.saturation = saturation;
    s_mode          = COLOR_MODE_HS;
    esp_err_t err = render();
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t strip_driver_set_temperature(uint32_t kelvin)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_kelvin = kelvin;
    s_mode   = COLOR_MODE_CT;
    esp_err_t err = render();
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t strip_driver_set_xy(uint16_t x, uint16_t y)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_xy.x = x;
    s_xy.y = y;
    s_mode = COLOR_MODE_XY;
    esp_err_t err = render();
    xSemaphoreGive(s_lock);
    return err;
}

int strip_driver_last_ma(void)
{
    return s_last_ma;
}
