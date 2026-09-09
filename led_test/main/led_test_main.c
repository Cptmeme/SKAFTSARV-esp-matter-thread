/*
 * Addressable-LED wiring / solder-joint test for an ESP32-C6 SuperMini
 * driving the IKEA SKAFTSARV strip. Data line defaults to GPIO14.
 *
 * Everything (pin, LED count, chip model, colour order, brightness) can be
 * changed from the serial console at runtime, so you can keep probing joints
 * without reflashing. Type "help" in the monitor for the command list.
 */

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "led_strip.h"

static const char *TAG = "ledtest";

/* ------------------------------------------------------------------------ */
/* Defaults                                                                  */
/* ------------------------------------------------------------------------ */
#define DEFAULT_DATA_GPIO   14   /* SKAFTSARV data wire                      */
#define DEFAULT_LED_COUNT   30   /* change with "count N" once you know it   */
#define DEFAULT_BRIGHTNESS  40   /* 0-255. Low on purpose: USB 5V is ~500 mA */
#define RMT_RESOLUTION_HZ   (10 * 1000 * 1000)

#define DEFAULT_BTN1_GPIO   19   /* SKAFTSARV buttons                        */
#define DEFAULT_BTN2_GPIO   18
#define BTN_POLL_MS         10
#define BTN_STABLE_SAMPLES  3    /* 30 ms of quiet before a state is trusted */

/* Current budget for the whole strip. WLED defaults to 850 mA and scales the
   output down to fit; we do the same, because a starved WS2812 chain looks
   exactly like bad soldering: green and blue die before red, and once a chip
   browns out it stops passing data so everything after it goes dark too. */
#define DEFAULT_MAX_MA      850

/* Button brightness control: step size, and auto-repeat while held. */
#define BRIGHT_STEP         8
#define BRIGHT_HOLD_MS      600
#define BRIGHT_REPEAT_MS    200

typedef enum {
    MODE_IDLE,      /* console wrote the pixels; leave them alone */
    MODE_AUTO,      /* full diagnostic sequence, looping          */
    MODE_WALK,      /* one lit pixel crawling down the strip      */
    MODE_CHASE,
    MODE_RAINBOW,
    MODE_PROBE,     /* data pin as plain GPIO square wave         */
    MODE_BUTTON,    /* LEDs mirror the two buttons               */
    MODE_SWEEP,     /* walk the current budget up until it fails */
} test_mode_t;

/* ------------------------------------------------------------------------ */
/* State                                                                     */
/* ------------------------------------------------------------------------ */
static led_strip_handle_t s_strip;
static SemaphoreHandle_t  s_lock;

static int         s_gpio   = DEFAULT_DATA_GPIO;
static int         s_count  = DEFAULT_LED_COUNT;
static int         s_bright = DEFAULT_BRIGHTNESS;
static led_model_t s_model  = LED_MODEL_WS2812;
static char        s_order[5] = "grb";
static led_color_component_format_t s_fmt = LED_STRIP_COLOR_COMPONENT_FMT_GRB;

static int      s_walk_ms  = 120;
static int      s_probe_hz = 2;

static int      s_max_ma   = DEFAULT_MAX_MA;
static int      s_sweep_from  = 400;
static int      s_sweep_to    = 2000;
static int      s_sweep_step  = 100;
static int      s_sweep_dwell = 2500;
static uint8_t *s_fb;        /* r,g,b,w per pixel, before brightness/limiter */
static int      s_fb_leds;
static int      s_last_ma;
static bool     s_limiting;

static volatile test_mode_t s_mode = MODE_AUTO;
static volatile uint32_t    s_seq;   /* bumped on every mode change so a
                                        running pattern aborts promptly */

#define LOCK()   xSemaphoreTake(s_lock, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(s_lock)

/* ------------------------------------------------------------------------ */
/* Strip lifecycle                                                           */
/* ------------------------------------------------------------------------ */
static const char *model_name(led_model_t m)
{
    switch (m) {
    case LED_MODEL_WS2812: return "ws2812";
    case LED_MODEL_SK6812: return "sk6812";
    case LED_MODEL_WS2811: return "ws2811";
    case LED_MODEL_WS2816: return "ws2816";
    default:               return "?";
    }
}

/* Caller must hold s_lock. */
static void strip_destroy(void)
{
    if (s_strip) {
        led_strip_del(s_strip);
        s_strip = NULL;
    }
    free(s_fb);
    s_fb      = NULL;
    s_fb_leds = 0;
}

/* Caller must hold s_lock. */
static esp_err_t strip_create(void)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num         = s_gpio,
        .max_leds               = s_count,
        .led_model              = s_model,
        .color_component_format = s_fmt,
        .flags = { .invert_out = false },
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = RMT_RESOLUTION_HZ,
        /* The driver defaults to one 48-symbol block on the C6. Claiming two
            doubles the time the ISR has to refill, so a late interrupt cannot
            tear the frame and strand the tail of the strip. */
        .mem_block_symbols = 96,
        .flags = { .with_dma = false },
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip);
    if (err != ESP_OK) {
        s_strip = NULL;
        ESP_LOGE(TAG, "led_strip_new_rmt_device failed: %s", esp_err_to_name(err));
        return err;
    }
    led_strip_clear(s_strip);

    s_fb = calloc((size_t)s_count, 4);
    s_fb_leds = s_fb ? s_count : 0;
    if (!s_fb) {
        ESP_LOGE(TAG, "out of memory for a %d pixel frame buffer", s_count);
    }

    /* Strongest pad drive: sharper edges on a long bench data wire. */
    gpio_set_drive_capability(s_gpio, GPIO_DRIVE_CAP_3);

    ESP_LOGI(TAG, "strip ready: gpio=%d leds=%d model=%s order=%s brightness=%d budget=%d mA",
             s_gpio, s_count, model_name(s_model), s_order, s_bright, s_max_ma);
    return ESP_OK;
}

static void strip_rebuild(void)
{
    LOCK();
    strip_destroy();
    gpio_reset_pin(s_gpio);
    strip_create();
    UNLOCK();
}

/* ------------------------------------------------------------------------ */
/* Pixel helpers (all brightness-scaled)                                     */
/* ------------------------------------------------------------------------ */
static inline uint32_t sc(uint32_t v)
{
    return (v * (uint32_t)s_bright + 127) / 255;
}

/* Worst case (all channels on) for the current count and brightness cap.
   ~20 mA per colour channel per LED, plus ~1 mA of quiescent per chip. */
static int estimated_ma(void)
{
    int per_led = 20 * (int)s_fmt.format.num_components;
    return (s_count * per_led * s_bright) / 255 + s_count;
}

/* Patterns draw into this buffer; frame_push() applies the brightness cap and
   the current limiter on the way out. Caller must hold s_lock. */
static void fb_clear(void)
{
    if (s_fb) {
        memset(s_fb, 0, (size_t)s_fb_leds * 4);
    }
}

static void fb_set(int i, uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    if (!s_fb || i < 0 || i >= s_fb_leds) {
        return;
    }
    s_fb[i * 4 + 0] = r;
    s_fb[i * 4 + 1] = g;
    s_fb[i * 4 + 2] = b;
    s_fb[i * 4 + 3] = w;
}

static void fb_fill(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    for (int i = 0; i < s_count; i++) {
        fb_set(i, r, g, b, w);
    }
}

/* Caller must hold s_lock. */
static void frame_push(void)
{
    if (!s_strip || !s_fb) {
        return;
    }
    int comps = (int)s_fmt.format.num_components;

    /* What this frame asks for, in channel units of 0-255, after brightness. */
    uint32_t sum = 0;
    for (int i = 0; i < s_count; i++) {
        for (int c = 0; c < comps; c++) {
            sum += sc(s_fb[i * 4 + c]);
        }
    }
    uint32_t lit_ma  = (sum * 20 + 127) / 255;   /* ~20 mA per lit channel */
    uint32_t idle_ma = (uint32_t)s_count;        /* ~1 mA per chip, not dimmable */
    uint32_t scale   = 256;                      /* Q8 */

    if (s_max_ma > 0 && lit_ma + idle_ma > (uint32_t)s_max_ma) {
        uint32_t budget = ((uint32_t)s_max_ma > idle_ma) ? (s_max_ma - idle_ma) : 0;
        scale = lit_ma ? (budget * 256) / lit_ma : 256;
        if (scale > 256) {
            scale = 256;
        }
    }

    bool limiting = (scale < 256);
    if (limiting != s_limiting) {
        s_limiting = limiting;
        if (limiting) {
            ESP_LOGW(TAG, "current limiter engaged: frame wants ~%lu mA against a "
                          "%d mA budget, scaling output to %lu%%",
                     (unsigned long)(lit_ma + idle_ma), s_max_ma,
                     (unsigned long)(scale * 100 / 256));
        } else {
            ESP_LOGI(TAG, "current limiter released");
        }
    }
    s_last_ma = (int)((lit_ma * scale) / 256 + idle_ma);

    for (int i = 0; i < s_count; i++) {
        uint32_t v[4];
        for (int c = 0; c < 4; c++) {
            v[c] = (sc(s_fb[i * 4 + c]) * scale) >> 8;
        }
        if (comps == 4) {
            led_strip_set_pixel_rgbw(s_strip, i, v[0], v[1], v[2], v[3]);
        } else {
            led_strip_set_pixel(s_strip, i, v[0], v[1], v[2]);
        }
    }
    led_strip_refresh(s_strip);
}

static void hsv_to_rgb(uint16_t h, uint8_t sat, uint8_t val,
                       uint8_t *r, uint8_t *g, uint8_t *b)
{
    h %= 360;
    uint8_t  region = h / 60;
    uint32_t rem    = (h % 60) * 255 / 60;
    uint8_t  p = (val * (255 - sat)) / 255;
    uint8_t  q = (val * (255 - (sat * rem) / 255)) / 255;
    uint8_t  t = (val * (255 - (sat * (255 - rem)) / 255)) / 255;

    switch (region) {
    case 0:  *r = val; *g = t;   *b = p;   break;
    case 1:  *r = q;   *g = val; *b = p;   break;
    case 2:  *r = p;   *g = val; *b = t;   break;
    case 3:  *r = p;   *g = q;   *b = val; break;
    case 4:  *r = t;   *g = p;   *b = val; break;
    default: *r = val; *g = p;   *b = q;   break;
    }
}

/* Re-send whatever is already in the frame buffer. Lets 'bright' and 'maxma'
   take effect immediately on a static frame instead of waiting for a pattern. */
static void frame_repush(void)
{
    LOCK();
    frame_push();
    UNLOCK();
}

static void show_clear(void)
{
    LOCK();
    fb_clear();
    frame_push();
    UNLOCK();
}

static void show_solid(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    LOCK();
    fb_fill(r, g, b, w);
    frame_push();
    UNLOCK();
}

static void show_only(int index, uint8_t r, uint8_t g, uint8_t b)
{
    LOCK();
    fb_clear();
    fb_set(index, r, g, b, 0);
    frame_push();
    UNLOCK();
}

/* Sleep in small slices, aborting as soon as the console changes the mode. */
static bool hold(uint32_t seq, int ms)
{
    while (ms > 0) {
        if (s_seq != seq) {
            return false;
        }
        int chunk = ms > 25 ? 25 : ms;
        vTaskDelay(pdMS_TO_TICKS(chunk));
        ms -= chunk;
    }
    return s_seq == seq;
}

static void set_mode(test_mode_t mode)
{
    s_mode = mode;
    s_seq++;
}

/* ------------------------------------------------------------------------ */
/* Patterns                                                                  */
/* ------------------------------------------------------------------------ */
static bool run_walk(uint32_t seq)
{
    ESP_LOGI(TAG, "walk: one white pixel, 0 -> %d. Note the index where it "
                  "stops moving - that is where the chain breaks.", s_count - 1);
    for (int i = 0; i < s_count; i++) {
        show_only(i, 255, 255, 255);
        if (s_count <= 60 || (i % 5) == 0) {
            ESP_LOGI(TAG, "  pixel %d", i);
        }
        if (!hold(seq, s_walk_ms)) {
            return false;
        }
    }
    return true;
}

static bool run_chase(uint32_t seq, int rounds)
{
    for (int r = 0; r < rounds; r++) {
        for (int head = 0; head < s_count; head++) {
            LOCK();
            for (int i = 0; i < s_count; i++) {
                int d = head - i;
                uint8_t v = (d == 0) ? 255 : (d == 1) ? 90 : (d == 2) ? 30 : 0;
                fb_set(i, v, v, v, 0);
            }
            frame_push();
            UNLOCK();
            if (!hold(seq, 40)) {
                return false;
            }
        }
    }
    return true;
}

static bool run_rainbow(uint32_t seq, int ms)
{
    static uint16_t phase;
    int elapsed = 0;
    while (elapsed < ms) {
        LOCK();
        for (int i = 0; i < s_count; i++) {
            uint16_t hue = (phase + (i * 360) / (s_count ? s_count : 1)) % 360;
            uint8_t r, g, b;
            hsv_to_rgb(hue, 255, 255, &r, &g, &b);
            fb_set(i, r, g, b, 0);
        }
        frame_push();
        UNLOCK();
        phase = (phase + 4) % 360;
        if (!hold(seq, 30)) {
            return false;
        }
        elapsed += 30;
    }
    return true;
}

/* Hold every LED at white and walk the current budget upwards. Because the
   limiter scales the frame to fit the budget, the budget IS the current draw -
   so this is a current dial, which is the variable that actually fails. */
static bool run_sweep(uint32_t seq)
{
    s_bright = 255;
    ESP_LOGW(TAG, "sweep: all %d LEDs white, budget %d -> %d mA in steps of %d. "
                  "Watch the FAR END: when it dims, drifts orange/red or flickers, "
                  "the step before that is your ceiling.",
             s_count, s_sweep_from, s_sweep_to, s_sweep_step);

    for (int ma = s_sweep_from; ma <= s_sweep_to; ma += s_sweep_step) {
        s_max_ma   = ma;
        s_limiting = false;
        LOCK();
        fb_fill(255, 255, 255, 255);
        frame_push();
        UNLOCK();
        ESP_LOGW(TAG, "  budget %4d mA  ->  frame draws ~%d mA", ma, s_last_ma);
        if (!hold(seq, s_sweep_dwell)) {
            return false;
        }
    }

    ESP_LOGW(TAG, "sweep finished at %d mA. Settle on a safe value with "
                  "'maxma <mA>', or 'off' to stop.", s_max_ma);
    set_mode(MODE_IDLE);
    return true;
}

static bool run_auto(uint32_t seq)
{
    ESP_LOGI(TAG, "===== auto sequence (gpio %d, %d leds) =====", s_gpio, s_count);

    ESP_LOGI(TAG, "[1/6] all OFF - every LED must be dark. A stuck-on or "
                  "flickering LED here means noise on the data line.");
    show_clear();
    if (!hold(seq, 1500)) return false;

    ESP_LOGI(TAG, "[2/6] pixel 0 only: RED, then GREEN, then BLUE. If the "
                  "colours are swapped the chip order is not %s -> try 'order rgb'.",
             s_order);
    const uint8_t first[3][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}};
    for (int i = 0; i < 3; i++) {
        show_only(0, first[i][0], first[i][1], first[i][2]);
        if (!hold(seq, 900)) return false;
    }

    ESP_LOGI(TAG, "[3/6] walking pixel - counts the strip and finds breaks");
    if (!run_walk(seq)) return false;

    ESP_LOGI(TAG, "[4/6] solid RED / GREEN / BLUE - look for dark or "
                  "wrong-coloured LEDs (a dead LED kills everything after it)");
    for (int i = 0; i < 3; i++) {
        show_solid(first[i][0], first[i][1], first[i][2], 0);
        if (!hold(seq, 1200)) return false;
    }

    ESP_LOGI(TAG, "[5/6] white fade - the current test. Flicker, a colour "
                  "shift towards the far end, or a reboot here is a POWER "
                  "problem (supply/ground), not a data problem.");
    for (int v = 0; v <= 255; v += 5) {
        show_solid(v, v, v, v);
        if (!hold(seq, 12)) return false;
    }
    for (int v = 255; v >= 0; v -= 5) {
        show_solid(v, v, v, v);
        if (!hold(seq, 12)) return false;
    }

    ESP_LOGI(TAG, "[6/6] rainbow - all three channels driven at once");
    if (!run_rainbow(seq, 4000)) return false;

    show_clear();
    return hold(seq, 800);
}

/* ------------------------------------------------------------------------ */
/* Buttons                                                                   */
/* ------------------------------------------------------------------------ */
typedef struct {
    const char *name;
    int         gpio;
    int         raw;      /* last raw sample                              */
    int         stable;   /* debounced level                              */
    int         settled;  /* consecutive identical samples                */
    int         bounces;  /* raw edges seen since the last settled change */
    int64_t     down_us;
    int64_t     repeat_us;
    uint32_t    presses;
} button_t;

static button_t s_btn[2] = {
    { .name = "BTN1", .gpio = DEFAULT_BTN1_GPIO },
    { .name = "BTN2", .gpio = DEFAULT_BTN2_GPIO },
};
static int s_btn_active;   /* level the pin reads while the button is held */

static bool btn_pressed(const button_t *b)
{
    return b->stable == s_btn_active;
}

/* BTN1 brighter, BTN2 dimmer. An explicit press is allowed to push past the
   default current budget - finding the real ceiling is the whole point - but
   the budget is never dropped below the default behind your back. */
static void brightness_step(int delta)
{
    int v = s_bright + delta;
    if (v < 0)   v = 0;
    if (v > 255) v = 255;

    if (v == s_bright) {
        ESP_LOGW(TAG, "brightness already at %d/255", s_bright);
        return;
    }
    s_bright = v;

    int want = estimated_ma();
    if (s_max_ma > 0 && want > s_max_ma) {
        s_max_ma   = want;
        s_limiting = false;
    }
    frame_repush();

    ESP_LOGW(TAG, "brightness %3d/255  (%2d%%)  ->  all-on ~%4d mA, this frame ~%4d mA",
             s_bright, (s_bright * 100) / 255, estimated_ma(), s_last_ma);
}

static void buttons_configure(void)
{
    for (int i = 0; i < 2; i++) {
        button_t *b = &s_btn[i];
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << b->gpio,
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = (s_btn_active == 0) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
            .pull_down_en = (s_btn_active == 0) ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        if (gpio_config(&cfg) != ESP_OK) {
            ESP_LOGE(TAG, "%s: gpio %d cannot be used as an input", b->name, b->gpio);
            continue;
        }
        b->raw     = gpio_get_level(b->gpio);
        b->stable  = b->raw;
        b->settled = BTN_STABLE_SAMPLES;
        b->bounces = 0;
    }
    ESP_LOGI(TAG, "buttons: %s=gpio%d %s=gpio%d, pressed = %s, internal pull-%s",
             s_btn[0].name, s_btn[0].gpio, s_btn[1].name, s_btn[1].gpio,
             s_btn_active ? "HIGH" : "LOW", s_btn_active ? "down" : "up");
}

/* Polled rather than interrupt driven on purpose: polling lets us count the
   contact bounce, which is what tells a good joint from a cold one. */
static void button_task(void *arg)
{
    while (1) {
        for (int i = 0; i < 2; i++) {
            button_t *b = &s_btn[i];
            int raw = gpio_get_level(b->gpio);

            if (raw != b->raw) {
                b->raw     = raw;
                b->settled = 0;
                b->bounces++;
            } else if (b->settled < BTN_STABLE_SAMPLES) {
                if (++b->settled == BTN_STABLE_SAMPLES && raw != b->stable) {
                    b->stable = raw;
                    if (btn_pressed(b)) {
                        b->down_us   = esp_timer_get_time();
                        b->repeat_us = b->down_us;
                        b->presses++;
                        ESP_LOGI(TAG, "%s (gpio %d) PRESSED  - press #%lu, %d edge(s) of bounce",
                                 b->name, b->gpio, (unsigned long)b->presses, b->bounces);
                        if (s_mode != MODE_BUTTON) {
                            brightness_step(i == 0 ? BRIGHT_STEP : -BRIGHT_STEP);
                        }
                    } else {
                        int ms = (int)((esp_timer_get_time() - b->down_us) / 1000);
                        ESP_LOGI(TAG, "%s (gpio %d) released - held %d ms, %d edge(s) of bounce",
                                 b->name, b->gpio, ms, b->bounces);
                    }
                    b->bounces = 0;
                }
            }

            /* Auto-repeat: hold a button to ramp instead of tapping. */
            if (btn_pressed(b) && s_mode != MODE_BUTTON) {
                int64_t now = esp_timer_get_time();
                if (now - b->down_us   > BRIGHT_HOLD_MS * 1000 &&
                    now - b->repeat_us > BRIGHT_REPEAT_MS * 1000) {
                    b->repeat_us = now;
                    brightness_step(i == 0 ? BRIGHT_STEP : -BRIGHT_STEP);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(BTN_POLL_MS));
    }
}

static bool run_button_mirror(uint32_t seq)
{
    ESP_LOGI(TAG, "button mirror: hold %s -> first half green, hold %s -> second half blue "
                  "(pixel 0 stays dim white so you can see the mode is alive)",
             s_btn[0].name, s_btn[1].name);
    int half = s_count / 2;
    while (1) {
        bool one = btn_pressed(&s_btn[0]);
        bool two = btn_pressed(&s_btn[1]);
        LOCK();
        for (int i = 0; i < s_count; i++) {
            bool lit = (i < half) ? one : two;
            uint8_t r = 0, g = 0, b = 0;
            if (lit) {
                if (i < half) g = 255;
                else          b = 255;
            } else if (i == 0) {
                r = g = b = 60;
            }
            fb_set(i, r, g, b, 0);
        }
        frame_push();
        UNLOCK();
        if (!hold(seq, 50)) {
            return false;
        }
    }
}

static void pattern_task(void *arg)
{
    while (1) {
        uint32_t seq  = s_seq;
        test_mode_t m = s_mode;

        switch (m) {
        case MODE_AUTO:
            run_auto(seq);
            break;
        case MODE_WALK:
            run_walk(seq);
            break;
        case MODE_CHASE:
            run_chase(seq, 1);
            break;
        case MODE_RAINBOW:
            run_rainbow(seq, 3000);
            break;
        case MODE_PROBE: {
            int half = (s_probe_hz > 0) ? (500 / s_probe_hz) : 0;
            if (half <= 0) {
                gpio_set_level(s_gpio, 1);
                vTaskDelay(pdMS_TO_TICKS(100));
            } else {
                gpio_set_level(s_gpio, 1);
                if (!hold(seq, half)) break;
                gpio_set_level(s_gpio, 0);
                hold(seq, half);
            }
            break;
        }
        case MODE_BUTTON:
            run_button_mirror(seq);
            break;
        case MODE_SWEEP:
            run_sweep(seq);
            break;
        case MODE_IDLE:
        default:
            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Console commands                                                          */
/* ------------------------------------------------------------------------ */
static bool pin_is_sane(int gpio)
{
    if (!GPIO_IS_VALID_OUTPUT_GPIO(gpio)) {
        ESP_LOGE(TAG, "gpio %d is not a valid output pin on the esp32c6", gpio);
        return false;
    }
    if (gpio == 12 || gpio == 13) {
        ESP_LOGE(TAG, "gpio %d is the native USB port - that would kill this console", gpio);
        return false;
    }
    if (gpio >= 24 && gpio <= 30) {
        ESP_LOGE(TAG, "gpio %d is wired to the SPI flash - do not touch", gpio);
        return false;
    }
    return true;
}

/* Leave probe mode and make sure a strip object exists again. */
static void back_to_strip(void)
{
    LOCK();
    bool need = (s_strip == NULL);
    UNLOCK();
    if (need) {
        strip_rebuild();
    }
}

static int cmd_info(int argc, char **argv)
{
    printf("data gpio      : %d\n", s_gpio);
    printf("led count      : %d\n", s_count);
    printf("chip model     : %s\n", model_name(s_model));
    printf("colour order   : %s (%d bytes/pixel)\n", s_order, s_fmt.format.num_components);
    printf("brightness cap : %d/255\n", s_bright);
    printf("current budget : %d mA%s\n", s_max_ma,
           s_max_ma ? (s_limiting ? " (limiter ACTIVE)" : "") : " (limiter off)");
    printf("last frame     : ~%d mA\n", s_last_ma);
    printf("all-on draw    : ~%d mA at this count and brightness\n", estimated_ma());
    printf("walk step      : %d ms\n", s_walk_ms);
    printf("strip handle   : %s\n", s_strip ? "created" : "none (probe mode?)");
    printf("free heap      : %lu bytes\n", (unsigned long)esp_get_free_heap_size());
    return 0;
}

static int cmd_pin(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: pin <gpio>\n");
        return 1;
    }
    int gpio = atoi(argv[1]);
    if (!pin_is_sane(gpio)) {
        return 1;
    }
    set_mode(MODE_IDLE);
    LOCK();
    strip_destroy();
    gpio_reset_pin(s_gpio);
    s_gpio = gpio;
    strip_create();
    UNLOCK();
    set_mode(MODE_AUTO);
    return 0;
}

static int cmd_count(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: count <n>\n");
        return 1;
    }
    int n = atoi(argv[1]);
    if (n < 1 || n > 2000) {
        printf("count must be 1..2000\n");
        return 1;
    }
    set_mode(MODE_IDLE);
    s_count = n;
    strip_rebuild();
    set_mode(MODE_AUTO);
    return 0;
}

static int cmd_bright(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: bright <0-255>\n");
        return 1;
    }
    int v = atoi(argv[1]);
    if (v < 0 || v > 255) {
        printf("brightness must be 0..255\n");
        return 1;
    }
    s_bright = v;
    printf("brightness cap = %d, all-on draw ~%d mA, budget %d mA\n",
           s_bright, estimated_ma(), s_max_ma);
    if (s_max_ma > 0 && estimated_ma() > s_max_ma) {
        printf("note: the %d mA budget will clamp this - raise it with 'maxma' "
               "if the supply can take it\n", s_max_ma);
    }
    frame_repush();
    return 0;
}

static int cmd_model(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: model <ws2812|sk6812|ws2811|ws2816>\n");
        return 1;
    }
    led_model_t m;
    if (!strcasecmp(argv[1], "ws2812"))      m = LED_MODEL_WS2812;
    else if (!strcasecmp(argv[1], "sk6812")) m = LED_MODEL_SK6812;
    else if (!strcasecmp(argv[1], "ws2811")) m = LED_MODEL_WS2811;
    else if (!strcasecmp(argv[1], "ws2816")) m = LED_MODEL_WS2816;
    else {
        printf("unknown model '%s'\n", argv[1]);
        return 1;
    }
    set_mode(MODE_IDLE);
    s_model = m;
    strip_rebuild();
    set_mode(MODE_AUTO);
    return 0;
}

static int cmd_order(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: order <grb|rgb|brg|grbw|rgbw|...>\n");
        return 1;
    }
    const char *s = argv[1];
    size_t n = strlen(s);
    if (n != 3 && n != 4) {
        printf("order must be 3 or 4 letters from r/g/b/w\n");
        return 1;
    }
    int pos[4] = { -1, -1, -1, -1 };   /* r, g, b, w */
    char pretty[5] = {0};
    for (size_t i = 0; i < n; i++) {
        int idx;
        switch (tolower((unsigned char)s[i])) {
        case 'r': idx = 0; break;
        case 'g': idx = 1; break;
        case 'b': idx = 2; break;
        case 'w': idx = 3; break;
        default:
            printf("bad letter '%c'\n", s[i]);
            return 1;
        }
        if (pos[idx] >= 0) {
            printf("letter '%c' repeated\n", s[i]);
            return 1;
        }
        pos[idx]  = (int)i;
        pretty[i] = (char)tolower((unsigned char)s[i]);
    }
    if (pos[0] < 0 || pos[1] < 0 || pos[2] < 0) {
        printf("order needs r, g and b\n");
        return 1;
    }
    if (n == 3) {
        pos[3] = 3;
    } else if (pos[3] < 0) {
        printf("4-letter order needs a w\n");
        return 1;
    }

    led_color_component_format_t fmt = { .format_id = 0 };
    fmt.format.r_pos           = pos[0];
    fmt.format.g_pos           = pos[1];
    fmt.format.b_pos           = pos[2];
    fmt.format.w_pos           = pos[3];
    fmt.format.bytes_per_color = 1;
    fmt.format.num_components  = n;

    set_mode(MODE_IDLE);
    s_fmt = fmt;
    strlcpy(s_order, pretty, sizeof(s_order));
    strip_rebuild();
    set_mode(MODE_AUTO);
    return 0;
}

static int cmd_auto(int argc, char **argv)
{
    back_to_strip();
    set_mode(MODE_AUTO);
    return 0;
}

static int cmd_walk(int argc, char **argv)
{
    if (argc >= 2) {
        int ms = atoi(argv[1]);
        if (ms >= 5 && ms <= 5000) {
            s_walk_ms = ms;
        }
    }
    back_to_strip();
    set_mode(MODE_WALK);
    return 0;
}

static int cmd_chase(int argc, char **argv)
{
    back_to_strip();
    set_mode(MODE_CHASE);
    return 0;
}

static int cmd_rainbow(int argc, char **argv)
{
    back_to_strip();
    set_mode(MODE_RAINBOW);
    return 0;
}

static int cmd_solid(int argc, char **argv)
{
    if (argc < 4) {
        printf("usage: solid <r> <g> <b> [w]   (0-255 each, scaled by 'bright')\n");
        return 1;
    }
    back_to_strip();
    set_mode(MODE_IDLE);
    show_solid(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), argc > 4 ? atoi(argv[4]) : 0);
    return 0;
}

static int cmd_pixel(int argc, char **argv)
{
    if (argc < 5) {
        printf("usage: pixel <index> <r> <g> <b>\n");
        return 1;
    }
    back_to_strip();
    set_mode(MODE_IDLE);
    show_only(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
    return 0;
}

static int cmd_off(int argc, char **argv)
{
    back_to_strip();
    set_mode(MODE_IDLE);
    show_clear();
    return 0;
}

static int cmd_probe(int argc, char **argv)
{
    int hz = (argc >= 2) ? atoi(argv[1]) : 2;
    if (hz < 0 || hz > 500) {
        printf("usage: probe [hz]   (0 = hold the pin high)\n");
        return 1;
    }
    set_mode(MODE_IDLE);
    LOCK();
    strip_destroy();
    gpio_reset_pin(s_gpio);
    gpio_set_direction(s_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(s_gpio, 1);
    UNLOCK();
    s_probe_hz = hz;
    set_mode(MODE_PROBE);
    if (hz == 0) {
        printf("gpio %d held HIGH (3.3 V). Meter it at the far end of the data "
               "wire to check the joint.\n", s_gpio);
    } else {
        printf("gpio %d toggling at %d Hz. A meter on the far end should read "
               "about 1.6 V average.\n", s_gpio, hz);
    }
    printf("run 'auto' (or any pattern command) to go back to driving LEDs\n");
    return 0;
}

static int cmd_maxma(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: maxma <mA>   (0 disables the limiter)\n");
        return 1;
    }
    int ma = atoi(argv[1]);
    if (ma < 0 || ma > 20000) {
        printf("budget must be 0..20000 mA\n");
        return 1;
    }
    s_max_ma   = ma;
    s_limiting = false;
    if (ma == 0) {
        printf("limiter OFF - nothing holds the strip back now. %d LEDs all-on is "
               "~%d mA; make sure the supply can actually deliver it.\n",
               s_count, estimated_ma());
    } else {
        printf("current budget = %d mA (all-on wants ~%d mA)\n", ma, estimated_ma());
    }
    frame_repush();
    return 0;
}

static int cmd_tune(int argc, char **argv)
{
    back_to_strip();
    set_mode(MODE_IDLE);
    LOCK();
    fb_fill(255, 255, 255, 255);
    frame_push();
    UNLOCK();
    printf("all %d LEDs white at brightness %d/255 (~%d mA) - the worst case.\n",
           s_count, s_bright, estimated_ma());
    printf("BTN1 (gpio %d) brighter, BTN2 (gpio %d) dimmer. Hold to ramp.\n",
           s_btn[0].gpio, s_btn[1].gpio);
    printf("Stop when the far end dims, drifts orange or flickers - the value\n"
           "printed just before that is your ceiling.\n");
    return 0;
}

static int cmd_sweep(int argc, char **argv)
{
    if (argc >= 2) s_sweep_from = atoi(argv[1]);
    if (argc >= 3) s_sweep_to   = atoi(argv[2]);
    if (argc >= 4) s_sweep_step = atoi(argv[3]);

    if (s_sweep_from < 50 || s_sweep_to > 20000 || s_sweep_to < s_sweep_from ||
        s_sweep_step < 10) {
        printf("usage: sweep [from_ma] [to_ma] [step_ma]   (defaults 400 2000 100)\n");
        return 1;
    }
    back_to_strip();
    set_mode(MODE_SWEEP);
    return 0;
}

static int cmd_btn(int argc, char **argv)
{
    printf("pressed level  : %s\n", s_btn_active ? "HIGH (1)" : "LOW (0)");
    for (int i = 0; i < 2; i++) {
        button_t *b = &s_btn[i];
        printf("%s  gpio %-2d  raw=%d  %-8s  presses=%lu\n",
               b->name, b->gpio, gpio_get_level(b->gpio),
               btn_pressed(b) ? "PRESSED" : "released",
               (unsigned long)b->presses);
    }
    printf("an open (unsoldered) line sits at raw=%d for ever; a shorted one at raw=%d\n",
           s_btn_active ? 0 : 1, s_btn_active);
    return 0;
}

static int cmd_btnpin(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: btnpin <1|2> <gpio>\n");
        return 1;
    }
    int idx  = atoi(argv[1]) - 1;
    int gpio = atoi(argv[2]);
    if (idx < 0 || idx > 1) {
        printf("button must be 1 or 2\n");
        return 1;
    }
    if (!pin_is_sane(gpio)) {
        return 1;
    }
    if (gpio == s_gpio) {
        printf("gpio %d is the LED data line\n", gpio);
        return 1;
    }
    gpio_reset_pin(s_btn[idx].gpio);
    s_btn[idx].gpio    = gpio;
    s_btn[idx].presses = 0;
    buttons_configure();
    return 0;
}

static int cmd_btnactive(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: btnactive <0|1>   (0 = the switch pulls the pin to GND)\n");
        return 1;
    }
    int level = atoi(argv[1]);
    if (level != 0 && level != 1) {
        printf("level must be 0 or 1\n");
        return 1;
    }
    s_btn_active = level;
    buttons_configure();
    return 0;
}

static int cmd_buttons(int argc, char **argv)
{
    back_to_strip();
    set_mode(MODE_BUTTON);
    return 0;
}

static void register_cmd(const char *cmd, const char *help, const char *hint,
                         esp_console_cmd_func_t func)
{
    const esp_console_cmd_t c = {
        .command = cmd,
        .help    = help,
        .hint    = hint,
        .func    = func,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&c));
}

static void console_start(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt             = "led>";
    repl_config.max_cmdline_length = 128;

    register_cmd("info",    "show the current configuration",                NULL,             cmd_info);
    register_cmd("pin",     "move the data line to another gpio",            "<gpio>",         cmd_pin);
    register_cmd("count",   "set the number of LEDs",                        "<n>",            cmd_count);
    register_cmd("bright",  "brightness cap, 0-255 (keep it low on USB)",    "<0-255>",        cmd_bright);
    register_cmd("model",   "LED chip timing",                               "<ws2812|sk6812|ws2811|ws2816>", cmd_model);
    register_cmd("order",   "colour byte order",                             "<grb|rgb|grbw|...>", cmd_order);
    register_cmd("auto",    "run the full diagnostic sequence in a loop",    NULL,             cmd_auto);
    register_cmd("walk",    "crawl a single lit pixel down the strip",       "[step_ms]",      cmd_walk);
    register_cmd("chase",   "moving comet",                                  NULL,             cmd_chase);
    register_cmd("rainbow", "rotating rainbow",                              NULL,             cmd_rainbow);
    register_cmd("solid",   "set every LED to one colour",                   "<r> <g> <b> [w]", cmd_solid);
    register_cmd("pixel",   "light exactly one LED",                         "<i> <r> <g> <b>", cmd_pixel);
    register_cmd("off",     "all LEDs off",                                  NULL,             cmd_off);
    register_cmd("probe",   "drive the data pin as a plain square wave",     "[hz]",           cmd_probe);
    register_cmd("maxma",     "current budget in mA, 0 = off (WLED-style)", "<mA>",            cmd_maxma);
    register_cmd("sweep",     "ramp the budget up on white to find the limit","[from] [to] [step]", cmd_sweep);
    register_cmd("tune",      "all white, then set brightness with the buttons", NULL,           cmd_tune);
    register_cmd("btn",       "button status: levels, state, press counts",  NULL,              cmd_btn);
    register_cmd("btnpin",    "move a button to another gpio",              "<1|2> <gpio>",    cmd_btnpin);
    register_cmd("btnactive", "level the pin reads while pressed",          "<0|1>",           cmd_btnactive);
    register_cmd("buttons",   "LEDs mirror the buttons while held",         NULL,              cmd_buttons);

#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));
#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));
#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    esp_console_dev_usb_cdc_config_t hw_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&hw_config, &repl_config, &repl));
#else
#error "No usable console: enable USB Serial/JTAG or UART console in menuconfig"
#endif
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

/* ------------------------------------------------------------------------ */
void app_main(void)
{
    s_lock = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_lock ? ESP_OK : ESP_ERR_NO_MEM);

    printf("\n");
    printf("=========================================================\n");
    printf(" SKAFTSARV LED wiring test - ESP32-C6 SuperMini\n");
    printf(" LEDs   : gpio %d, %d x %s, order %s\n",
           s_gpio, s_count, model_name(s_model), s_order);
    printf(" Buttons: gpio %d and gpio %d, active low, internal pull-ups\n",
           s_btn[0].gpio, s_btn[1].gpio);
    printf("\n");
    printf(" BTN1 = brighter, BTN2 = dimmer (hold to ramp). Every step is\n");
    printf(" printed with its estimated current draw.\n");
    printf("\n");
    printf(" Type 'tune' for the worst case: all LEDs white, buttons set the\n");
    printf(" level. Otherwise the LED sequence starts on its own. 'help'\n");
    printf(" lists everything; 'count N' and 'pin N' take effect live.\n");
    printf("=========================================================\n\n");

    strip_rebuild();
    buttons_configure();

    xTaskCreate(pattern_task, "pattern", 4096, NULL, 4, NULL);
    xTaskCreate(button_task,  "buttons", 3072, NULL, 5, NULL);
    console_start();
}
