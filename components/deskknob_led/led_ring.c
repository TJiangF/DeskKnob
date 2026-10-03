#include "led_ring.h"
#include "board_pins.h"
#include "settings.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "led_strip.h"
#include "led_strip_rmt.h"

static const char *TAG = "led_ring";

static led_strip_handle_t s_strip;
static led_mode_t s_mode = LED_MODE_RAINBOW;
static int s_brightness = 40;
static uint8_t s_r = 255, s_g = 80, s_b = 0;
static float s_progress = 0.0f;
static uint32_t s_phase = 0;

static inline uint8_t scale8(uint8_t v, int pct)
{
    return (uint8_t)((uint32_t)v * (uint32_t)pct / 100u);
}

static void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t region = h / 60;
    uint8_t rem = (h - (region * 60)) * 6;
    uint8_t p = (uint8_t)((v * (255 - s)) >> 8);
    uint8_t q = (uint8_t)((v * (255 - ((s * rem) >> 8))) >> 8);
    uint8_t t = (uint8_t)((v * (255 - ((s * (255 - rem)) >> 8))) >> 8);
    switch (region) {
    case 0: *r = v; *g = t; *b = p; break;
    case 1: *r = q; *g = v; *b = p; break;
    case 2: *r = p; *g = v; *b = t; break;
    case 3: *r = p; *g = q; *b = v; break;
    case 4: *r = t; *g = p; *b = v; break;
    default: *r = v; *g = p; *b = q; break;
    }
}

void led_ring_refresh(void)
{
    if (!s_strip) {
        return;
    }

    int active = BOARD_LED_RING_NUM;
    if (s_mode == LED_MODE_PROGRESS) {
        active = (int)lroundf(s_progress * BOARD_LED_RING_NUM);
        if (active < 0) active = 0;
        if (active > BOARD_LED_RING_NUM) active = BOARD_LED_RING_NUM;
    }

    for (int i = 0; i < BOARD_LED_RING_NUM; i++) {
        uint8_t r = 0, g = 0, b = 0;
        bool lit = true;

        switch (s_mode) {
        case LED_MODE_OFF:
            lit = false;
            break;
        case LED_MODE_SOLID:
            r = s_r; g = s_g; b = s_b;
            break;
        case LED_MODE_RAINBOW: {
            uint16_t hue = (uint16_t)((i * (360 / BOARD_LED_RING_NUM) + s_phase) % 360);
            hsv_to_rgb(hue, 255, 255, &r, &g, &b);
            break;
        }
        case LED_MODE_BREATHE: {
            float s = (sinf((float)s_phase * (float)M_PI / 180.0f) + 1.0f) * 0.5f;
            r = scale8(s_r, (int)(s * 100));
            g = scale8(s_g, (int)(s * 100));
            b = scale8(s_b, (int)(s * 100));
            break;
        }
        case LED_MODE_PROGRESS:
            r = s_r; g = s_g; b = s_b;
            if (i >= active) {
                r /= 8; g /= 8; b /= 8;
            }
            break;
        default:
            lit = false;
            break;
        }

        if (!lit || s_brightness == 0) {
            led_strip_set_pixel(s_strip, i, 0, 0, 0);
        } else {
            led_strip_set_pixel(s_strip, i, scale8(r, s_brightness), scale8(g, s_brightness), scale8(b, s_brightness));
        }
    }
    led_strip_refresh(s_strip);
}

static void led_task(void *arg)
{
    (void)arg;
    int tick = 0;
    while (1) {
        s_phase = (uint32_t)(tick * 6) % 360;
        led_ring_refresh();
        tick++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

esp_err_t led_ring_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = BOARD_PIN_LED_RING,
        .max_leds = BOARD_LED_RING_NUM,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
    };
    ESP_RETURN_ON_ERROR(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip), TAG, "led strip");

    s_mode = (led_mode_t)settings_get_int(SET_LED_MODE, LED_MODE_RAINBOW);
    s_brightness = settings_get_int(SET_LED_BRIGHTNESS, 40);
    s_r = settings_get_int(SET_LED_COLOR_R, 255);
    s_g = settings_get_int(SET_LED_COLOR_G, 80);
    s_b = settings_get_int(SET_LED_COLOR_B, 0);

    ESP_LOGI(TAG, "led ring ready mode=%d bright=%d", s_mode, s_brightness);
    return ESP_OK;
}

esp_err_t led_ring_start_task(void)
{
    /* Keep LED animation on core 0 alongside the UI. */
    BaseType_t ok = xTaskCreatePinnedToCore(led_task, "led_ring", 3072, NULL, 3, NULL, 0);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void led_ring_set_mode(led_mode_t mode)
{
    s_mode = mode;
    settings_set_int(SET_LED_MODE, mode);
}

led_mode_t led_ring_get_mode(void) { return s_mode; }

void led_ring_set_brightness(int percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    s_brightness = percent;
    settings_set_int(SET_LED_BRIGHTNESS, percent);
}

int led_ring_get_brightness(void) { return s_brightness; }

void led_ring_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    s_r = r; s_g = g; s_b = b;
    settings_set_int(SET_LED_COLOR_R, r);
    settings_set_int(SET_LED_COLOR_G, g);
    settings_set_int(SET_LED_COLOR_B, b);
}

void led_ring_get_color(uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (r) *r = s_r;
    if (g) *g = s_g;
    if (b) *b = s_b;
}

void led_ring_set_progress(float progress)
{
    if (progress < 0) progress = 0;
    if (progress > 1) progress = 1;
    s_progress = progress;
}
