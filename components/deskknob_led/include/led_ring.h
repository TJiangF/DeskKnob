/*
 * WS2812 LED ring (8 px) with a few simple animated modes.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LED_MODE_OFF = 0,
    LED_MODE_SOLID,
    LED_MODE_RAINBOW,
    LED_MODE_BREATHE,
    LED_MODE_PROGRESS,
    LED_MODE_COUNT
} led_mode_t;

esp_err_t led_ring_init(void);          /* apply persisted settings */
esp_err_t led_ring_start_task(void);

void led_ring_set_mode(led_mode_t mode);
led_mode_t led_ring_get_mode(void);

void led_ring_set_brightness(int percent); /* 0..100, persisted */
int  led_ring_get_brightness(void);

void led_ring_set_color(uint8_t r, uint8_t g, uint8_t b); /* persisted */
void led_ring_get_color(uint8_t *r, uint8_t *g, uint8_t *b);

void led_ring_set_progress(float progress); /* 0..1 for LED_MODE_PROGRESS */
void led_ring_refresh(void);

#ifdef __cplusplus
}
#endif
