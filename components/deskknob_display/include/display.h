/*
 * Round GC9A01 LCD bring-up + LVGL port + backlight PWM.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t display_init(void);

/* Backlight brightness 0..100 (persisted via deskknob_settings on change). */
void display_set_brightness(int percent);
int  display_get_brightness(void);

lv_display_t *display_get_lv(void);
esp_lcd_panel_handle_t display_get_panel(void);

/* Debug: stop the LVGL refresh timer so direct panel writes are not overwritten. */
void display_pause_lvgl(void);

/* LVGL is not thread-safe: take this around any lv_* call outside the LVGL task. */
bool display_lock(uint32_t timeout_ms);
void display_unlock(void);

#ifdef __cplusplus
}
#endif
