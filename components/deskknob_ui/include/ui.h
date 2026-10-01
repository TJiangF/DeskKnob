/*
 * DeskKnob UI subsystem.
 *
 * Starts a dedicated task that owns all interaction:
 *   - consumes input events (pressure = select, button = back)
 *   - observes motor gear changes (rotary navigation)
 *   - drives the LVGL screens with animated transitions
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ui_start(void);

#ifdef __cplusplus
}
#endif
