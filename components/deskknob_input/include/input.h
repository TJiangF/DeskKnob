/*
 * User input for DeskKnob.
 *
 * Two independent sources feed a single FreeRTOS event queue:
 *   - The HX710 film-pressure sensor (PRESS)   -> "confirm / select"
 *   - The side push button (BACK)              -> "back / exit"
 *
 * The pressure detection mirrors the original Arduino firmware: a rolling
 * baseline is measured at boot, a trigger threshold is derived from it, and a
 * press is only emitted after N consecutive samples stay above it.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    INPUT_EVT_NONE = 0,
    INPUT_EVT_PRESS,        /* pressure film pressed          */
    INPUT_EVT_BACK,         /* push button pressed            */
    INPUT_EVT_BACK_HOLD,    /* push button held > 1s          */
} input_event_t;

typedef struct {
    input_event_t type;
    int32_t value;      /* raw HX710 value for PRESS, else 0 */
} input_msg_t;

esp_err_t input_init(void);

/* Block for the next event (pdMS_TO_TICKS timeout). Returns false on timeout. */
bool input_wait(input_msg_t *out, uint32_t timeout_ms);

/* Latest pressure frame (milliseconds since boot) for diagnostics. */
int32_t input_pressure_raw(void);
int32_t input_pressure_baseline(void);
uint32_t input_pressure_last_ms(void);
bool input_is_pressed(void);

#ifdef __cplusplus
}
#endif
