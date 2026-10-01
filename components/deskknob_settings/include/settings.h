/*
 * Persistent scalar settings backed by NVS.
 *
 * All settings share the NVS namespace "deskknob". Wi-Fi credentials are kept
 * separately by deskknob_wifi (they need a list, not scalars).
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SETTINGS_STR_MAX 64

typedef enum {
    SET_LCD_BRIGHTNESS = 0,  /* int 0..100 */
    SET_LED_ENABLED,         /* int 0/1   */
    SET_LED_MODE,            /* int       */
    SET_LED_BRIGHTNESS,      /* int 0..100 */
    SET_LED_COLOR_R,         /* int 0..255 */
    SET_LED_COLOR_G,         /* int 0..255 */
    SET_LED_COLOR_B,         /* int 0..255 */
    SET_RATCHET_DETENTS,     /* int 2..60 */
    SET_RATCHET_STIFFNESS,   /* int x10   */
    SET_SALARY_RATE,         /* int  yuan/hour x100 */
    SET_SALARY_HOURS,        /* int  hours x10 */
    SET_SALARY_DAYS,         /* int  days x10 */
    SET_SETTING_COUNT
} setting_id_t;

esp_err_t settings_init(void);

/* Typed accessors with built-in defaults. */
int32_t settings_get_int(setting_id_t id, int32_t fallback);
void    settings_set_int(setting_id_t id, int32_t value);

int32_t settings_get_default(setting_id_t id);

/* String settings (e.g. API key). */
esp_err_t settings_get_str(const char *key, char *out, size_t out_len, const char *fallback);
esp_err_t settings_set_str(const char *key, const char *value);

/* Raw key/value helpers for keys outside the enum. */
int32_t settings_get_int_key(const char *key, int32_t fallback);
void    settings_set_int_key(const char *key, int32_t value);

#ifdef __cplusplus
}
#endif
