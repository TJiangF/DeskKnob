/*
 * Media control service (BLE HID consumer control).
 *
 * The UI calls media_send() for play/pause, next/previous track and mute.
 * The transport is intentionally abstracted so that a BLE HID implementation
 * can be dropped in without touching the UI.
 *
 * NOTE: media_init() is currently a stub - it captures the intent and logs.
 * See docs/roadmap in the README for the planned NimBLE HID implementation.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MEDIA_PLAY_PAUSE = 0,
    MEDIA_NEXT_TRACK,
    MEDIA_PREV_TRACK,
    MEDIA_MUTE,
    MEDIA_VOLUME_UP,
    MEDIA_VOLUME_DOWN,
    MEDIA_KEY_COUNT,
} media_key_t;

esp_err_t media_init(void);
void media_send(media_key_t key);
bool media_is_connected(void);

#ifdef __cplusplus
}
#endif
