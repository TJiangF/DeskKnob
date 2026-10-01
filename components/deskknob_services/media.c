#include "media.h"

#include "esp_log.h"

static const char *TAG = "media";

static const char *s_names[MEDIA_KEY_COUNT] = {
    [MEDIA_PLAY_PAUSE] = "PLAY_PAUSE",
    [MEDIA_NEXT_TRACK] = "NEXT_TRACK",
    [MEDIA_PREV_TRACK] = "PREV_TRACK",
    [MEDIA_MUTE] = "MUTE",
    [MEDIA_VOLUME_UP] = "VOLUME_UP",
    [MEDIA_VOLUME_DOWN] = "VOLUME_DOWN",
};

esp_err_t media_init(void)
{
    ESP_LOGI(TAG, "media service ready (BLE HID consumer control - stub)");
    return ESP_OK;
}

void media_send(media_key_t key)
{
    if (key < 0 || key >= MEDIA_KEY_COUNT) {
        return;
    }
    ESP_LOGI(TAG, "media key: %s", s_names[key]);
    /* TODO: emit a BLE HID consumer-control report over NimBLE. */
}

bool media_is_connected(void)
{
    return false;
}
