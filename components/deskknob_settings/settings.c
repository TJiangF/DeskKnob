#include "settings.h"

#include <string.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG = "settings";
static const char *NVS_NS = "deskknob";
static nvs_handle_t s_nvs;
static bool s_ready;

typedef struct {
    const char *key;
    int32_t def;
} setting_desc_t;

static const setting_desc_t s_desc[SET_SETTING_COUNT] = {
    [SET_LCD_BRIGHTNESS]   = {"lcd_bright",   80},
    [SET_LED_ENABLED]      = {"led_en",       1},
    [SET_LED_MODE]         = {"led_mode",     1},   /* 1 = rainbow */
    [SET_LED_BRIGHTNESS]   = {"led_bright",   40},
    [SET_LED_COLOR_R]      = {"led_r",        255},
    [SET_LED_COLOR_G]      = {"led_g",        255},
    [SET_LED_COLOR_B]      = {"led_b",        255},
    [SET_RATCHET_DETENTS]  = {"ratchet_n",    24},
    [SET_RATCHET_STIFFNESS] = {"ratchet_k",   50},  /* 5.0 */
    [SET_SALARY_RATE]      = {"sal_rate",     1000000}, /* monthly 10000.00 yuan */
    [SET_SALARY_HOURS]     = {"sal_hours",    80},    /* 8.0 h/day */
    [SET_SALARY_DAYS]      = {"sal_days",     218},   /* 21.8 d/month */
};

esp_err_t settings_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &s_nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }
    s_ready = true;
    ESP_LOGI(TAG, "settings ready");
    return ESP_OK;
}

int32_t settings_get_default(setting_id_t id)
{
    if (id < 0 || id >= SET_SETTING_COUNT) {
        return 0;
    }
    return s_desc[id].def;
}

int32_t settings_get_int(setting_id_t id, int32_t fallback)
{
    if (!s_ready || id < 0 || id >= SET_SETTING_COUNT) {
        return fallback;
    }
    int32_t v = fallback;
    if (nvs_get_i32(s_nvs, s_desc[id].key, &v) != ESP_OK) {
        v = fallback;
    }
    return v;
}

void settings_set_int(setting_id_t id, int32_t value)
{
    if (!s_ready || id < 0 || id >= SET_SETTING_COUNT) {
        return;
    }
    nvs_set_i32(s_nvs, s_desc[id].key, value);
    nvs_commit(s_nvs);
}

int32_t settings_get_int_key(const char *key, int32_t fallback)
{
    if (!s_ready || key == NULL) {
        return fallback;
    }
    int32_t v = fallback;
    if (nvs_get_i32(s_nvs, key, &v) != ESP_OK) {
        v = fallback;
    }
    return v;
}

void settings_set_int_key(const char *key, int32_t value)
{
    if (!s_ready || key == NULL) {
        return;
    }
    nvs_set_i32(s_nvs, key, value);
    nvs_commit(s_nvs);
}

esp_err_t settings_get_str(const char *key, char *out, size_t out_len, const char *fallback)
{
    if (out == NULL || out_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    out[0] = '\0';
    if (fallback) {
        strlcpy(out, fallback, out_len);
    }
    if (!s_ready || key == NULL) {
        return ESP_OK;
    }
    size_t len = out_len;
    esp_err_t err = nvs_get_str(s_nvs, key, out, &len);
    return err;
}

esp_err_t settings_set_str(const char *key, const char *value)
{
    if (!s_ready || key == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = nvs_set_str(s_nvs, key, value ? value : "");
    if (err == ESP_OK) {
        err = nvs_commit(s_nvs);
    }
    return err;
}
