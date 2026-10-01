#include "balance.h"
#include "settings.h"

#include <stdarg.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"

static const char *TAG = "balance";
#define BALANCE_URL "https://api.deepseek.com/user/balance"
#define BALANCE_KEY_NVS "deepseek_key"

static balance_info_t s_info;
static SemaphoreHandle_t s_lock;
static volatile bool s_request;

static void set_message(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_info.message, sizeof(s_info.message), fmt, ap);
    va_end(ap);
}

static esp_err_t http_event_cb(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        char *buf = (char *)evt->user_data;
        size_t used = strlen(buf);
        size_t cap = 2048;
        if (used + evt->data_len < cap) {
            memcpy(buf + used, evt->data, evt->data_len);
        }
    }
    return ESP_OK;
}

static void parse_response(const char *body)
{
    cJSON *root = cJSON_Parse(body);
    if (!root) {
        set_message("bad JSON");
        return;
    }
    cJSON *avail = cJSON_GetObjectItem(root, "is_available");
    if (cJSON_IsBool(avail)) {
        s_info.available = cJSON_IsTrue(avail);
    }
    cJSON *infos = cJSON_GetObjectItem(root, "balance_infos");
    if (cJSON_IsArray(infos) && cJSON_GetArraySize(infos) > 0) {
        cJSON *first = cJSON_GetArrayItem(infos, 0);
        cJSON *cur = cJSON_GetObjectItem(first, "currency");
        cJSON *tot = cJSON_GetObjectItem(first, "total_balance");
        if (cJSON_IsString(cur)) {
            strlcpy(s_info.currency, cur->valuestring, sizeof(s_info.currency));
        }
        if (cJSON_IsString(tot)) {
            s_info.total = atof(tot->valuestring);
        }
        s_info.valid = true;
        set_message("updated");
    } else {
        set_message("no balance data");
    }
    cJSON_Delete(root);
}

void balance_set_api_key(const char *key)
{
    settings_set_str(BALANCE_KEY_NVS, key ? key : "");
}

const char *balance_get_api_key(void)
{
    static char key[SETTINGS_STR_MAX];
    settings_get_str(BALANCE_KEY_NVS, key, sizeof(key), "");
    return key;
}

static void balance_task(void *arg)
{
    (void)arg;
    char *body = NULL;
    for (;;) {
        if (!s_request) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        s_request = false;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_info.busy = true;
        s_info.valid = false;
        xSemaphoreGive(s_lock);

        const char *key = balance_get_api_key();
        if (key[0] == '\0') {
            xSemaphoreTake(s_lock, portMAX_DELAY);
            s_info.busy = false;
            set_message("no API key");
            xSemaphoreGive(s_lock);
            continue;
        }

        if (!body) {
            body = calloc(1, 2048);
        }
        if (!body) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        body[0] = '\0';

        char auth[128];
        snprintf(auth, sizeof(auth), "Bearer %s", key);

        esp_http_client_config_t cfg = {
            .url = BALANCE_URL,
            .method = HTTP_METHOD_GET,
            .timeout_ms = 10000,
            .event_handler = http_event_cb,
            .user_data = body,
            .crt_bundle_attach = esp_crt_bundle_attach,
        };
        esp_http_client_handle_t client = esp_http_client_init(&cfg);
        esp_http_client_set_header(client, "Authorization", auth);
        esp_http_client_set_header(client, "Accept", "application/json");

        esp_err_t err = esp_http_client_perform(client);
        int status = esp_http_client_get_status_code(client);

        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_info.busy = false;
        if (err != ESP_OK) {
            set_message("http error");
        } else if (status != 200) {
            set_message("http %d", status);
        } else {
            parse_response(body);
        }
        xSemaphoreGive(s_lock);

        esp_http_client_cleanup(client);
    }
}

esp_err_t balance_service_init(void)
{
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
    }
    memset(&s_info, 0, sizeof(s_info));
    strlcpy(s_info.currency, "CNY", sizeof(s_info.currency));
    xTaskCreate(balance_task, "balance", 6144, NULL, 4, NULL);
    ESP_LOGI(TAG, "balance service ready");
    return ESP_OK;
}

void balance_request_refresh(void)
{
    s_request = true;
}

void balance_get(balance_info_t *out)
{
    if (!out) return;
    if (s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        *out = s_info;
        xSemaphoreGive(s_lock);
    } else {
        *out = s_info;
    }
}
