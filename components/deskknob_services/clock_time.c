#include "clock_time.h"

#include <string.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "esp_event.h"
#include "esp_netif.h"

static const char *TAG = "clock";
#define TZ_INFO "CST-8"   /* China Standard Time (UTC+8) */

static bool s_started;

static void start_sntp_cb(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    if (id == IP_EVENT_STA_GOT_IP) {
        clock_time_start_sync();
    }
}

void clock_time_start_sync(void)
{
    if (s_started) {
        return;
    }
    s_started = true;

    setenv("TZ", TZ_INFO, 1);
    tzset();

    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    cfg.start = true;
    cfg.server_from_dhcp = true;
    cfg.renew_servers_after_new_IP = true;
    esp_err_t err = esp_netif_sntp_init(&cfg);
    if (err != ESP_OK) {
        s_started = false;
        ESP_LOGW(TAG, "sntp init failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "SNTP started");
}

esp_err_t clock_time_init(void)
{
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, start_sntp_cb, NULL, NULL);
    return ESP_OK;
}

bool clock_time_valid(void)
{
    time_t now = 0;
    time(&now);
    /* Anything after 2021 is considered a real synced time. */
    return now > 1600000000;
}

bool clock_time_hm(int *hour, int *minute)
{
    if (!clock_time_valid()) {
        return false;
    }
    time_t now;
    struct tm tm;
    time(&now);
    localtime_r(&now, &tm);
    if (hour) *hour = tm.tm_hour;
    if (minute) *minute = tm.tm_min;
    return true;
}

void clock_time_str(char *out, int out_len)
{
    int h, m;
    if (!out || out_len <= 0) {
        return;
    }
    if (clock_time_hm(&h, &m)) {
        snprintf(out, out_len, "%02d:%02d", h, m);
    } else {
        strlcpy(out, "--:--", out_len);
    }
}
