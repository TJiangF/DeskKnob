#include "wifi_mgr.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "wifi_mgr";
static const char *NVS_NS = "wifi";

#define MAX_STORED 8

typedef struct {
    char ssid[WIFI_SSID_MAX];
    char pass[64];
} cred_t;

static cred_t s_stored[MAX_STORED];
static int s_stored_count;
static char s_last_ssid[WIFI_SSID_MAX];

static volatile wifi_state_t s_state = WIFI_ST_DISCONNECTED;
static char s_current_ssid[WIFI_SSID_MAX];
static char s_ip_str[16];

static volatile bool s_scan_done;
static int s_scan_count;
static struct {
    char ssid[WIFI_SSID_MAX];
    int rssi;
} s_scan[WIFI_SCAN_MAX];

static esp_netif_t *s_netif;
static bool s_inited;

static void load_creds(void)
{
    s_stored_count = 0;
    s_last_ssid[0] = '\0';
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    int32_t count = 0;
    if (nvs_get_i32(h, "count", &count) == ESP_OK && count > 0) {
        if (count > MAX_STORED) count = MAX_STORED;
        for (int i = 0; i < count; i++) {
            char key[16];
            size_t len = sizeof(s_stored[i].ssid);
            snprintf(key, sizeof(key), "ssid_%d", i);
            if (nvs_get_str(h, key, s_stored[i].ssid, &len) != ESP_OK) continue;
            len = sizeof(s_stored[i].pass);
            snprintf(key, sizeof(key), "pass_%d", i);
            if (nvs_get_str(h, key, s_stored[i].pass, &len) != ESP_OK) {
                s_stored[i].pass[0] = '\0';
            }
            if (s_stored[i].ssid[0]) s_stored_count++;
        }
    }
    size_t len = sizeof(s_last_ssid);
    nvs_get_str(h, "last", s_last_ssid, &len);
    nvs_close(h);
}

static void save_creds(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_i32(h, "count", s_stored_count);
    for (int i = 0; i < s_stored_count; i++) {
        char key[16];
        snprintf(key, sizeof(key), "ssid_%d", i);
        nvs_set_str(h, key, s_stored[i].ssid);
        snprintf(key, sizeof(key), "pass_%d", i);
        nvs_set_str(h, key, s_stored[i].pass);
    }
    if (s_last_ssid[0]) {
        nvs_set_str(h, "last", s_last_ssid);
    }
    nvs_commit(h);
    nvs_close(h);
}

int wifi_mgr_stored_count(void) { return s_stored_count; }
const char *wifi_mgr_stored_ssid(int index)
{
    return (index >= 0 && index < s_stored_count) ? s_stored[index].ssid : "";
}
const char *wifi_mgr_stored_pass(int index)
{
    return (index >= 0 && index < s_stored_count) ? s_stored[index].pass : "";
}
const char *wifi_mgr_last_ssid(void) { return s_last_ssid; }

bool wifi_mgr_has_credentials(const char *ssid)
{
    if (!ssid) return false;
    for (int i = 0; i < s_stored_count; i++) {
        if (strcmp(s_stored[i].ssid, ssid) == 0) return true;
    }
    return false;
}

esp_err_t wifi_mgr_forget(const char *ssid)
{
    if (!ssid) return ESP_ERR_INVALID_ARG;
    int w = 0;
    for (int i = 0; i < s_stored_count; i++) {
        if (strcmp(s_stored[i].ssid, ssid) != 0) {
            if (w != i) s_stored[w] = s_stored[i];
            w++;
        }
    }
    s_stored_count = w;
    if (strcmp(s_last_ssid, ssid) == 0) s_last_ssid[0] = '\0';
    save_creds();
    ESP_LOGI(TAG, "forgot '%s', %d remain", ssid, s_stored_count);
    return ESP_OK;
}

static void store_credential(const char *ssid, const char *pass)
{
    for (int i = 0; i < s_stored_count; i++) {
        if (strcmp(s_stored[i].ssid, ssid) == 0) {
            strlcpy(s_stored[i].pass, pass ? pass : "", sizeof(s_stored[i].pass));
            strlcpy(s_last_ssid, ssid, sizeof(s_last_ssid));
            save_creds();
            return;
        }
    }
    if (s_stored_count < MAX_STORED) {
        strlcpy(s_stored[s_stored_count].ssid, ssid, sizeof(s_stored[0].ssid));
        strlcpy(s_stored[s_stored_count].pass, pass ? pass : "", sizeof(s_stored[0].pass));
        s_stored_count++;
    }
    strlcpy(s_last_ssid, ssid, sizeof(s_last_ssid));
    save_creds();
}

/* ---- event handlers ---- */
static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_STA_START:
            break;
        case WIFI_EVENT_STA_CONNECTED: {
            wifi_event_sta_connected_t *e = (wifi_event_sta_connected_t *)data;
            memcpy(s_current_ssid, e->ssid, e->ssid_len);
            s_current_ssid[e->ssid_len] = '\0';
            break;
        }
        case WIFI_EVENT_STA_DISCONNECTED: {
            s_state = WIFI_ST_FAILED;
            s_ip_str[0] = '\0';
            ESP_LOGW(TAG, "disconnected");
            break;
        }
        case WIFI_EVENT_SCAN_DONE: {
            uint16_t n = 0;
            esp_err_t num_err = esp_wifi_scan_get_ap_num(&n);
            ESP_LOGI(TAG, "SCAN_DONE event, ap_num err=%s n=%u",
                     esp_err_to_name(num_err), (unsigned)n);
            wifi_ap_record_t *recs = calloc(n ? n : 1, sizeof(wifi_ap_record_t));
            if (recs) {
                esp_err_t rec_err = esp_wifi_scan_get_ap_records(&n, recs);
                ESP_LOGI(TAG, "get_ap_records err=%s n=%u", esp_err_to_name(rec_err), (unsigned)n);
                s_scan_count = 0;
                for (int i = 0; i < n && s_scan_count < WIFI_SCAN_MAX; i++) {
                    if (recs[i].ssid[0] == '\0') continue;
                    bool dup = false;
                    for (int k = 0; k < s_scan_count; k++) {
                        if (strcmp(s_scan[k].ssid, (char *)recs[i].ssid) == 0) { dup = true; break; }
                    }
                    if (dup) continue;
                    strlcpy(s_scan[s_scan_count].ssid, (char *)recs[i].ssid, WIFI_SSID_MAX);
                    s_scan[s_scan_count].rssi = recs[i].rssi;
                    s_scan_count++;
                }
                free(recs);
            }
            s_scan_done = true;
            ESP_LOGI(TAG, "scan done: %d APs", s_scan_count);
            break;
        }
        default:
            break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        snprintf(s_ip_str, sizeof(s_ip_str), IPSTR, IP2STR(&e->ip_info.ip));
        s_state = WIFI_ST_CONNECTED;
        ESP_LOGI(TAG, "connected to '%s', ip=%s", s_current_ssid, s_ip_str);
    }
}

esp_err_t wifi_mgr_init(void)
{
    if (s_inited) return ESP_OK;

    load_creds();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    /* A valid country code is required for scanning to enumerate channels. */
    wifi_country_t country = {
        .cc = "CN",
        .schan = 1,
        .nchan = 13,
        .policy = WIFI_COUNTRY_POLICY_AUTO,
    };
    esp_err_t cerr = esp_wifi_set_country(&country);
    ESP_LOGI(TAG, "set_country -> %s", esp_err_to_name(cerr));

    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_country_t got = {0};
    if (esp_wifi_get_country(&got) == ESP_OK) {
        ESP_LOGI(TAG, "country cc=%c%c%c ch=%d-%d", got.cc[0], got.cc[1], got.cc[2], got.schan, got.schan + got.nchan - 1);
    }
    uint8_t mac[6];
    if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) {
        ESP_LOGI(TAG, "sta mac %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    s_inited = true;
    ESP_LOGI(TAG, "wifi ready, %d stored network(s)", s_stored_count);

    if (s_last_ssid[0]) {
        for (int i = 0; i < s_stored_count; i++) {
            if (strcmp(s_stored[i].ssid, s_last_ssid) == 0) {
                wifi_mgr_connect(s_stored[i].ssid, s_stored[i].pass, false);
                break;
            }
        }
    }
    return ESP_OK;
}

esp_err_t wifi_mgr_connect(const char *ssid, const char *pass, bool persist)
{
    if (!ssid) return ESP_ERR_INVALID_ARG;
    wifi_config_t wc = {0};
    strlcpy((char *)wc.sta.ssid, ssid, sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, pass ? pass : "", sizeof(wc.sta.password));
    wc.sta.threshold.authmode = WIFI_AUTH_OPEN;

    s_state = WIFI_ST_CONNECTING;
    esp_wifi_disconnect();
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    esp_err_t err = esp_wifi_connect();
    if (err == ESP_OK && persist) {
        store_credential(ssid, pass);
    }
    ESP_LOGI(TAG, "connecting to '%s'%s", ssid, persist ? " (saved)" : "");
    return err;
}

esp_err_t wifi_mgr_reconnect(void)
{
    esp_wifi_disconnect();
    return esp_wifi_connect();
}

esp_err_t wifi_mgr_disconnect(void)
{
    s_state = WIFI_ST_DISCONNECTED;
    return esp_wifi_disconnect();
}

wifi_state_t wifi_mgr_state(void) { return s_state; }
bool wifi_mgr_is_connected(void) { return s_state == WIFI_ST_CONNECTED; }
const char *wifi_mgr_current_ssid(void) { return s_current_ssid; }
const char *wifi_mgr_ip_str(void) { return s_ip_str; }

esp_err_t wifi_mgr_scan_start(void)
{
    s_scan_done = false;
    s_scan_count = 0;
    wifi_scan_config_t sc = {0};
    sc.show_hidden = true;
    /* Scanning is allowed while connected; ESP-IDF may briefly hop channels. */
    esp_err_t err = esp_wifi_scan_start(&sc, false);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "scan_start failed: %s", esp_err_to_name(err));
        /* One retry after a short settle (e.g. right after esp_wifi_start). */
        vTaskDelay(pdMS_TO_TICKS(300));
        err = esp_wifi_scan_start(&sc, false);
    }
    ESP_LOGI(TAG, "scan_start -> %s", esp_err_to_name(err));
    return err;
}

bool wifi_mgr_scan_done(void) { return s_scan_done; }
int wifi_mgr_scan_count(void) { return s_scan_count; }
const char *wifi_mgr_scan_ssid(int index)
{
    return (index >= 0 && index < s_scan_count) ? s_scan[index].ssid : "";
}
int wifi_mgr_scan_rssi(int index)
{
    return (index >= 0 && index < s_scan_count) ? s_scan[index].rssi : 0;
}
