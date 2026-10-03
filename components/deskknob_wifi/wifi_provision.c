#include "wifi_provision.h"
#include "wifi_mgr.h"

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_log.h"

static const char *TAG = "wifi_prov";

static esp_netif_t *s_ap_netif;
static httpd_handle_t s_httpd;
static volatile bool s_active;

static const char *PAGE_HEAD =
    "<!doctype html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>DeskKnob Setup</title><style>"
    "body{font-family:-apple-system,system-ui,sans-serif;background:#111;color:#eee;"
    "margin:0;padding:24px;}h1{font-size:20px}label{display:block;margin:14px 0 6px}"
    "select,input{width:100%;padding:10px;font-size:16px;border-radius:8px;border:1px solid #444;"
    "background:#1c1c1e;color:#eee}button{margin-top:18px;width:100%;padding:12px;font-size:16px;"
    "border-radius:8px;border:0;background:#2e9bff;color:#fff}</style></head><body>"
    "<h1>DeskKnob Wi-Fi Setup</h1>";

static esp_err_t root_get(httpd_req_t *req)
{
    httpd_resp_send_chunk(req, PAGE_HEAD, HTTPD_RESP_USE_STRLEN);

    /* Build the <select> from the last scan. */
    httpd_resp_send_chunk(req, "<form method='POST' action='/save'>", HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, "<label>Network</label><select name='ssid'>", HTTPD_RESP_USE_STRLEN);
    int n = wifi_mgr_scan_count();
    for (int i = 0; i < n; i++) {
        char opt[96];
        snprintf(opt, sizeof(opt), "<option value=\"%s\">%s (%d dBm)</option>",
                 wifi_mgr_scan_ssid(i), wifi_mgr_scan_ssid(i), wifi_mgr_scan_rssi(i));
        httpd_resp_send_chunk(req, opt, HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_send_chunk(req, "</select>", HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req,
        "<label>Password</label><input name='pass' type='text' autocapitalize='off' "
        "autocorrect='off' spellcheck='false' placeholder='Wi-Fi password'>"
        "<button type='submit'>Save &amp; Connect</button></form>",
        HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, "</body></html>", HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static void urldecode(char *s)
{
    char *o = s;
    while (*s) {
        if (*s == '%' && s[1] && s[2]) {
            char h[3] = {s[1], s[2], 0};
            *o++ = (char)strtol(h, NULL, 16);
            s += 3;
        } else if (*s == '+') {
            *o++ = ' ';
            s++;
        } else {
            *o++ = *s++;
        }
    }
    *o = '\0';
}

static void form_get(const char *body, const char *key, char *out, size_t out_len)
{
    out[0] = '\0';
    char pat[24];
    snprintf(pat, sizeof(pat), "%s=", key);
    const char *p = strstr(body, pat);
    if (!p) {
        return;
    }
    p += strlen(pat);
    const char *end = strchr(p, '&');
    size_t len = end ? (size_t)(end - p) : strlen(p);
    if (len >= out_len) {
        len = out_len - 1;
    }
    memcpy(out, p, len);
    out[len] = '\0';
    urldecode(out);
}

static esp_err_t save_post(httpd_req_t *req)
{
    char body[256];
    int len = req->content_len;
    if (len >= (int)sizeof(body)) {
        len = sizeof(body) - 1;
    }
    int received = httpd_req_recv(req, body, len);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no body");
        return ESP_FAIL;
    }
    body[received] = '\0';

    char ssid[64] = {0};
    char pass[80] = {0};
    form_get(body, "ssid", ssid, sizeof(ssid));
    form_get(body, "pass", pass, sizeof(pass));
    ESP_LOGI(TAG, "provision: ssid='%s'", ssid);

    const char *ok =
        "<!doctype html><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<body style='font-family:sans-serif;background:#111;color:#eee;padding:24px'>"
        "<h2>Saved!</h2><p>DeskKnob is connecting to your Wi-Fi. "
        "You can disconnect this setup network.</p></body>";

    if (ssid[0]) {
        /* Persist and hand back to the WiFi manager for STA connection. */
        wifi_mgr_connect(ssid, pass, true);
    }
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, ok, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t wifi_provision_start(void)
{
    if (s_active) {
        return ESP_OK;
    }

    /* Trigger a scan so the form has a fresh list (STA must be started). */
    wifi_mgr_scan_start();
    vTaskDelay(pdMS_TO_TICKS(2500));

    if (!s_ap_netif) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
    }

    wifi_config_t ap = {0};
    strlcpy((char *)ap.ap.ssid, WIFI_PROV_AP_SSID, sizeof(ap.ap.ssid));
    ap.ap.ssid_len = strlen(WIFI_PROV_AP_SSID);
    if (WIFI_PROV_AP_PASSWORD[0]) {
        strlcpy((char *)ap.ap.password, WIFI_PROV_AP_PASSWORD, sizeof(ap.ap.password));
        ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        ap.ap.authmode = WIFI_AUTH_OPEN;
    }
    ap.ap.max_connection = 2;
    ap.ap.channel = 1;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());

    if (!s_httpd) {
        httpd_config_t hc = HTTPD_DEFAULT_CONFIG();
        hc.max_uri_handlers = 4;
        if (httpd_start(&s_httpd, &hc) != ESP_OK) {
            ESP_LOGE(TAG, "httpd_start failed");
            return ESP_FAIL;
        }
        httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = root_get};
        httpd_uri_t save = {.uri = "/save", .method = HTTP_POST, .handler = save_post};
        httpd_register_uri_handler(s_httpd, &root);
        httpd_register_uri_handler(s_httpd, &save);
    }

    s_active = true;
    ESP_LOGI(TAG, "provisioning AP '%s' up, open http://192.168.4.1/", WIFI_PROV_AP_SSID);
    return ESP_OK;
}

void wifi_provision_stop(void)
{
    if (!s_active) {
        return;
    }
    if (s_httpd) {
        httpd_stop(s_httpd);
        s_httpd = NULL;
    }
    esp_wifi_set_mode(WIFI_MODE_STA);
    s_active = false;
    ESP_LOGI(TAG, "provisioning stopped");
}

bool wifi_provision_active(void) { return s_active; }
