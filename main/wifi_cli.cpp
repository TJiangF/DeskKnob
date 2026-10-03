/*
 * Serial debug CLI for the Wi-Fi / balance flow.
 *
 * Commands (type in idf.py monitor / any serial terminal):
 *   scan                 start an async scan
 *   list                 print cached scan results
 *   connect <ssid> <pwd> connect and persist credentials on success
 *   reconnect            reconnect to last used network
 *   disconnect           disconnect, keep credentials
 *   forget <ssid>        erase stored credentials
 *   stored               list stored networks
 *   apikey <key>         set the DeepSeek API key ("apikey" alone clears it)
 *   balance              fetch the DeepSeek balance
 *   help
 */
#include "wifi_mgr.h"
#include "wifi_provision.h"
#include "balance.h"

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define WIFI_CLI_BUF 160

static const char *TAG = "wificli";

static void handle_line(char *line)
{
    char *cmd = strtok(line, " \t\r\n");
    if (!cmd) {
        return;
    }
    if (strcmp(cmd, "scan") == 0) {
        wifi_mgr_scan_start();
        ESP_LOGI(TAG, "scan requested");
    } else if (strcmp(cmd, "list") == 0) {
        int n = wifi_mgr_scan_count();
        ESP_LOGI(TAG, "scan done=%d count=%d", wifi_mgr_scan_done(), n);
        for (int i = 0; i < n; i++) {
            ESP_LOGI(TAG, "  [%d] %s (%d dBm) known=%d",
                     i, wifi_mgr_scan_ssid(i), wifi_mgr_scan_rssi(i),
                     wifi_mgr_has_credentials(wifi_mgr_scan_ssid(i)));
        }
    } else if (strcmp(cmd, "connect") == 0) {
        char *ssid = strtok(NULL, " \t\r\n");
        char *pass = strtok(NULL, " \t\r\n");
        if (ssid) {
            wifi_mgr_connect(ssid, pass ? pass : "", true);
        }
    } else if (strcmp(cmd, "reconnect") == 0) {
        wifi_mgr_reconnect();
    } else if (strcmp(cmd, "disconnect") == 0) {
        wifi_mgr_disconnect();
    } else if (strcmp(cmd, "forget") == 0) {
        char *ssid = strtok(NULL, " \t\r\n");
        if (ssid) {
            wifi_mgr_forget(ssid);
        }
    } else if (strcmp(cmd, "stored") == 0) {
        int n = wifi_mgr_stored_count();
        ESP_LOGI(TAG, "stored=%d last='%s'", n, wifi_mgr_last_ssid());
        for (int i = 0; i < n; i++) {
            ESP_LOGI(TAG, "  [%d] %s", i, wifi_mgr_stored_ssid(i));
        }
    } else if (strcmp(cmd, "apikey") == 0) {
        char *key = strtok(NULL, " \t\r\n");
        balance_set_api_key(key ? key : "");
        ESP_LOGI(TAG, "apikey set (%s)", key ? "provided" : "cleared");
    } else if (strcmp(cmd, "balance") == 0) {
        balance_request_refresh();
    } else if (strcmp(cmd, "ap") == 0) {
        wifi_provision_start();
    } else if (strcmp(cmd, "apstop") == 0) {
        wifi_provision_stop();
    } else if (strcmp(cmd, "help") == 0) {
        ESP_LOGI(TAG, "cmds: scan list connect <ssid> <pwd> reconnect disconnect forget <ssid> stored apikey <key> balance");
    } else {
        ESP_LOGW(TAG, "unknown: %s", cmd);
    }
}

extern "C" void wifi_cli_start(void)
{
    static char buf[WIFI_CLI_BUF];
    xTaskCreate([](void *) {
        size_t len = 0;
        for (;;) {
            int c = getchar();
            if (c == EOF) {
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }
            if (c == '\r' || c == '\n') {
                if (len > 0) {
                    buf[len] = '\0';
                    handle_line(buf);
                    len = 0;
                }
            } else if (len < sizeof(buf) - 1) {
                buf[len++] = (char)c;
            }
        }
    }, "wificli", 4096, NULL, 2, NULL);
}
