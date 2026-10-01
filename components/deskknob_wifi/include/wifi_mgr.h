/*
 * Wi-Fi station manager.
 *
 * Event-driven (no blocking connect loops). Supports:
 *   - async scan with cached results
 *   - connect with optional credential persistence
 *   - disconnect (keep creds) / forget (erase creds)
 *   - automatic reconnect to the last successfully used network
 *
 * Credentials are stored in NVS namespace "wifi" as ssid_i / pass_i, plus a
 * "count" and "last" index.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WIFI_ST_DISCONNECTED = 0,
    WIFI_ST_CONNECTING,
    WIFI_ST_CONNECTED,
    WIFI_ST_FAILED,
} wifi_state_t;

#define WIFI_SCAN_MAX 24
#define WIFI_SSID_MAX 33

esp_err_t wifi_mgr_init(void);

/* ---- connection ---- */
esp_err_t wifi_mgr_connect(const char *ssid, const char *pass, bool persist);
esp_err_t wifi_mgr_disconnect(void);      /* keeps stored credentials */
esp_err_t wifi_mgr_reconnect(void);

wifi_state_t wifi_mgr_state(void);
bool wifi_mgr_is_connected(void);
const char *wifi_mgr_current_ssid(void);
const char *wifi_mgr_ip_str(void);

/* ---- stored credentials ---- */
int wifi_mgr_stored_count(void);
const char *wifi_mgr_stored_ssid(int index);
const char *wifi_mgr_stored_pass(int index);
bool wifi_mgr_has_credentials(const char *ssid);
esp_err_t wifi_mgr_forget(const char *ssid);
const char *wifi_mgr_last_ssid(void);

/* ---- scan ---- */
esp_err_t wifi_mgr_scan_start(void);
bool wifi_mgr_scan_done(void);
int wifi_mgr_scan_count(void);
const char *wifi_mgr_scan_ssid(int index);
int wifi_mgr_scan_rssi(int index);

#ifdef __cplusplus
}
#endif
