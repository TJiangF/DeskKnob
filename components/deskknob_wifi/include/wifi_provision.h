/*
 * Phone-based Wi-Fi provisioning.
 *
 * Starts a SoftAP ("DeskKnob-Setup") with a tiny HTTP page so a phone can
 * pick a network, type the password, and save it. The result is stored via
 * wifi_mgr (NVS) and the device then connects in STA mode.
 *
 * AP mode and STA mode are exclusive on a single radio, so while provisioning
 * the STA interface is stopped; on success we return to STA.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WIFI_PROV_AP_SSID     "DeskKnob-Setup"
#define WIFI_PROV_AP_PASSWORD ""          /* open; set non-empty to require a pass */

esp_err_t wifi_provision_start(void);
void      wifi_provision_stop(void);
bool      wifi_provision_active(void);

#ifdef __cplusplus
}
#endif
