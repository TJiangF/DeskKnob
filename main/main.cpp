/*
 * DeskKnob - application entry point.
 *
 * Boot order:
 *   1. NVS + persistent settings
 *   2. display (GC9A01 + LVGL), LED ring
 *   3. input (HX710 pressure film + back button)
 *   4. motor (SimpleFOC 6PWM + AS5600)
 *   5. background services (WiFi, balance)
 *   6. UI task (owns navigation & rendering)
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "nvs_flash.h"

#include "settings.h"
#include "display.h"
#include "input.h"
#include "motor.h"
#include "led_ring.h"
#include "wifi_mgr.h"
#include "balance.h"
#include "media.h"
#include "ui.h"

static const char *TAG = "deskknob";

extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "DeskKnob booting (ESP-IDF %s)", esp_get_idf_version());
    ESP_ERROR_CHECK(settings_init());

    ESP_ERROR_CHECK(display_init());
    ESP_ERROR_CHECK(led_ring_init());
    ESP_ERROR_CHECK(led_ring_start_task());

    ESP_ERROR_CHECK(input_init());

    if (motor_init() == ESP_OK) {
        motor_start_task();
        motor_set_stiffness((float)settings_get_int(SET_RATCHET_STIFFNESS, 50) / 10.0f);
        motor_set_detents(settings_get_int(SET_RATCHET_DETENTS, 24));
        motor_set_mode(MOTOR_MODE_TORQUE_BOUNDED, 4);
    } else {
        ESP_LOGE(TAG, "motor init failed, continuing without haptics");
    }

    ESP_ERROR_CHECK(wifi_mgr_init());
    ESP_ERROR_CHECK(balance_service_init());
    ESP_ERROR_CHECK(media_init());

    ESP_ERROR_CHECK(ui_start());

    ESP_LOGI(TAG, "boot complete, free heap=%u", (unsigned)esp_get_free_heap_size());
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
