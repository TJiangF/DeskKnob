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
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "nvs_flash.h"

#include "settings.h"
#include "display.h"
#include "input.h"
#include "motor.h"
#include "led_ring.h"
#include "wifi_mgr.h"
#include "balance.h"
#include "media.h"
#include "clock_time.h"
#include "ui.h"

static const char *TAG = "deskknob";

extern "C" void wifi_cli_start(void);

extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "DeskKnob booting (ESP-IDF %s)", esp_get_idf_version());

    /* ---- module capability report (flash / PSRAM) ---- */
    {
        esp_chip_info_t chip;
        esp_chip_info(&chip);
        ESP_LOGI(TAG, "chip: %s rev v%d.%d, %d core(s), wifi=%d ble=%d",
                 CONFIG_IDF_TARGET, chip.revision / 100, chip.revision % 100,
                 chip.cores, chip.features & CHIP_FEATURE_WIFI_BGN,
                 chip.features & CHIP_FEATURE_BLE);

        uint32_t flash_size = 0;
        esp_flash_get_size(NULL, &flash_size);
        ESP_LOGI(TAG, "flash: %u MB (%u bytes)", (unsigned)(flash_size / (1024 * 1024)), (unsigned)flash_size);

        size_t psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
        if (psram > 0) {
            ESP_LOGI(TAG, "psram: %u MB (%u bytes), free=%u",
                     (unsigned)(psram / (1024 * 1024)), (unsigned)psram,
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        } else {
            ESP_LOGW(TAG, "psram: not detected / not enabled");
        }
    }

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
    ESP_ERROR_CHECK(clock_time_init());

    ESP_ERROR_CHECK(ui_start());

    wifi_cli_start();

    ESP_LOGI(TAG, "boot complete, free heap=%u (type 'help' for WiFi CLI)",
             (unsigned)esp_get_free_heap_size());
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
