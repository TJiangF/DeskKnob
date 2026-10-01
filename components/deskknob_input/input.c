#include "input.h"
#include "board_pins.h"

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "input";

/* -------- HX710 bit-banged 24-bit ADC -------- */

#define HX710_READY_TIMEOUT_MS 120

static int32_t hx710_read(int32_t *out)
{
    /* Wait until DOUT drops low (conversion ready). */
    int64_t deadline = esp_timer_get_time() + HX710_READY_TIMEOUT_MS * 1000;
    while (gpio_get_level(BOARD_PIN_HX710_DOUT) != 0) {
        if (esp_timer_get_time() > deadline) {
            return ESP_ERR_TIMEOUT;
        }
        esp_rom_delay_us(5);
    }

    uint32_t value = 0;
    for (int i = 0; i < 24; i++) {
        gpio_set_level(BOARD_PIN_HX710_SCK, 1);
        esp_rom_delay_us(1);
        value = (value << 1) | (uint32_t)gpio_get_level(BOARD_PIN_HX710_DOUT);
        gpio_set_level(BOARD_PIN_HX710_SCK, 0);
        esp_rom_delay_us(1);
    }
    /* 25th pulse selects channel A / gain 128 for the next conversion. */
    gpio_set_level(BOARD_PIN_HX710_SCK, 1);
    esp_rom_delay_us(1);
    gpio_set_level(BOARD_PIN_HX710_SCK, 0);
    esp_rom_delay_us(1);

    if (value & 0x00800000u) {
        value |= 0xFF000000u;   /* sign extend */
    }
    *out = (int32_t)value;
    return ESP_OK;
}

/* -------- shared state -------- */

static QueueHandle_t s_queue;
static volatile int32_t s_raw;
static volatile int32_t s_baseline;
static volatile uint32_t s_last_ms;
static volatile uint32_t s_last_press_ms;

static int32_t s_noise_threshold = 500000;
static int32_t s_trigger_threshold = 1000000;
static bool s_pressed;

#define DEBOUNCE_MS 500
#define REQUIRED_COUNT 3
#define SAMPLE_PERIOD_MS 50

static void calculate_baseline(void)
{
    int64_t sum = 0;
    int count = 0;
    int64_t start = esp_timer_get_time();

    ESP_LOGI(TAG, "calibrating pressure baseline...");
    while ((esp_timer_get_time() - start) < 1000000LL) {
        int32_t v = 0;
        if (hx710_read(&v) == ESP_OK && v > s_noise_threshold) {
            sum += v;
            count++;
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
    if (count > 0) {
        s_baseline = (int32_t)(sum / count);
        s_trigger_threshold = (int32_t)(s_baseline * 1.022f);
        s_noise_threshold = (int32_t)(s_baseline * 0.5f);
        ESP_LOGI(TAG, "baseline=%ld trigger=%ld noise=%ld",
                 (long)s_baseline, (long)s_trigger_threshold, (long)s_noise_threshold);
    } else {
        ESP_LOGW(TAG, "baseline calibration failed, keeping defaults");
    }
}

static void emit(input_event_t type, int32_t value)
{
    input_msg_t msg = {.type = type, .value = value};
    if (s_queue) {
        xQueueSend(s_queue, &msg, 0);
    }
}

static void pressure_task(void *arg)
{
    (void)arg;
    int press_count = 0;
    for (;;) {
        int32_t v = 0;
        if (hx710_read(&v) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
            continue;
        }
        s_raw = v;
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        s_last_ms = now;

        if (v < s_noise_threshold) {
            press_count = 0;
            s_pressed = false;
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
            continue;
        }

        if (v > s_trigger_threshold) {
            press_count++;
            if (press_count >= REQUIRED_COUNT && (now - s_last_press_ms) > DEBOUNCE_MS) {
                s_last_press_ms = now;
                s_pressed = true;
                emit(INPUT_EVT_PRESS, v);
            }
        } else {
            press_count = 0;
        }

        if (s_pressed && (now - s_last_press_ms) > DEBOUNCE_MS) {
            s_pressed = false;
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

/* The button ISR fires on the falling edge only; a small helper task watches
 * for a long hold and emits BACK_HOLD after ~1s while the line stays low. */
static void button_task(void *arg)
{
    (void)arg;
    bool sent_hold = false;
    for (;;) {
        bool low = (gpio_get_level(BOARD_PIN_BUTTON) == 0);
        if (low && !sent_hold) {
            int held = 0;
            while (gpio_get_level(BOARD_PIN_BUTTON) == 0 && held < 1200) {
                vTaskDelay(pdMS_TO_TICKS(20));
                held += 20;
            }
            if (held >= 1000) {
                input_msg_t m = {.type = INPUT_EVT_BACK_HOLD, .value = held};
                xQueueSend(s_queue, &m, 0);
                sent_hold = true;
            }
            /* wait for release to avoid repeats */
            while (gpio_get_level(BOARD_PIN_BUTTON) == 0) {
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
        if (!low) {
            sent_hold = false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void IRAM_ATTR button_isr(void *arg)
{
    static volatile int64_t last_isr_us;
    (void)arg;
    int64_t now = esp_timer_get_time();
    if (now - last_isr_us < (DEBOUNCE_MS * 1000)) {
        return;
    }
    last_isr_us = now;
    BaseType_t hp = pdFALSE;
    input_msg_t msg = {.type = INPUT_EVT_BACK, .value = 0};
    if (s_queue) {
        xQueueSendFromISR(s_queue, &msg, &hp);
    }
    if (hp) {
        portYIELD_FROM_ISR();
    }
}

esp_err_t input_init(void)
{
    s_queue = xQueueCreate(8, sizeof(input_msg_t));
    if (!s_queue) {
        return ESP_ERR_NO_MEM;
    }

    /* HX710 */
    gpio_config_t hx = {
        .pin_bit_mask = (1ULL << BOARD_PIN_HX710_SCK),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&hx));
    gpio_set_level(BOARD_PIN_HX710_SCK, 0);

    gpio_config_t hx_in = {
        .pin_bit_mask = (1ULL << BOARD_PIN_HX710_DOUT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&hx_in));

    /* Button */
    gpio_config_t btn = {
        .pin_bit_mask = (1ULL << BOARD_PIN_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&btn));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOARD_PIN_BUTTON, button_isr, NULL);

    calculate_baseline();

    xTaskCreatePinnedToCore(pressure_task, "hx710", 3072, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(button_task, "button", 2560, NULL, 3, NULL, 0);
    ESP_LOGI(TAG, "input ready (dout=%d sck=%d btn=%d)",
             BOARD_PIN_HX710_DOUT, BOARD_PIN_HX710_SCK, BOARD_PIN_BUTTON);
    return ESP_OK;
}

bool input_wait(input_msg_t *out, uint32_t timeout_ms)
{
    if (!s_queue || !out) {
        return false;
    }
    return xQueueReceive(s_queue, out, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

int32_t input_pressure_raw(void) { return s_raw; }
int32_t input_pressure_baseline(void) { return s_baseline; }
uint32_t input_pressure_last_ms(void) { return s_last_ms; }
bool input_is_pressed(void) { return s_pressed; }
