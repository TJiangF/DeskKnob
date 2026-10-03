#include "display.h"
#include "board_pins.h"
#include "settings.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_gc9a01.h"
#include "esp_lvgl_port.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "display";

#define LCD_PIXEL_CLOCK_HZ   (30 * 1000 * 1000)
#define LCD_BL_LEDC_TIMER     LEDC_TIMER_0
#define LCD_BL_LEDC_CHANNEL   LEDC_CHANNEL_0
#define LCD_BL_LEDC_MODE      LEDC_LOW_SPEED_MODE
#define LCD_BL_LEDC_RES       LEDC_TIMER_10_BIT

static lv_display_t *s_disp;
static esp_lcd_panel_handle_t s_panel;
static int s_brightness = 80;

static void backlight_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode = LCD_BL_LEDC_MODE,
        .duty_resolution = LCD_BL_LEDC_RES,
        .timer_num = LCD_BL_LEDC_TIMER,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));

    ledc_channel_config_t c = {
        .gpio_num = BOARD_LCD_BL,
        .speed_mode = LCD_BL_LEDC_MODE,
        .channel = LCD_BL_LEDC_CHANNEL,
        .timer_sel = LCD_BL_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&c));
}

void display_set_brightness(int percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    s_brightness = percent;
    uint32_t max_duty = (1u << 10) - 1u;
    ledc_set_duty(LCD_BL_LEDC_MODE, LCD_BL_LEDC_CHANNEL, (max_duty * percent) / 100);
    ledc_update_duty(LCD_BL_LEDC_MODE, LCD_BL_LEDC_CHANNEL);
    settings_set_int(SET_LCD_BRIGHTNESS, percent);
}

int display_get_brightness(void)
{
    return s_brightness;
}

esp_err_t display_init(void)
{
    backlight_init();

    /* ---- SPI bus ---- */
    const spi_bus_config_t buscfg = GC9A01_PANEL_BUS_SPI_CONFIG(
        BOARD_LCD_SCLK, BOARD_LCD_MOSI,
        BOARD_LCD_H_RES * 40 * (int)sizeof(uint16_t));
    ESP_RETURN_ON_ERROR(spi_bus_initialize(BOARD_LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "spi bus");

    /* Explicit hardware reset before anything else (long pulses are safer for
     * clone GC9A01 modules whose reset is marginal). */
    gpio_config_t rst = {
        .pin_bit_mask = (1ULL << BOARD_LCD_RST),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&rst));
    gpio_set_level(BOARD_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(BOARD_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(40));
    gpio_set_level(BOARD_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(150));

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_config = GC9A01_PANEL_IO_SPI_CONFIG(
        BOARD_LCD_CS, BOARD_LCD_DC, NULL, NULL);
    io_config.pclk_hz = LCD_PIXEL_CLOCK_HZ;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BOARD_LCD_SPI_HOST, &io_config, &io),
                        TAG, "panel io");

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,   /* already reset above */
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_gc9a01(io, &panel_config, &s_panel), TAG, "new panel");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "init");
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(s_panel, true), TAG, "invert");
    /* Mirroring is applied by esp_lvgl_port based on disp_cfg.rotation below. */
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(s_panel, false, false), TAG, "mirror");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "disp on");

    /* ---- LVGL port ---- */
    const lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_RETURN_ON_ERROR(lvgl_port_init(&port_cfg), TAG, "lvgl port");

    bool have_psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0;
    /* With PSRAM we can afford a bigger, double-buffered draw area. The buffer
     * is not DMA-capable from PSRAM, so use it only when PSRAM is present. */
    uint32_t buf_px = have_psram ? (BOARD_LCD_H_RES * 100) : (BOARD_LCD_H_RES * 40);

    lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io,
        .panel_handle = s_panel,
        .buffer_size = buf_px,
        .double_buffer = true,
        .hres = BOARD_LCD_H_RES,
        .vres = BOARD_LCD_V_RES,
        .monochrome = false,
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = false,
        },
        .color_format = LV_COLOR_FORMAT_RGB565,
        .flags = {
            .buff_dma = !have_psram,
            .buff_spiram = have_psram,
            .swap_bytes = true,
        },
    };
    s_disp = lvgl_port_add_disp(&disp_cfg);
    if (s_disp == NULL) {
        ESP_LOGE(TAG, "lvgl_port_add_disp failed");
        return ESP_FAIL;
    }

    s_brightness = settings_get_int(SET_LCD_BRIGHTNESS, 80);
    display_set_brightness(s_brightness);

    ESP_LOGI(TAG, "display ready %dx%d", BOARD_LCD_H_RES, BOARD_LCD_V_RES);
    return ESP_OK;
}

lv_display_t *display_get_lv(void) { return s_disp; }

esp_lcd_panel_handle_t display_get_panel(void) { return s_panel; }

void display_pause_lvgl(void)
{
    lvgl_port_stop();
}

bool display_lock(uint32_t timeout_ms)
{
    return lvgl_port_lock(timeout_ms);
}

void display_unlock(void)
{
    lvgl_port_unlock();
}
