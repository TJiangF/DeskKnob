idf_component_register(
    SRCS "display.c"
    INCLUDE_DIRS "include"
    REQUIRES
        deskknob_board
        deskknob_settings
        espressif__esp_lvgl_port
        espressif__esp_lcd_gc9a01
        lvgl__lvgl
        esp_driver_spi
        esp_driver_ledc
        esp_driver_gpio
)
