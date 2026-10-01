idf_component_register(
    SRCS "led_ring.c"
    INCLUDE_DIRS "include"
    REQUIRES
        deskknob_board
        deskknob_settings
        espressif__led_strip
        esp_driver_rmt
        esp_driver_gpio
)
