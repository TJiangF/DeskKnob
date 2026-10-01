idf_component_register(
    SRCS "input.c"
    INCLUDE_DIRS "include"
    REQUIRES
        deskknob_board
        esp_driver_gpio
        esp_timer
)
