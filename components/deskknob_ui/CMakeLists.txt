idf_component_register(
    SRCS "ui.c"
    INCLUDE_DIRS "include"
    REQUIRES
        deskknob_board
        deskknob_display
        deskknob_input
        deskknob_motor
        deskknob_led
        deskknob_wifi
        deskknob_settings
        deskknob_services
        lvgl__lvgl
)
