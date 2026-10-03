idf_component_register(
    SRCS "main.cpp" "wifi_cli.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        deskknob_board
        deskknob_display
        deskknob_input
        deskknob_motor
        deskknob_led
        deskknob_wifi
        deskknob_services
        deskknob_ui
        nvs_flash
)
