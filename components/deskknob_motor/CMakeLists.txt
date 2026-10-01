idf_component_register(
    SRCS "motor.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        deskknob_board
        espressif__esp_simplefoc
        esp_driver_mcpwm
        esp_driver_ledc
        esp_timer
)
