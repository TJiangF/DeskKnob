idf_component_register(
    SRCS
        "balance.c"
        "salary.c"
        "media.c"
    INCLUDE_DIRS "include"
    REQUIRES
        deskknob_settings
        deskknob_wifi
        esp_http_client
        esp-tls
        espressif__cjson
        esp_timer
        mbedtls
)
