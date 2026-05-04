idf_component_register(SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_event esp_wifi nvs_flash lwip esp_timer)
