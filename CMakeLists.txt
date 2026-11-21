idf_component_register(SRCS "main.c" "esp_now.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp32-camera nvs_flash bt esp_wifi esp_netif esp_event)