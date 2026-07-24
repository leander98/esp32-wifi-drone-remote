idf_component_register(
    SRCS "esp32-wifi-drone-remote.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_event esp_http_server esp_netif esp_wifi mdns nvs_flash
    EMBED_TXTFILES "controller.html" "wifi-setup.html"
)
