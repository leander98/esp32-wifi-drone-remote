idf_component_register(
    SRCS "esp32-wifi-drone-remote.c"
         "esp32-wifi-drone-remote-pages.c"
         "esp32-wifi-drone-remote-storage.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_event esp_http_server esp_netif esp_wifi mdns nvs_flash
    EMBED_TXTFILES "controller.html" "wifi-setup.html"
                   "settings.html" "wifi-ap-settings.html"
                   "imu-settings.html" "pid-settings.html" "esc-settings.html"
                   "esc-pwm-channel-settings.html"
                   "esc-manual.html" "esc-programming.html"
                   "esc-throttle-range.html"
)
