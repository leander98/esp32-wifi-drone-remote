# ESP32 Wi-Fi drone remote component

ESP-IDF component for a Wi-Fi server that provides a browser-based drone
controller.

The component currently contains the public API and configuration scaffold.
Wi-Fi access-point setup, the HTTP server, the controller page, and control
message handling still need to be implemented.

## Adding the component

Place `esp32-wifi-drone-remote` in the project's `components` directory and
declare it from the consuming component:

```cmake
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp32-wifi-drone-remote
)
```

## Configuration

The access-point SSID, password, and connection limit can be configured under
`Component config > Wi-Fi drone remote configuration`.

```c
#include "esp32-wifi-drone-remote.h"

void app_main(void)
{
    esp32_wifi_drone_remote_config_t config =
        ESP32_WIFI_DRONE_REMOTE_DEFAULT_CONFIG();

    ESP_ERROR_CHECK(esp32_wifi_drone_remote_start(&config));
}
```

The start and stop functions currently return `ESP_ERR_NOT_SUPPORTED` until
the component implementation is added.
