# ESP32 Wi-Fi drone remote component

ESP-IDF component providing a browser-based drone controller. It tries to join
the configured Wi-Fi network first and falls back to its own access point when
that connection fails. The controller is then served over HTTP.

The embedded page source is kept in `controller.html` and can be edited
without changing the C implementation.

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

The access-point SSID, password, connection limit, optional existing-network
credentials, and connection timeout can be configured under
`Component config > Wi-Fi drone remote configuration`.

Leave the existing-network SSID empty to always use access-point mode. In
access-point mode, connect to the configured SSID and open
`http://192.168.4.1`.

```c
#include "esp32-wifi-drone-remote.h"

void app_main(void)
{
    esp32_wifi_drone_remote_config_t config =
        ESP32_WIFI_DRONE_REMOTE_DEFAULT_CONFIG();

    ESP_ERROR_CHECK(esp32_wifi_drone_remote_start(&config));
}
```

## Placeholder API

The page sends JSON `POST` requests to these endpoints:

| Endpoint | Control |
| --- | --- |
| `/api/connect` | Connect button |
| `/api/settings` | Settings button |
| `/api/turn-lock` | Turn-lock button |
| `/api/brightness` | Brightness button |
| `/api/headlight` | Headlight button |
| `/api/sound` | Sound button |
| `/api/left-stick` | Throttle/turn stick |
| `/api/right-stick` | Direction stick |

Stick request bodies have the form `{"x": 0.0, "y": 0.0}`, with both values
normalized to the range -1 through 1. All endpoint handlers currently consume
the request, log its URI, and return HTTP 204 without controlling hardware.
