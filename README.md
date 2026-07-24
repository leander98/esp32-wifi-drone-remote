# ESP32 Wi-Fi drone remote component

ESP-IDF component providing a browser-based drone controller. It tries to join
the configured Wi-Fi network first and falls back to its own access point when
that connection fails. The controller is then served over HTTP.

The embedded page source is kept in `controller.html` and can be edited
without changing the C implementation. The station setup form is stored
separately in `wifi-setup.html`.

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
`http://esp32drone.local`. The component advertises this hostname through
mDNS in both access-point and station modes. `http://192.168.4.1` remains
available as an access-point fallback.

The controller's **Connect** button opens `/wifi`, where station credentials
can be saved from a phone. Submitted credentials are stored in NVS and take
priority over the menuconfig defaults. The ESP32 restarts, attempts the saved
network, and returns to access-point mode if that attempt fails.

The controller is optimized for landscape phone use, respects display safe
areas, limits joystick requests to 20 updates per second, and provides a
fullscreen button. Fullscreen and orientation locking depend on browser
support and require a user gesture.

The Brightness button toggles a persistent light/dark theme while continuing
to invoke the configurable `/api/brightness` control endpoint. Fullscreen
mode displays Wi-Fi signal bars and HTTP round-trip latency to the ESP32. In
station mode RSSI is measured by the ESP32 against its upstream access point;
in access-point mode RSSI is selected for the phone requesting the status.
Signal bars turn red when latency exceeds the configured threshold, which
defaults to 150 ms.

The Settings page contains a Wi-Fi access-point submenu available in both
connection modes. AP SSID, password/open-network mode, maximum client count,
and transmit power are stored in NVS and applied after the automatic restart.

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
| `/api/turn-lock` | Turn-lock button |
| `/api/brightness` | Brightness button |
| `/api/headlight` | Headlight button |
| `/api/sound` | Sound button |
| `/api/left-stick` | Throttle/turn stick |
| `/api/right-stick` | Direction stick |
| `/api/wifi-config` | Saves station credentials and schedules a restart |
| `/api/telemetry` | Returns application-provided accelerometer and gyroscope vectors |

Stick request bodies have the form `{"x": 0.0, "y": 0.0}`, with both values
normalized to the range -1 through 1.

By default, controller requests are consumed, logged, and acknowledged with
HTTP 204 without controlling hardware. Set `api_handler` to receive them in
the application:

```c
static esp_err_t handle_remote_request(
    const char *uri,
    const uint8_t *body,
    size_t body_length,
    void *context)
{
    /* Dispatch by URI and parse the body as needed. */
    ESP_LOGI("remote", "%s received %u bytes",
             uri, (unsigned)body_length);
    return ESP_OK;
}

void app_main(void)
{
    esp32_wifi_drone_remote_config_t config =
        ESP32_WIFI_DRONE_REMOTE_DEFAULT_CONFIG();
    config.api_handler = handle_remote_request;
    config.api_context = NULL;

    ESP_ERROR_CHECK(esp32_wifi_drone_remote_start(&config));
}
```

The callback runs in the HTTP server task and should return quickly. Request
data is valid only until the callback returns. Returning an error produces an
HTTP 500 response; returning `ESP_OK` produces HTTP 204. Controller request
bodies are limited to 256 bytes.

Set `latency_handler`, `latency_context`, and `latency_timeout_ms` in the same
configuration structure to receive browser round-trip samples. The callback
receives the latency and a boolean indicating whether the threshold was
exceeded, allowing flight-control timeout behavior to be added externally.

Set `telemetry_handler` and `telemetry_context` to provide the most recent
accelerometer and gyroscope XYZ values. The browser polls `/api/telemetry` at
10 Hz and performs complementary-filter attitude estimation and artificial
horizon rendering entirely on the client. If no provider is configured, the
endpoint returns HTTP 503.
