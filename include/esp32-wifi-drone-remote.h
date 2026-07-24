#ifndef ESP32_WIFI_DRONE_REMOTE_H
#define ESP32_WIFI_DRONE_REMOTE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <esp_err.h>
#include <sdkconfig.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handle a request from one of the drone controller API endpoints.
 *
 * The callback runs in the ESP-IDF HTTP server task. The URI and body pointers
 * are only valid for the duration of the callback.
 *
 * @param uri Requested endpoint, for example `/api/left-stick`.
 * @param body Raw request body; not guaranteed to be null-terminated.
 * @param body_length Number of valid bytes in @p body.
 * @param context Application context configured in
 *                esp32_wifi_drone_remote_config_t.
 * @return ESP_OK when the request was accepted, otherwise an ESP-IDF error.
 */
typedef esp_err_t (*esp32_wifi_drone_remote_api_handler_t)(
    const char *uri,
    const uint8_t *body,
    size_t body_length,
    void *context);

/**
 * @brief Receive browser-to-ESP32 latency measurements.
 *
 * @param latency_ms Measured HTTP round-trip latency in milliseconds.
 * @param timeout_exceeded True when latency exceeds the configured threshold.
 * @param context Application context from the component configuration.
 */
typedef void (*esp32_wifi_drone_remote_latency_handler_t)(
    uint32_t latency_ms,
    bool timeout_exceeded,
    void *context);

/**
 * @brief IMU values consumed by the browser telemetry display.
 */
typedef struct {
    float acceleration_x;
    float acceleration_y;
    float acceleration_z;
    float gyroscope_x;
    float gyroscope_y;
    float gyroscope_z;
} esp32_wifi_drone_remote_telemetry_t;

/**
 * @brief Supply the latest IMU values for a telemetry HTTP request.
 *
 * The callback runs in the HTTP server task and must return quickly.
 *
 * @param telemetry Destination populated by the application.
 * @param context Application context from the component configuration.
 * @return ESP_OK when telemetry is available, otherwise an ESP-IDF error.
 */
typedef esp_err_t (*esp32_wifi_drone_remote_telemetry_handler_t)(
    esp32_wifi_drone_remote_telemetry_t *telemetry,
    void *context);

/**
 * @brief Runtime configuration for the Wi-Fi drone remote.
 */
typedef struct {
    /** SSID advertised when the component runs as an access point. */
    const char *ap_ssid;
    /** Access-point password; an empty string creates an open network. */
    const char *ap_password;
    /** Maximum number of stations accepted by the access point. */
    uint8_t ap_max_connections;
    /** Maximum access-point transmit power in quarter-dBm units. */
    int8_t ap_tx_power_quarter_dbm;
    /** Existing network to join; an empty string selects access-point mode. */
    const char *station_ssid;
    /** Password for the existing network. */
    const char *station_password;
    /** Time to wait for a station IP address before access-point fallback. */
    uint32_t station_timeout_ms;
    /** Optional controller endpoint callback; NULL uses the placeholder. */
    esp32_wifi_drone_remote_api_handler_t api_handler;
    /** Application value passed to api_handler on every request. */
    void *api_context;
    /** Optional callback receiving browser round-trip latency measurements. */
    esp32_wifi_drone_remote_latency_handler_t latency_handler;
    /** Application value passed to latency_handler. */
    void *latency_context;
    /** Latency above this value is reported as a timeout. */
    uint32_t latency_timeout_ms;
    /** Optional callback supplying accelerometer and gyroscope values. */
    esp32_wifi_drone_remote_telemetry_handler_t telemetry_handler;
    /** Application value passed to telemetry_handler. */
    void *telemetry_context;
} esp32_wifi_drone_remote_config_t;

/**
 * @brief Default configuration populated from menuconfig.
 */
#define ESP32_WIFI_DRONE_REMOTE_DEFAULT_CONFIG()                       \
    {                                                                 \
        .ap_ssid = CONFIG_ESP32_WIFI_DRONE_REMOTE_AP_SSID,             \
        .ap_password = CONFIG_ESP32_WIFI_DRONE_REMOTE_AP_PASSWORD,     \
        .ap_max_connections = CONFIG_ESP32_WIFI_DRONE_REMOTE_MAX_CONNECTIONS, \
        .ap_tx_power_quarter_dbm = CONFIG_ESP32_WIFI_DRONE_REMOTE_AP_TX_POWER, \
        .station_ssid = CONFIG_ESP32_WIFI_DRONE_REMOTE_STA_SSID,       \
        .station_password = CONFIG_ESP32_WIFI_DRONE_REMOTE_STA_PASSWORD, \
        .station_timeout_ms = CONFIG_ESP32_WIFI_DRONE_REMOTE_STA_TIMEOUT_MS, \
        .api_handler = NULL,                                            \
        .api_context = NULL,                                            \
        .latency_handler = NULL,                                        \
        .latency_context = NULL,                                        \
        .latency_timeout_ms = 150,                                      \
        .telemetry_handler = NULL,                                      \
        .telemetry_context = NULL,                                      \
    }

/**
 * @brief Start Wi-Fi and the drone remote web server.
 *
 * When a station SSID is configured, the component first tries to join that
 * network. It falls back to its own access point if the attempt fails or
 * times out.
 *
 * @param config Remote configuration. The referenced strings must remain
 *               valid until esp32_wifi_drone_remote_stop() is called.
 * @return ESP_OK on success, otherwise an ESP-IDF error.
 */
esp_err_t esp32_wifi_drone_remote_start(
    const esp32_wifi_drone_remote_config_t *config);

/**
 * @brief Stop the drone remote web server and release its Wi-Fi resources.
 *
 * @return ESP_OK on success, otherwise an ESP-IDF error.
 */
esp_err_t esp32_wifi_drone_remote_stop(void);

#ifdef __cplusplus
}
#endif

#endif
