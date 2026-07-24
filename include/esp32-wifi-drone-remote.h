#ifndef ESP32_WIFI_DRONE_REMOTE_H
#define ESP32_WIFI_DRONE_REMOTE_H

#include <stdint.h>

#include <esp_err.h>
#include <sdkconfig.h>

#ifdef __cplusplus
extern "C" {
#endif

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
    /** Existing network to join; an empty string selects access-point mode. */
    const char *station_ssid;
    /** Password for the existing network. */
    const char *station_password;
    /** Time to wait for a station IP address before access-point fallback. */
    uint32_t station_timeout_ms;
} esp32_wifi_drone_remote_config_t;

/**
 * @brief Default configuration populated from menuconfig.
 */
#define ESP32_WIFI_DRONE_REMOTE_DEFAULT_CONFIG()                       \
    {                                                                 \
        .ap_ssid = CONFIG_ESP32_WIFI_DRONE_REMOTE_AP_SSID,             \
        .ap_password = CONFIG_ESP32_WIFI_DRONE_REMOTE_AP_PASSWORD,     \
        .ap_max_connections = CONFIG_ESP32_WIFI_DRONE_REMOTE_MAX_CONNECTIONS, \
        .station_ssid = CONFIG_ESP32_WIFI_DRONE_REMOTE_STA_SSID,       \
        .station_password = CONFIG_ESP32_WIFI_DRONE_REMOTE_STA_PASSWORD, \
        .station_timeout_ms = CONFIG_ESP32_WIFI_DRONE_REMOTE_STA_TIMEOUT_MS, \
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
