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
    const char *ssid;
    const char *password;
    uint8_t max_connections;
} esp32_wifi_drone_remote_config_t;

/**
 * @brief Default configuration populated from menuconfig.
 */
#define ESP32_WIFI_DRONE_REMOTE_DEFAULT_CONFIG()                       \
    {                                                                 \
        .ssid = CONFIG_ESP32_WIFI_DRONE_REMOTE_AP_SSID,                \
        .password = CONFIG_ESP32_WIFI_DRONE_REMOTE_AP_PASSWORD,        \
        .max_connections = CONFIG_ESP32_WIFI_DRONE_REMOTE_MAX_CONNECTIONS, \
    }

/**
 * @brief Start the Wi-Fi access point and drone remote web server.
 *
 * This is the public entry point reserved by the component template.
 *
 * @param config Remote configuration. The referenced strings must remain
 *               valid until esp32_wifi_drone_remote_stop() is called.
 * @return ESP_ERR_NOT_SUPPORTED until the component is implemented.
 */
esp_err_t esp32_wifi_drone_remote_start(
    const esp32_wifi_drone_remote_config_t *config);

/**
 * @brief Stop the drone remote web server and Wi-Fi access point.
 *
 * @return ESP_ERR_NOT_SUPPORTED until the component is implemented.
 */
esp_err_t esp32_wifi_drone_remote_stop(void);

#ifdef __cplusplus
}
#endif

#endif
