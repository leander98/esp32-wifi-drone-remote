#include <esp_err.h>
#include <esp_log.h>

#include "esp32-wifi-drone-remote.h"

static const char *TAG = "wifi-drone-remote";

esp_err_t esp32_wifi_drone_remote_start(
    const esp32_wifi_drone_remote_config_t *config)
{
    if (config == NULL || config->ssid == NULL || config->password == NULL ||
        config->max_connections == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGW(TAG, "Wi-Fi drone remote is not implemented yet");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t esp32_wifi_drone_remote_stop(void)
{
    ESP_LOGW(TAG, "Wi-Fi drone remote is not implemented yet");
    return ESP_ERR_NOT_SUPPORTED;
}
