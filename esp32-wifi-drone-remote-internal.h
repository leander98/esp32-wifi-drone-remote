/**
 * @file esp32-wifi-drone-remote-internal.h
 * @brief Private contracts shared by the Wi-Fi remote implementation modules.
 */
#pragma once

#include <esp_err.h>
#include <esp_http_server.h>

#include "esp32-wifi-drone-remote.h"

#define WIFI_REMOTE_SSID_SIZE 33U
#define WIFI_REMOTE_PASSWORD_SIZE 65U

esp_err_t save_station_credentials(const char *ssid, const char *password);
void apply_saved_network_config(
    const esp32_wifi_drone_remote_config_t *config,
    esp32_wifi_drone_remote_config_t *effective);

esp_err_t page_handler(httpd_req_t *request);
esp_err_t wifi_setup_page_handler(httpd_req_t *request);
esp_err_t settings_page_handler(httpd_req_t *request);
esp_err_t ap_settings_page_handler(httpd_req_t *request);
esp_err_t imu_settings_page_handler(httpd_req_t *request);
esp_err_t pid_settings_page_handler(httpd_req_t *request);
esp_err_t esc_settings_page_handler(httpd_req_t *request);
esp_err_t esc_pwm_channel_settings_page_handler(httpd_req_t *request);
esp_err_t esc_manual_page_handler(httpd_req_t *request);
esp_err_t esc_programming_page_handler(httpd_req_t *request);
