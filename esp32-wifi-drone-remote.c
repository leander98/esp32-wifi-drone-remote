#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <esp_check.h>
#include <esp_event.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_wifi_ap_get_sta_list.h>
#include <esp_wifi_default.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <mdns.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <lwip/sockets.h>

#include "esp32-wifi-drone-remote.h"

static const char *TAG = "wifi-drone-remote";
static const EventBits_t WIFI_CONNECTED_BIT = BIT0;
static const EventBits_t WIFI_FAILED_BIT = BIT1;
static const char *NETWORK_HOSTNAME = "esp32drone";
static const char *WIFI_CREDENTIALS_PARTITION = "wifi_creds";
static const char *WIFI_NVS_NAMESPACE = "drone_remote";

static EventGroupHandle_t s_wifi_events;
static esp_event_handler_instance_t s_wifi_handler;
static esp_event_handler_instance_t s_ip_handler;
static esp_netif_t *s_netif;
static httpd_handle_t s_server;
static bool s_started;
static bool s_mdns_started;
static bool s_wifi_scan_in_progress;
static wifi_mode_t s_wifi_mode = WIFI_MODE_NULL;
static esp32_wifi_drone_remote_api_handler_t s_api_handler;
static void *s_api_context;
static esp32_wifi_drone_remote_latency_handler_t s_latency_handler;
static void *s_latency_context;
static uint32_t s_latency_timeout_ms = 150U;
static esp32_wifi_drone_remote_telemetry_handler_t s_telemetry_handler;
static void *s_telemetry_context;
static esp32_wifi_drone_remote_imu_get_handler_t s_imu_get_handler;
static esp32_wifi_drone_remote_imu_set_handler_t s_imu_set_handler;
static void *s_imu_context;
static esp32_wifi_drone_remote_esc_get_handler_t s_esc_get_handler;
static esp32_wifi_drone_remote_esc_set_handler_t s_esc_set_handler;
static esp32_wifi_drone_remote_esc_throttle_handler_t s_esc_throttle_handler;
static esp32_wifi_drone_remote_esc_program_handler_t s_esc_program_handler;
static void *s_esc_context;
static char s_saved_station_ssid[33];
static char s_saved_station_password[65];
static char s_saved_ap_ssid[33];
static char s_saved_ap_password[65];
static esp32_wifi_drone_remote_config_t s_effective_config;

extern const uint8_t controller_html_start[]
    asm("_binary_controller_html_start");
extern const uint8_t wifi_setup_html_start[]
    asm("_binary_wifi_setup_html_start");
extern const uint8_t settings_html_start[]
    asm("_binary_settings_html_start");
extern const uint8_t wifi_ap_settings_html_start[]
    asm("_binary_wifi_ap_settings_html_start");
extern const uint8_t imu_settings_html_start[]
    asm("_binary_imu_settings_html_start");
extern const uint8_t esc_settings_html_start[]
    asm("_binary_esc_settings_html_start");
extern const uint8_t esc_pwm_channel_settings_html_start[]
    asm("_binary_esc_pwm_channel_settings_html_start");
extern const uint8_t esc_manual_html_start[]
    asm("_binary_esc_manual_html_start");
extern const uint8_t esc_programming_html_start[]
    asm("_binary_esc_programming_html_start");

/**
 * @brief Advertise the controller through multicast DNS.
 *
 * @return ESP_OK on success, otherwise an mDNS initialization error.
 */
static esp_err_t start_mdns(void)
{
    ESP_RETURN_ON_ERROR(mdns_init(), TAG, "Could not initialize mDNS");
    s_mdns_started = true;
    ESP_RETURN_ON_ERROR(mdns_hostname_set(NETWORK_HOSTNAME), TAG,
                        "Could not set mDNS hostname");
    ESP_RETURN_ON_ERROR(
        mdns_instance_name_set("ESP32 Drone Remote"), TAG,
        "Could not set mDNS instance name");
    ESP_RETURN_ON_ERROR(
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0), TAG,
        "Could not advertise HTTP service");
    ESP_LOGI(TAG, "Controller available at http://%s.local",
             NETWORK_HOSTNAME);
    return ESP_OK;
}

/**
 * @brief Process Wi-Fi station lifecycle events.
 *
 * Starts the station connection after Wi-Fi initialization and signals the
 * waiting startup task when the station either receives an IP address or is
 * disconnected.
 *
 * @param arg Unused event-handler context.
 * @param event_base Event source (`WIFI_EVENT` or `IP_EVENT`).
 * @param event_id Event identifier within the source.
 * @param event_data Unused event-specific data.
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START &&
        !s_wifi_scan_in_progress) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupSetBits(s_wifi_events, WIFI_FAILED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief Serve the embedded controller page.
 *
 * @param request HTTP GET request for the root URI.
 * @return ESP_OK when the response is sent, otherwise an HTTP-server error.
 */
static esp_err_t page_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html");
    httpd_resp_set_hdr(request, "Cache-Control",
                       "no-store, no-cache, must-revalidate");
    return httpd_resp_send(request, (const char *)controller_html_start,
                           HTTPD_RESP_USE_STRLEN);
}

/**
 * @brief Serve the embedded station configuration page.
 *
 * @param request HTTP GET request for `/wifi`.
 * @return ESP_OK when the response is sent, otherwise an HTTP-server error.
 */
static esp_err_t wifi_setup_page_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html");
    httpd_resp_set_hdr(request, "Cache-Control",
                       "no-store, no-cache, must-revalidate");
    return httpd_resp_send(request, (const char *)wifi_setup_html_start,
                           HTTPD_RESP_USE_STRLEN);
}

/** @brief Serve the embedded settings menu. */
static esp_err_t settings_page_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html");
    httpd_resp_set_hdr(request, "Cache-Control",
                       "no-store, no-cache, must-revalidate");
    return httpd_resp_send(request, (const char *)settings_html_start,
                           HTTPD_RESP_USE_STRLEN);
}

/** @brief Serve the embedded Wi-Fi access-point settings page. */
static esp_err_t ap_settings_page_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html");
    httpd_resp_set_hdr(request, "Cache-Control",
                       "no-store, no-cache, must-revalidate");
    return httpd_resp_send(request, (const char *)wifi_ap_settings_html_start,
                           HTTPD_RESP_USE_STRLEN);
}

/** @brief Serve the embedded ISM330DLC settings page. */
static esp_err_t imu_settings_page_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html");
    httpd_resp_set_hdr(request, "Cache-Control",
                       "no-store, no-cache, must-revalidate");
    return httpd_resp_send(request, (const char *)imu_settings_html_start,
                           HTTPD_RESP_USE_STRLEN);
}

/** @brief Serve the XW30A settings menu. */
static esp_err_t esc_settings_page_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html");
    httpd_resp_set_hdr(request, "Cache-Control",
                       "no-store, no-cache, must-revalidate");
    return httpd_resp_send(request, (const char *)esc_settings_html_start,
                           HTTPD_RESP_USE_STRLEN);
}

/** @brief Serve the XW30A PWM-channel settings page. */
static esp_err_t esc_pwm_channel_settings_page_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html");
    httpd_resp_set_hdr(request, "Cache-Control",
                       "no-store, no-cache, must-revalidate");
    return httpd_resp_send(
        request, (const char *)esc_pwm_channel_settings_html_start,
        HTTPD_RESP_USE_STRLEN);
}

/** @brief Serve the manual XW30A throttle-control page. */
static esp_err_t esc_manual_page_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html");
    httpd_resp_set_hdr(request, "Cache-Control",
                       "no-store, no-cache, must-revalidate");
    return httpd_resp_send(request, (const char *)esc_manual_html_start,
                           HTTPD_RESP_USE_STRLEN);
}

/** @brief Serve the guided XW30A programming page. */
static esp_err_t esc_programming_page_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html");
    httpd_resp_set_hdr(request, "Cache-Control",
                       "no-store, no-cache, must-revalidate");
    return httpd_resp_send(request, (const char *)esc_programming_html_start,
                           HTTPD_RESP_USE_STRLEN);
}

/**
 * @brief Return the Wi-Fi mode and RSSI visible to the requesting browser.
 *
 * In station mode the RSSI belongs to the upstream access point. In
 * access-point mode it belongs to the station making this HTTP request.
 *
 * @param request HTTP GET request for `/api/status`.
 * @return ESP_OK after responding, otherwise an HTTP-server error.
 */
static esp_err_t status_handler(httpd_req_t *request)
{
    int rssi = -127;
    const char *mode = "unknown";

    if (s_wifi_mode == WIFI_MODE_STA) {
        wifi_ap_record_t access_point;
        mode = "sta";
        if (esp_wifi_sta_get_ap_info(&access_point) == ESP_OK) {
            rssi = access_point.rssi;
        }
    } else if (s_wifi_mode == WIFI_MODE_AP) {
        wifi_sta_list_t stations;
        mode = "ap";
        if (esp_wifi_ap_get_sta_list(&stations) == ESP_OK &&
            stations.num > 0) {
            int selected = 0;
            struct sockaddr_in peer;
            socklen_t peer_length = sizeof(peer);
            int socket = httpd_req_to_sockfd(request);
            wifi_sta_mac_ip_list_t station_ips;

            if (getpeername(socket, (struct sockaddr *)&peer,
                            &peer_length) == 0 &&
                esp_wifi_ap_get_sta_list_with_ip(&stations, &station_ips) ==
                    ESP_OK) {
                for (int i = 0; i < stations.num; ++i) {
                    if (station_ips.sta[i].ip.addr ==
                        peer.sin_addr.s_addr) {
                        selected = i;
                        break;
                    }
                }
            }
            rssi = stations.sta[selected].rssi;
        }
    }

    char response[80];
    int length = snprintf(response, sizeof(response),
                          "{\"mode\":\"%s\",\"rssi\":%d,\"timeout_ms\":%lu}",
                          mode, rssi,
                          (unsigned long)s_latency_timeout_ms);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, response, length);
}

/**
 * @brief Receive a browser latency sample and notify the application.
 */
static esp_err_t latency_handler(httpd_req_t *request)
{
    char body[48];
    if (request->content_len == 0U ||
        request->content_len >= sizeof(body)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid latency sample");
    }
    int received = httpd_req_recv(request, body, request->content_len);
    if (received != (int)request->content_len) {
        return ESP_FAIL;
    }
    body[received] = '\0';

    unsigned long latency_ms;
    if (sscanf(body, "{\"latency_ms\":%lu}", &latency_ms) != 1 ||
        latency_ms > UINT32_MAX) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid latency sample");
    }
    if (s_latency_handler != NULL) {
        s_latency_handler((uint32_t)latency_ms,
                          latency_ms > s_latency_timeout_ms,
                          s_latency_context);
    }
    httpd_resp_set_status(request, "204 No Content");
    return httpd_resp_send(request, NULL, 0);
}

/**
 * @brief Return application-provided IMU vectors as JSON.
 */
static esp_err_t telemetry_handler(httpd_req_t *request)
{
    if (s_telemetry_handler == NULL) {
        httpd_resp_set_status(request, "503 Service Unavailable");
        return httpd_resp_sendstr(request,
                                  "Telemetry provider is not configured");
    }

    esp32_wifi_drone_remote_telemetry_t telemetry = { 0 };
    esp_err_t err = s_telemetry_handler(&telemetry, s_telemetry_context);
    if (err != ESP_OK) {
        httpd_resp_set_status(request, "503 Service Unavailable");
        return httpd_resp_sendstr(request, esp_err_to_name(err));
    }

    char response[224];
    int length = snprintf(
        response, sizeof(response),
        "{\"accelerometer\":{\"x\":%.5f,\"y\":%.5f,\"z\":%.5f},"
        "\"gyroscope\":{\"x\":%.5f,\"y\":%.5f,\"z\":%.5f}}",
        telemetry.acceleration_x, telemetry.acceleration_y,
        telemetry.acceleration_z, telemetry.gyroscope_x,
        telemetry.gyroscope_y, telemetry.gyroscope_z);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, response, length);
}

/** @brief Return current application-provided IMU acquisition settings. */
static esp_err_t imu_config_get_handler(httpd_req_t *request)
{
    if (s_imu_get_handler == NULL) {
        httpd_resp_set_status(request, "503 Service Unavailable");
        return httpd_resp_sendstr(request, "IMU settings API is not configured");
    }
    esp32_wifi_drone_remote_imu_config_t config;
    esp_err_t err = s_imu_get_handler(&config, s_imu_context);
    if (err != ESP_OK) {
        httpd_resp_set_status(request, "503 Service Unavailable");
        return httpd_resp_sendstr(request, esp_err_to_name(err));
    }
    char response[128];
    int length = snprintf(
        response, sizeof(response),
        "{\"accel_odr\":%u,\"gyro_odr\":%u,"
        "\"accel_scale\":%u,\"gyro_scale\":%u}",
        config.accelerometer_odr, config.gyroscope_odr,
        config.accelerometer_full_scale, config.gyroscope_full_scale);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, response, length);
}

/** @brief Validate and dispatch submitted IMU acquisition settings. */
static esp_err_t imu_config_post_handler(httpd_req_t *request)
{
    char body[128];
    char accel_odr[4], gyro_odr[4], accel_scale[3], gyro_scale[3];
    if (request->content_len == 0U ||
        request->content_len >= sizeof(body)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid IMU settings");
    }
    int received = httpd_req_recv(request, body, request->content_len);
    if (received != (int)request->content_len) {
        return ESP_FAIL;
    }
    body[received] = '\0';
    if (httpd_query_key_value(body, "accel_odr", accel_odr,
                              sizeof(accel_odr)) != ESP_OK ||
        httpd_query_key_value(body, "gyro_odr", gyro_odr,
                              sizeof(gyro_odr)) != ESP_OK ||
        httpd_query_key_value(body, "accel_scale", accel_scale,
                              sizeof(accel_scale)) != ESP_OK ||
        httpd_query_key_value(body, "gyro_scale", gyro_scale,
                              sizeof(gyro_scale)) != ESP_OK) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid IMU settings");
    }
    esp32_wifi_drone_remote_imu_config_t config = {
        .accelerometer_odr = (uint8_t)atoi(accel_odr),
        .gyroscope_odr = (uint8_t)atoi(gyro_odr),
        .accelerometer_full_scale = (uint8_t)atoi(accel_scale),
        .gyroscope_full_scale = (uint8_t)atoi(gyro_scale),
    };
    if (config.accelerometer_odr > 10U || config.gyroscope_odr > 10U ||
        config.accelerometer_full_scale > 3U ||
        config.gyroscope_full_scale > 3U ||
        s_imu_set_handler == NULL) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Unsupported IMU settings");
    }
    esp_err_t err = s_imu_set_handler(&config, s_imu_context);
    if (err != ESP_OK) {
        return httpd_resp_send_err(request,
                                   HTTPD_500_INTERNAL_SERVER_ERROR,
                                   esp_err_to_name(err));
    }
    httpd_resp_set_status(request, "204 No Content");
    return httpd_resp_send(request, NULL, 0);
}

/** @brief Return the active configuration for a selected ESC channel. */
static esp_err_t esc_config_get_handler(httpd_req_t *request)
{
    char query[32];
    char index_text[4];
    if (s_esc_get_handler == NULL ||
        httpd_req_get_url_query_str(request, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "index", index_text,
                              sizeof(index_text)) != ESP_OK) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "ESC index is required");
    }
    unsigned long index = strtoul(index_text, NULL, 10);
    if (index >= ESP32_WIFI_DRONE_REMOTE_ESC_COUNT) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid ESC index");
    }

    esp32_wifi_drone_remote_esc_config_t config = {
        .index = (uint8_t)index,
    };
    esp_err_t err = s_esc_get_handler(&config, s_esc_context);
    if (err != ESP_OK) {
        return httpd_resp_send_err(request,
                                   HTTPD_500_INTERNAL_SERVER_ERROR,
                                   esp_err_to_name(err));
    }
    char response[192];
    int length = snprintf(
        response, sizeof(response),
        "{\"index\":%u,\"gpio\":%u,\"frequency\":%u,"
        "\"min_pulse\":%u,\"max_pulse\":%u,\"calibration_ms\":%u}",
        config.index, config.signal_gpio, config.pwm_frequency_hz,
        config.min_pulse_us, config.max_pulse_us,
        config.calibration_high_time_ms);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, response, length);
}

/** @brief Validate and dispatch a selected ESC channel's configuration. */
static esp_err_t esc_config_post_handler(httpd_req_t *request)
{
    char body[192];
    char index[4], gpio[4], frequency[8], min_pulse[8], max_pulse[8];
    char calibration_ms[8];
    if (request->content_len == 0U ||
        request->content_len >= sizeof(body)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid ESC settings");
    }
    int received = httpd_req_recv(request, body, request->content_len);
    if (received != (int)request->content_len) {
        return ESP_FAIL;
    }
    body[received] = '\0';
    if (httpd_query_key_value(body, "index", index, sizeof(index)) != ESP_OK ||
        httpd_query_key_value(body, "gpio", gpio, sizeof(gpio)) != ESP_OK ||
        httpd_query_key_value(body, "frequency", frequency,
                              sizeof(frequency)) != ESP_OK ||
        httpd_query_key_value(body, "min_pulse", min_pulse,
                              sizeof(min_pulse)) != ESP_OK ||
        httpd_query_key_value(body, "max_pulse", max_pulse,
                              sizeof(max_pulse)) != ESP_OK ||
        httpd_query_key_value(body, "calibration_ms", calibration_ms,
                              sizeof(calibration_ms)) != ESP_OK ||
        s_esc_set_handler == NULL) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid ESC settings");
    }
    unsigned long values[] = {
        strtoul(index, NULL, 10), strtoul(gpio, NULL, 10),
        strtoul(frequency, NULL, 10), strtoul(min_pulse, NULL, 10),
        strtoul(max_pulse, NULL, 10), strtoul(calibration_ms, NULL, 10),
    };
    if (values[0] >= ESP32_WIFI_DRONE_REMOTE_ESC_COUNT ||
        values[1] > UINT8_MAX || values[2] == 0U || values[2] > UINT16_MAX ||
        values[3] == 0U || values[3] > UINT16_MAX ||
        values[4] <= values[3] || values[4] > UINT16_MAX ||
        values[5] == 0U || values[5] > UINT16_MAX ||
        (1000000UL / values[2]) <= values[4]) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Unsupported ESC settings");
    }
    const esp32_wifi_drone_remote_esc_config_t config = {
        .index = values[0],
        .signal_gpio = values[1],
        .pwm_frequency_hz = values[2],
        .min_pulse_us = values[3],
        .max_pulse_us = values[4],
        .calibration_high_time_ms = values[5],
    };
    esp_err_t err = s_esc_set_handler(&config, s_esc_context);
    if (err != ESP_OK) {
        return httpd_resp_send_err(request,
                                   HTTPD_500_INTERNAL_SERVER_ERROR,
                                   esp_err_to_name(err));
    }
    httpd_resp_set_status(request, "204 No Content");
    return httpd_resp_send(request, NULL, 0);
}

/** @brief Apply a manual normalized throttle value to one ESC channel. */
static esp_err_t esc_throttle_post_handler(httpd_req_t *request)
{
    char body[64];
    char index_text[4], throttle_text[8];
    if (request->content_len == 0U ||
        request->content_len >= sizeof(body)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid ESC throttle");
    }
    int received = httpd_req_recv(request, body, request->content_len);
    if (received != (int)request->content_len) {
        return ESP_FAIL;
    }
    body[received] = '\0';
    if (httpd_query_key_value(body, "index", index_text,
                              sizeof(index_text)) != ESP_OK ||
        httpd_query_key_value(body, "throttle", throttle_text,
                              sizeof(throttle_text)) != ESP_OK ||
        s_esc_throttle_handler == NULL) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid ESC throttle");
    }
    unsigned long index = strtoul(index_text, NULL, 10);
    unsigned long throttle = strtoul(throttle_text, NULL, 10);
    if (index >= ESP32_WIFI_DRONE_REMOTE_ESC_COUNT || throttle > 1000U) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Unsupported ESC throttle");
    }
    esp_err_t err = s_esc_throttle_handler(
        (uint8_t)index, (float)throttle / 1000.0f, s_esc_context);
    if (err != ESP_OK) {
        return httpd_resp_send_err(request,
                                   HTTPD_500_INTERNAL_SERVER_ERROR,
                                   esp_err_to_name(err));
    }
    httpd_resp_set_status(request, "204 No Content");
    return httpd_resp_send(request, NULL, 0);
}

/** @brief Dispatch one guided ESC programming step. */
static esp_err_t esc_programming_post_handler(httpd_req_t *request)
{
    char body[64];
    char index_text[4], action_text[4], selection_text[4];
    if (request->content_len == 0U ||
        request->content_len >= sizeof(body)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid programming command");
    }
    int received = httpd_req_recv(request, body, request->content_len);
    if (received != (int)request->content_len) {
        return ESP_FAIL;
    }
    body[received] = '\0';
    if (httpd_query_key_value(body, "index", index_text,
                              sizeof(index_text)) != ESP_OK ||
        httpd_query_key_value(body, "action", action_text,
                              sizeof(action_text)) != ESP_OK ||
        httpd_query_key_value(body, "selection", selection_text,
                              sizeof(selection_text)) != ESP_OK ||
        s_esc_program_handler == NULL) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid programming command");
    }
    unsigned long index = strtoul(index_text, NULL, 10);
    unsigned long action = strtoul(action_text, NULL, 10);
    unsigned long selection = strtoul(selection_text, NULL, 10);
    if (index >= ESP32_WIFI_DRONE_REMOTE_ESC_COUNT ||
        action > ESP32_WIFI_ESC_PROGRAM_CANCEL || selection > UINT8_MAX) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Unsupported programming command");
    }
    esp_err_t err = s_esc_program_handler(
        (uint8_t)index,
        (esp32_wifi_drone_remote_esc_program_action_t)action,
        (uint8_t)selection, s_esc_context);
    if (err != ESP_OK) {
        return httpd_resp_send_err(request,
                                   HTTPD_500_INTERNAL_SERVER_ERROR,
                                   esp_err_to_name(err));
    }
    httpd_resp_set_status(request, "204 No Content");
    return httpd_resp_send(request, NULL, 0);
}

/**
 * @brief Convert a hexadecimal URL-encoding digit to its numeric value.
 *
 * @param character ASCII hexadecimal character.
 * @return Value from 0 through 15, or -1 when the character is invalid.
 */
static int url_hex_value(char character)
{
    if (character >= '0' && character <= '9') {
        return character - '0';
    }
    if (character >= 'a' && character <= 'f') {
        return character - 'a' + 10;
    }
    if (character >= 'A' && character <= 'F') {
        return character - 'A' + 10;
    }
    return -1;
}

/**
 * @brief Decode an application/x-www-form-urlencoded value in place.
 *
 * @param value Null-terminated encoded value to decode.
 * @return ESP_OK on success or ESP_ERR_INVALID_ARG for malformed `%XX` data.
 */
static esp_err_t url_decode(char *value)
{
    char *source = value;
    char *destination = value;

    while (*source != '\0') {
        if (*source == '+') {
            *destination++ = ' ';
            ++source;
        } else if (*source == '%') {
            if (source[1] == '\0' || source[2] == '\0') {
                return ESP_ERR_INVALID_ARG;
            }
            int high = url_hex_value(source[1]);
            int low = url_hex_value(source[2]);
            if (high < 0 || low < 0) {
                return ESP_ERR_INVALID_ARG;
            }
            *destination++ = (char)((high << 4) | low);
            source += 3;
        } else {
            *destination++ = *source++;
        }
    }
    *destination = '\0';
    return ESP_OK;
}

/**
 * @brief Restart the ESP32 after allowing the HTTP response to reach the phone.
 *
 * @param context Unused task context.
 */
static void delayed_restart_task(void *context)
{
    (void)context;
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();
}

/** @brief Escape one Wi-Fi SSID for inclusion in a JSON string. */
static bool json_escape_ssid(const uint8_t *ssid, char *escaped, size_t size)
{
    size_t output = 0U;
    for (size_t input = 0U; ssid[input] != '\0'; ++input) {
        uint8_t value = ssid[input];
        if (value == '"' || value == '\\') {
            if (output + 2U >= size) {
                return false;
            }
            escaped[output++] = '\\';
            escaped[output++] = (char)value;
        } else if (value < 0x20U) {
            if (output + 6U >= size) {
                return false;
            }
            int written = snprintf(escaped + output, size - output,
                                   "\\u%04x", value);
            if (written != 6) {
                return false;
            }
            output += 6U;
        } else {
            if (output + 1U >= size) {
                return false;
            }
            escaped[output++] = (char)value;
        }
    }
    escaped[output] = '\0';
    return true;
}

/**
 * @brief Scan for nearby station networks and return them as JSON.
 *
 * AP-only operation is temporarily changed to AP+station mode so the access
 * point remains available while the station interface performs the scan.
 */
static esp_err_t wifi_scan_handler(httpd_req_t *request)
{
    wifi_mode_t original_mode;
    esp_err_t err = esp_wifi_get_mode(&original_mode);
    if (err != ESP_OK) {
        return httpd_resp_send_err(request,
                                   HTTPD_500_INTERNAL_SERVER_ERROR,
                                   esp_err_to_name(err));
    }

    bool restore_ap_mode = original_mode == WIFI_MODE_AP;
    if (restore_ap_mode) {
        s_wifi_scan_in_progress = true;
        err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    }

    const wifi_scan_config_t scan_config = {
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    };
    if (err == ESP_OK) {
        err = esp_wifi_scan_start(&scan_config, true);
    }

    wifi_ap_record_t records[20];
    uint16_t record_count = sizeof(records) / sizeof(records[0]);
    if (err == ESP_OK) {
        err = esp_wifi_scan_get_ap_records(&record_count, records);
    }
    if (restore_ap_mode) {
        esp_err_t restore_err = esp_wifi_set_mode(WIFI_MODE_AP);
        s_wifi_scan_in_progress = false;
        if (err == ESP_OK) {
            err = restore_err;
        }
    }
    if (err != ESP_OK) {
        return httpd_resp_send_err(request,
                                   HTTPD_500_INTERNAL_SERVER_ERROR,
                                   esp_err_to_name(err));
    }

    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    ESP_RETURN_ON_ERROR(httpd_resp_send_chunk(request, "[", 1), TAG,
                        "Could not start Wi-Fi scan response");

    bool first = true;
    for (uint16_t index = 0U; index < record_count; ++index) {
        if (records[index].ssid[0] == '\0') {
            continue;
        }
        bool duplicate = false;
        for (uint16_t previous = 0U; previous < index; ++previous) {
            if (strcmp((const char *)records[index].ssid,
                       (const char *)records[previous].ssid) == 0) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) {
            continue;
        }

        char escaped_ssid[193];
        char item[256];
        if (!json_escape_ssid(records[index].ssid, escaped_ssid,
                              sizeof(escaped_ssid))) {
            continue;
        }
        int length = snprintf(item, sizeof(item),
                              "%s{\"ssid\":\"%s\",\"rssi\":%d}",
                              first ? "" : ",", escaped_ssid,
                              records[index].rssi);
        if (length < 0 || (size_t)length >= sizeof(item)) {
            continue;
        }
        ESP_RETURN_ON_ERROR(
            httpd_resp_send_chunk(request, item, (size_t)length), TAG,
            "Could not send Wi-Fi scan result");
        first = false;
    }
    ESP_RETURN_ON_ERROR(httpd_resp_send_chunk(request, "]", 1), TAG,
                        "Could not finish Wi-Fi scan response");
    return httpd_resp_send_chunk(request, NULL, 0);
}

/**
 * @brief Persist station credentials in their dedicated NVS partition.
 */
static esp_err_t save_station_credentials(const char *ssid,
                                          const char *password)
{
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open_from_partition(
        WIFI_CREDENTIALS_PARTITION, WIFI_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_str(nvs, "sta_ssid", ssid);
    }
    if (err == ESP_OK) {
        err = nvs_set_str(nvs, "sta_password", password);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    if (nvs != 0) {
        nvs_close(nvs);
    }
    return err;
}

/**
 * @brief Store submitted station credentials and schedule a restart.
 *
 * The browser sends URL-encoded `ssid` and `password` fields. Credentials are
 * persisted in NVS and take precedence over menuconfig station credentials on
 * the next boot.
 *
 * @param request HTTP POST request for `/api/wifi-config`.
 * @return ESP_OK after responding, otherwise an ESP-IDF or HTTP-server error.
 */
static esp_err_t wifi_config_handler(httpd_req_t *request)
{
    /* URL encoding can expand every input byte to a three-byte %XX token. */
    char body[320];
    char ssid[sizeof(s_saved_station_ssid) * 3U];
    char password[sizeof(s_saved_station_password) * 3U];

    if (request->content_len == 0U ||
        request->content_len >= sizeof(body)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid form size");
    }

    size_t received_total = 0U;
    while (received_total < request->content_len) {
        int received = httpd_req_recv(
            request, body + received_total,
            request->content_len - received_total);
        if (received <= 0) {
            return ESP_FAIL;
        }
        received_total += (size_t)received;
    }
    body[received_total] = '\0';

    if (httpd_query_key_value(body, "ssid", ssid, sizeof(ssid)) != ESP_OK ||
        httpd_query_key_value(body, "password", password,
                              sizeof(password)) != ESP_OK ||
        url_decode(ssid) != ESP_OK || url_decode(password) != ESP_OK ||
        ssid[0] == '\0' ||
        strlen(ssid) >= sizeof(s_saved_station_ssid) ||
        strlen(password) >= sizeof(s_saved_station_password)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid Wi-Fi credentials");
    }

    esp_err_t err = save_station_credentials(ssid, password);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not save station credentials: %s",
                 esp_err_to_name(err));
        return httpd_resp_send_err(request,
                                   HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Could not save Wi-Fi credentials");
    }

    httpd_resp_set_type(request, "application/json");
    err = httpd_resp_sendstr(request, "{\"saved\":true}");
    if (err == ESP_OK &&
        xTaskCreate(delayed_restart_task, "wifi-restart", 2048, NULL, 5,
                    NULL) != pdPASS) {
        ESP_LOGE(TAG, "Could not create restart task");
    }
    return err;
}

/**
 * @brief Return the effective access-point settings without exposing its key.
 */
static esp_err_t ap_config_get_handler(httpd_req_t *request)
{
    char escaped_ssid[67];
    size_t output = 0U;
    for (const char *input = s_effective_config.ap_ssid;
         *input != '\0' && output + 2U < sizeof(escaped_ssid); ++input) {
        if (*input == '"' || *input == '\\') {
            escaped_ssid[output++] = '\\';
        }
        escaped_ssid[output++] = *input;
    }
    escaped_ssid[output] = '\0';

    char response[180];
    int length = snprintf(
        response, sizeof(response),
        "{\"ssid\":\"%s\",\"password_set\":%s,"
        "\"max_connections\":%u,\"tx_power\":%d}",
        escaped_ssid,
        s_effective_config.ap_password[0] != '\0' ? "true" : "false",
        s_effective_config.ap_max_connections,
        s_effective_config.ap_tx_power_quarter_dbm);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, response, length);
}

/**
 * @brief Persist access-point settings submitted by the settings page.
 */
static esp_err_t ap_config_post_handler(httpd_req_t *request)
{
    char body[512];
    char ssid[sizeof(s_saved_ap_ssid) * 3U];
    char password[sizeof(s_saved_ap_password) * 3U] = "";
    char clients_text[4];
    char power_text[4];
    char open_text[2];

    if (request->content_len == 0U ||
        request->content_len >= sizeof(body)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid form size");
    }
    size_t received_total = 0U;
    while (received_total < request->content_len) {
        int received = httpd_req_recv(
            request, body + received_total,
            request->content_len - received_total);
        if (received <= 0) {
            return ESP_FAIL;
        }
        received_total += (size_t)received;
    }
    body[received_total] = '\0';

    bool open_network =
        httpd_query_key_value(body, "open", open_text, sizeof(open_text)) ==
        ESP_OK;
    bool password_supplied =
        httpd_query_key_value(body, "password", password,
                              sizeof(password)) == ESP_OK;
    if (httpd_query_key_value(body, "ssid", ssid, sizeof(ssid)) != ESP_OK ||
        httpd_query_key_value(body, "clients", clients_text,
                              sizeof(clients_text)) != ESP_OK ||
        httpd_query_key_value(body, "tx_power", power_text,
                              sizeof(power_text)) != ESP_OK ||
        url_decode(ssid) != ESP_OK ||
        (password_supplied && url_decode(password) != ESP_OK)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid access-point settings");
    }

    int clients = atoi(clients_text);
    int tx_power = atoi(power_text);
    size_t ssid_length = strlen(ssid);
    size_t password_length = strlen(password);
    if (ssid_length == 0U || ssid_length >= sizeof(s_saved_ap_ssid) ||
        clients < 1 || clients > 10 ||
        (tx_power != 20 && tx_power != 40 &&
         tx_power != 60 && tx_power != 78) ||
        (!open_network && password_supplied && password_length > 0U &&
         password_length < 8U) ||
        password_length >= sizeof(s_saved_ap_password)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "Invalid access-point settings");
    }

    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open("drone_remote", NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_str(nvs, "ap_ssid", ssid);
    }
    if (err == ESP_OK && open_network) {
        err = nvs_set_str(nvs, "ap_password", "");
    } else if (err == ESP_OK && password_supplied && password_length > 0U) {
        err = nvs_set_str(nvs, "ap_password", password);
    } else if (err == ESP_OK) {
        err = nvs_set_str(nvs, "ap_password",
                          s_effective_config.ap_password);
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "ap_clients", (uint8_t)clients);
    }
    if (err == ESP_OK) {
        err = nvs_set_i8(nvs, "ap_tx_power", (int8_t)tx_power);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    if (nvs != 0) {
        nvs_close(nvs);
    }
    if (err != ESP_OK) {
        return httpd_resp_send_err(request,
                                   HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Could not save access-point settings");
    }

    httpd_resp_set_type(request, "application/json");
    err = httpd_resp_sendstr(request, "{\"saved\":true}");
    if (err == ESP_OK &&
        xTaskCreate(delayed_restart_task, "ap-restart", 2048, NULL, 5,
                    NULL) != pdPASS) {
        ESP_LOGE(TAG, "Could not create restart task");
    }
    return err;
}

/**
 * @brief Override compile-time network settings with values stored in NVS.
 *
 * Missing or invalid saved values are ignored, leaving the supplied runtime
 * configuration unchanged.
 *
 * @param config Runtime configuration supplied by the application.
 * @param effective Destination configuration used for Wi-Fi startup.
 */
static void apply_saved_network_config(
    const esp32_wifi_drone_remote_config_t *config,
    esp32_wifi_drone_remote_config_t *effective)
{
    *effective = *config;

    bool station_loaded = false;
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open_from_partition(
        WIFI_CREDENTIALS_PARTITION, WIFI_NVS_NAMESPACE, NVS_READONLY, &nvs);

    size_t ssid_size = sizeof(s_saved_station_ssid);
    size_t password_size = sizeof(s_saved_station_password);
    if (err == ESP_OK) {
        err = nvs_get_str(nvs, "sta_ssid", s_saved_station_ssid, &ssid_size);
    }
    if (err == ESP_OK) {
        err = nvs_get_str(nvs, "sta_password", s_saved_station_password,
                          &password_size);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            s_saved_station_password[0] = '\0';
            err = ESP_OK;
        }
    }
    if (err == ESP_OK && s_saved_station_ssid[0] != '\0') {
        effective->station_ssid = s_saved_station_ssid;
        effective->station_password = s_saved_station_password;
        station_loaded = true;
        ESP_LOGI(TAG, "Using station credentials saved from the setup page");
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Ignoring invalid saved station credentials: %s",
                 esp_err_to_name(err));
    }
    if (nvs != 0) {
        nvs_close(nvs);
        nvs = 0;
    }

    if (!station_loaded &&
        nvs_open(WIFI_NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        ssid_size = sizeof(s_saved_station_ssid);
        password_size = sizeof(s_saved_station_password);
        err = nvs_get_str(nvs, "sta_ssid", s_saved_station_ssid, &ssid_size);
        if (err == ESP_OK) {
            err = nvs_get_str(nvs, "sta_password",
                              s_saved_station_password, &password_size);
            if (err == ESP_ERR_NVS_NOT_FOUND) {
                s_saved_station_password[0] = '\0';
                err = ESP_OK;
            }
        }
        nvs_close(nvs);
        nvs = 0;
        if (err == ESP_OK && s_saved_station_ssid[0] != '\0') {
            effective->station_ssid = s_saved_station_ssid;
            effective->station_password = s_saved_station_password;
            station_loaded = true;
            err = save_station_credentials(s_saved_station_ssid,
                                           s_saved_station_password);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Migrated station credentials to dedicated NVS");
            } else {
                ESP_LOGW(TAG, "Could not migrate station credentials: %s",
                         esp_err_to_name(err));
            }
        }
    }

    err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return;
    }
    size_t ap_ssid_size = sizeof(s_saved_ap_ssid);
    size_t ap_password_size = sizeof(s_saved_ap_password);
    if (nvs_get_str(nvs, "ap_ssid", s_saved_ap_ssid,
                    &ap_ssid_size) == ESP_OK &&
        nvs_get_str(nvs, "ap_password", s_saved_ap_password,
                    &ap_password_size) == ESP_OK) {
        effective->ap_ssid = s_saved_ap_ssid;
        effective->ap_password = s_saved_ap_password;
    }
    uint8_t clients;
    int8_t tx_power;
    if (nvs_get_u8(nvs, "ap_clients", &clients) == ESP_OK) {
        effective->ap_max_connections = clients;
    }
    if (nvs_get_i8(nvs, "ap_tx_power", &tx_power) == ESP_OK) {
        effective->ap_tx_power_quarter_dbm = tx_power;
    }
    nvs_close(nvs);
}

/**
 * @brief Dispatch a controller request to the configured application callback.
 *
 * When no callback is configured, the request is logged and acknowledged by
 * the original placeholder behavior.
 *
 * @param request HTTP POST request for a controller endpoint.
 * @return ESP_OK after responding, or an HTTP-server receive/send error.
 */
static esp_err_t api_handler(httpd_req_t *request)
{
    uint8_t body[256];
    size_t received_total = 0U;

    if (request->content_len > sizeof(body)) {
        httpd_resp_set_status(request, "413 Content Too Large");
        return httpd_resp_sendstr(request, "Controller request is too large");
    }

    while (received_total < request->content_len) {
        int received = httpd_req_recv(
            request, (char *)body + received_total,
            request->content_len - received_total);
        if (received <= 0) {
            return ESP_FAIL;
        }
        received_total += (size_t)received;
    }

    if (s_api_handler != NULL) {
        esp_err_t err = s_api_handler(request->uri, body, received_total,
                                      s_api_context);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "API callback rejected %s: %s", request->uri,
                     esp_err_to_name(err));
            return httpd_resp_send_err(request,
                                       HTTPD_500_INTERNAL_SERVER_ERROR,
                                       esp_err_to_name(err));
        }
    } else {
        ESP_LOGI(TAG, "Placeholder request: %s", request->uri);
    }

    httpd_resp_set_status(request, "204 No Content");
    return httpd_resp_send(request, NULL, 0);
}

/**
 * @brief Start the HTTP server and register page and controller endpoints.
 *
 * @return ESP_OK on success, otherwise an ESP-IDF HTTP-server error.
 */
static esp_err_t start_webserver(void)
{
    static const char *api_paths[] = {
        "/api/settings", /* Compatibility with cached controller pages. */
        "/api/turn-lock", "/api/brightness", "/api/headlight",
        "/api/sound", "/api/left-stick", "/api/right-stick",
    };
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 32;

    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) {
        return err;
    }

    const httpd_uri_t page = {
        .uri = "/", .method = HTTP_GET, .handler = page_handler
    };
    err = httpd_register_uri_handler(s_server, &page);

    const httpd_uri_t wifi_page = {
        .uri = "/wifi",
        .method = HTTP_GET,
        .handler = wifi_setup_page_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &wifi_page);
    }

    const httpd_uri_t settings_page = {
        .uri = "/settings",
        .method = HTTP_GET,
        .handler = settings_page_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &settings_page);
    }

    const httpd_uri_t ap_settings_page = {
        .uri = "/settings/wifi-ap",
        .method = HTTP_GET,
        .handler = ap_settings_page_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &ap_settings_page);
    }

    const httpd_uri_t imu_settings_page = {
        .uri = "/settings/imu",
        .method = HTTP_GET,
        .handler = imu_settings_page_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &imu_settings_page);
    }

    const httpd_uri_t esc_settings_page = {
        .uri = "/settings/esc",
        .method = HTTP_GET,
        .handler = esc_settings_page_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &esc_settings_page);
    }

    const httpd_uri_t esc_pwm_channel_settings_page = {
        .uri = "/settings/esc/pwm-channels",
        .method = HTTP_GET,
        .handler = esc_pwm_channel_settings_page_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(
            s_server, &esc_pwm_channel_settings_page);
    }

    const httpd_uri_t esc_manual_page = {
        .uri = "/settings/esc/manual",
        .method = HTTP_GET,
        .handler = esc_manual_page_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &esc_manual_page);
    }

    const httpd_uri_t esc_programming_page = {
        .uri = "/settings/esc/programming",
        .method = HTTP_GET,
        .handler = esc_programming_page_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &esc_programming_page);
    }

    const httpd_uri_t wifi_config = {
        .uri = "/api/wifi-config",
        .method = HTTP_POST,
        .handler = wifi_config_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &wifi_config);
    }

    const httpd_uri_t wifi_scan = {
        .uri = "/api/wifi-scan",
        .method = HTTP_GET,
        .handler = wifi_scan_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &wifi_scan);
    }

    const httpd_uri_t status = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = status_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &status);
    }

    const httpd_uri_t latency = {
        .uri = "/api/latency",
        .method = HTTP_POST,
        .handler = latency_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &latency);
    }

    const httpd_uri_t telemetry = {
        .uri = "/api/telemetry",
        .method = HTTP_GET,
        .handler = telemetry_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &telemetry);
    }

    const httpd_uri_t imu_config_get = {
        .uri = "/api/imu-config",
        .method = HTTP_GET,
        .handler = imu_config_get_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &imu_config_get);
    }

    const httpd_uri_t imu_config_post = {
        .uri = "/api/imu-config",
        .method = HTTP_POST,
        .handler = imu_config_post_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &imu_config_post);
    }

    const httpd_uri_t esc_config_get = {
        .uri = "/api/esc-config",
        .method = HTTP_GET,
        .handler = esc_config_get_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &esc_config_get);
    }

    const httpd_uri_t esc_config_post = {
        .uri = "/api/esc-config",
        .method = HTTP_POST,
        .handler = esc_config_post_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &esc_config_post);
    }

    const httpd_uri_t esc_throttle_post = {
        .uri = "/api/esc-throttle",
        .method = HTTP_POST,
        .handler = esc_throttle_post_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &esc_throttle_post);
    }

    const httpd_uri_t esc_programming_post = {
        .uri = "/api/esc-programming",
        .method = HTTP_POST,
        .handler = esc_programming_post_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &esc_programming_post);
    }

    const httpd_uri_t ap_config_get = {
        .uri = "/api/ap-config",
        .method = HTTP_GET,
        .handler = ap_config_get_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &ap_config_get);
    }

    const httpd_uri_t ap_config_post = {
        .uri = "/api/ap-config",
        .method = HTTP_POST,
        .handler = ap_config_post_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &ap_config_post);
    }

    for (size_t i = 0; err == ESP_OK &&
         i < sizeof(api_paths) / sizeof(api_paths[0]); ++i) {
        const httpd_uri_t api = {
            .uri = api_paths[i], .method = HTTP_POST, .handler = api_handler
        };
        err = httpd_register_uri_handler(s_server, &api);
    }

    if (err != ESP_OK) {
        httpd_stop(s_server);
        s_server = NULL;
    }
    return err;
}

/**
 * @brief Initialize NVS, the network stack, the event loop, and Wi-Fi.
 *
 * @return ESP_OK on success, otherwise the first ESP-IDF initialization error.
 */
static esp_err_t initialize_platform(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_flash_init_partition(WIFI_CREDENTIALS_PARTITION);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Persistent Wi-Fi credential storage unavailable: %s",
                 esp_err_to_name(err));
        return err;
    }

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    return esp_wifi_init(&wifi_config);
}

/**
 * @brief Connect to the configured existing Wi-Fi network.
 *
 * @param config Component configuration containing station credentials and
 *               the connection timeout.
 * @return ESP_OK after receiving an IP address, ESP_ERR_TIMEOUT when the
 *         connection fails or times out, or another ESP-IDF error.
 */
static esp_err_t start_station(const esp32_wifi_drone_remote_config_t *config)
{
    if (strlen(config->station_ssid) >= sizeof(((wifi_config_t *)0)->sta.ssid) ||
        strlen(config->station_password) >=
            sizeof(((wifi_config_t *)0)->sta.password)) {
        return ESP_ERR_INVALID_ARG;
    }

    s_netif = esp_netif_create_default_wifi_sta();
    if (s_netif == NULL) {
        return ESP_ERR_NO_MEM;
    }
    ESP_RETURN_ON_ERROR(esp_netif_set_hostname(s_netif, NETWORK_HOSTNAME), TAG,
                        "Could not set station hostname");

    wifi_config_t wifi_config = { 0 };
    snprintf((char *)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), "%s",
             config->station_ssid);
    snprintf((char *)wifi_config.sta.password,
             sizeof(wifi_config.sta.password), "%s",
             config->station_password);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG,
                        "Could not select station mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG,
                        "Could not configure station");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Could not start station");
    s_wifi_mode = WIFI_MODE_STA;

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_events, WIFI_CONNECTED_BIT | WIFI_FAILED_BIT, pdTRUE, pdFALSE,
        pdMS_TO_TICKS(config->station_timeout_ms));
    if ((bits & WIFI_CONNECTED_BIT) != 0U) {
        ESP_LOGI(TAG, "Connected to Wi-Fi network \"%s\"",
                 config->station_ssid);
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Start the ESP32-hosted Wi-Fi access point.
 *
 * @param config Component configuration containing access-point settings.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for invalid credentials, or
 *         another ESP-IDF Wi-Fi error.
 */
static esp_err_t start_access_point(
    const esp32_wifi_drone_remote_config_t *config)
{
    size_t ssid_length = strlen(config->ap_ssid);
    size_t password_length = strlen(config->ap_password);
    if (ssid_length == 0U ||
        ssid_length >= sizeof(((wifi_config_t *)0)->ap.ssid) ||
        password_length >= sizeof(((wifi_config_t *)0)->ap.password) ||
        (password_length > 0U && password_length < 8U)) {
        return ESP_ERR_INVALID_ARG;
    }

    s_netif = esp_netif_create_default_wifi_ap();
    if (s_netif == NULL) {
        return ESP_ERR_NO_MEM;
    }
    ESP_RETURN_ON_ERROR(esp_netif_set_hostname(s_netif, NETWORK_HOSTNAME), TAG,
                        "Could not set access-point hostname");

    wifi_config_t wifi_config = { 0 };
    snprintf((char *)wifi_config.ap.ssid, sizeof(wifi_config.ap.ssid), "%s",
             config->ap_ssid);
    snprintf((char *)wifi_config.ap.password,
             sizeof(wifi_config.ap.password), "%s", config->ap_password);
    wifi_config.ap.ssid_len = ssid_length;
    wifi_config.ap.max_connection = config->ap_max_connections;
    wifi_config.ap.authmode =
        password_length == 0U ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG,
                        "Could not select access-point mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &wifi_config), TAG,
                        "Could not configure access point");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Could not start access point");
    ESP_RETURN_ON_ERROR(
        esp_wifi_set_max_tx_power(config->ap_tx_power_quarter_dbm), TAG,
        "Could not set access-point transmit power");
    s_wifi_mode = WIFI_MODE_AP;
    ESP_LOGI(TAG, "Access point \"%s\" ready", config->ap_ssid);
    return ESP_OK;
}

esp_err_t esp32_wifi_drone_remote_start(
    const esp32_wifi_drone_remote_config_t *config)
{
    if (s_started) {
        return ESP_ERR_INVALID_STATE;
    }
    if (config == NULL || config->ap_ssid == NULL ||
        config->ap_password == NULL || config->ap_max_connections == 0U ||
        config->ap_tx_power_quarter_dbm < 8 ||
        config->ap_tx_power_quarter_dbm > 78 ||
        config->station_ssid == NULL || config->station_password == NULL ||
        config->station_timeout_ms == 0U ||
        config->latency_timeout_ms == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = initialize_platform();
    if (err != ESP_OK) {
        return err;
    }

    esp32_wifi_drone_remote_config_t effective_config;
    apply_saved_network_config(config, &effective_config);
    s_effective_config = effective_config;
    s_api_handler = effective_config.api_handler;
    s_api_context = effective_config.api_context;
    s_latency_handler = effective_config.latency_handler;
    s_latency_context = effective_config.latency_context;
    s_latency_timeout_ms = effective_config.latency_timeout_ms;
    s_telemetry_handler = effective_config.telemetry_handler;
    s_telemetry_context = effective_config.telemetry_context;
    s_imu_get_handler = effective_config.imu_get_handler;
    s_imu_set_handler = effective_config.imu_set_handler;
    s_imu_context = effective_config.imu_context;
    s_esc_get_handler = effective_config.esc_get_handler;
    s_esc_set_handler = effective_config.esc_set_handler;
    s_esc_throttle_handler = effective_config.esc_throttle_handler;
    s_esc_program_handler = effective_config.esc_program_handler;
    s_esc_context = effective_config.esc_context;

    s_wifi_events = xEventGroupCreate();
    if (s_wifi_events == NULL) {
        esp_wifi_deinit();
        return ESP_ERR_NO_MEM;
    }
    err = esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL,
        &s_wifi_handler);
    if (err != ESP_OK) {
        esp32_wifi_drone_remote_stop();
        return err;
    }
    err = esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL,
        &s_ip_handler);
    if (err != ESP_OK) {
        esp32_wifi_drone_remote_stop();
        return err;
    }

    if (effective_config.station_ssid[0] != '\0') {
        err = start_station(&effective_config);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Station connection failed; starting access point");
            esp_wifi_stop();
            esp_netif_destroy_default_wifi(s_netif);
            s_netif = NULL;
            err = start_access_point(&effective_config);
        }
    } else {
        err = start_access_point(&effective_config);
    }

    if (err == ESP_OK) {
        err = start_mdns();
    }
    if (err == ESP_OK) {
        err = start_webserver();
    }
    if (err != ESP_OK) {
        esp32_wifi_drone_remote_stop();
        return err;
    }

    s_started = true;
    return ESP_OK;
}

esp_err_t esp32_wifi_drone_remote_stop(void)
{
    if (s_server != NULL) {
        httpd_stop(s_server);
        s_server = NULL;
    }
    if (s_mdns_started) {
        mdns_free();
        s_mdns_started = false;
    }
    esp_wifi_stop();
    if (s_netif != NULL) {
        esp_netif_destroy_default_wifi(s_netif);
        s_netif = NULL;
    }
    if (s_wifi_handler != NULL) {
        esp_event_handler_instance_unregister(
            WIFI_EVENT, ESP_EVENT_ANY_ID, s_wifi_handler);
        s_wifi_handler = NULL;
    }
    if (s_ip_handler != NULL) {
        esp_event_handler_instance_unregister(
            IP_EVENT, IP_EVENT_STA_GOT_IP, s_ip_handler);
        s_ip_handler = NULL;
    }
    esp_wifi_deinit();
    if (s_wifi_events != NULL) {
        vEventGroupDelete(s_wifi_events);
        s_wifi_events = NULL;
    }
    s_started = false;
    s_api_handler = NULL;
    s_api_context = NULL;
    s_latency_handler = NULL;
    s_latency_context = NULL;
    s_latency_timeout_ms = 150U;
    s_telemetry_handler = NULL;
    s_telemetry_context = NULL;
    s_imu_get_handler = NULL;
    s_imu_set_handler = NULL;
    s_imu_context = NULL;
    s_esc_get_handler = NULL;
    s_esc_set_handler = NULL;
    s_esc_throttle_handler = NULL;
    s_esc_program_handler = NULL;
    s_esc_context = NULL;
    s_wifi_mode = WIFI_MODE_NULL;
    return ESP_OK;
}
