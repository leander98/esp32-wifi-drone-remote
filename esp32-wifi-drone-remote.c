#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <esp_check.h>
#include <esp_event.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <esp_wifi_default.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <nvs_flash.h>

#include "esp32-wifi-drone-remote.h"

static const char *TAG = "wifi-drone-remote";
static const EventBits_t WIFI_CONNECTED_BIT = BIT0;
static const EventBits_t WIFI_FAILED_BIT = BIT1;

static EventGroupHandle_t s_wifi_events;
static esp_event_handler_instance_t s_wifi_handler;
static esp_event_handler_instance_t s_ip_handler;
static esp_netif_t *s_netif;
static httpd_handle_t s_server;
static bool s_started;

extern const uint8_t controller_html_start[]
    asm("_binary_controller_html_start");

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

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
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
    return httpd_resp_send(request, (const char *)controller_html_start,
                           HTTPD_RESP_USE_STRLEN);
}

/**
 * @brief Consume a controller request without performing an action.
 *
 * Every controller endpoint currently uses this placeholder. Request bodies
 * are drained so the persistent HTTP connection remains usable.
 *
 * @param request HTTP POST request for a controller endpoint.
 * @return ESP_OK after sending HTTP 204, or ESP_FAIL on a receive error.
 */
static esp_err_t api_handler(httpd_req_t *request)
{
    char buffer[128];
    size_t remaining = request->content_len;

    while (remaining > 0U) {
        size_t chunk = remaining < sizeof(buffer) ? remaining : sizeof(buffer);
        int received = httpd_req_recv(request, buffer, chunk);
        if (received <= 0) {
            return ESP_FAIL;
        }
        remaining -= (size_t)received;
    }

    ESP_LOGI(TAG, "Placeholder request: %s", request->uri);
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
        "/api/connect", "/api/settings", "/api/turn-lock", "/api/brightness",
        "/api/headlight", "/api/sound", "/api/left-stick", "/api/right-stick",
    };
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 12;

    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) {
        return err;
    }

    const httpd_uri_t page = {
        .uri = "/", .method = HTTP_GET, .handler = page_handler
    };
    err = httpd_register_uri_handler(s_server, &page);

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
    ESP_LOGI(TAG, "Access point \"%s\" ready at http://192.168.4.1",
             config->ap_ssid);
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
        config->station_ssid == NULL || config->station_password == NULL ||
        config->station_timeout_ms == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = initialize_platform();
    if (err != ESP_OK) {
        return err;
    }

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

    if (config->station_ssid[0] != '\0') {
        err = start_station(config);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Station connection failed; starting access point");
            esp_wifi_stop();
            esp_netif_destroy_default_wifi(s_netif);
            s_netif = NULL;
            err = start_access_point(config);
        }
    } else {
        err = start_access_point(config);
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
    return ESP_OK;
}
