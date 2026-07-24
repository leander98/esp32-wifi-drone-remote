#include <stdbool.h>
#include <stdio.h>
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

static EventGroupHandle_t s_wifi_events;
static esp_event_handler_instance_t s_wifi_handler;
static esp_event_handler_instance_t s_ip_handler;
static esp_netif_t *s_netif;
static httpd_handle_t s_server;
static bool s_started;
static bool s_mdns_started;
static wifi_mode_t s_wifi_mode = WIFI_MODE_NULL;
static esp32_wifi_drone_remote_api_handler_t s_api_handler;
static void *s_api_context;
static char s_saved_station_ssid[33];
static char s_saved_station_password[65];

extern const uint8_t controller_html_start[]
    asm("_binary_controller_html_start");
extern const uint8_t wifi_setup_html_start[]
    asm("_binary_wifi_setup_html_start");

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
 * @brief Serve the embedded station configuration page.
 *
 * @param request HTTP GET request for `/wifi`.
 * @return ESP_OK when the response is sent, otherwise an HTTP-server error.
 */
static esp_err_t wifi_setup_page_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html");
    return httpd_resp_send(request, (const char *)wifi_setup_html_start,
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

    char response[48];
    int length = snprintf(response, sizeof(response),
                          "{\"mode\":\"%s\",\"rssi\":%d}", mode, rssi);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, response, length);
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

    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open("drone_remote", NVS_READWRITE, &nvs);
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
 * @brief Override compile-time station credentials with values stored in NVS.
 *
 * Missing or invalid saved values are ignored, leaving the supplied runtime
 * configuration unchanged.
 *
 * @param config Runtime configuration supplied by the application.
 * @param effective Destination configuration used for Wi-Fi startup.
 */
static void apply_saved_station_config(
    const esp32_wifi_drone_remote_config_t *config,
    esp32_wifi_drone_remote_config_t *effective)
{
    *effective = *config;

    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open("drone_remote", NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return;
    }

    size_t ssid_size = sizeof(s_saved_station_ssid);
    size_t password_size = sizeof(s_saved_station_password);
    err = nvs_get_str(nvs, "sta_ssid", s_saved_station_ssid, &ssid_size);
    if (err == ESP_OK) {
        err = nvs_get_str(nvs, "sta_password", s_saved_station_password,
                          &password_size);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            s_saved_station_password[0] = '\0';
            err = ESP_OK;
        }
    }
    nvs_close(nvs);

    if (err == ESP_OK && s_saved_station_ssid[0] != '\0') {
        effective->station_ssid = s_saved_station_ssid;
        effective->station_password = s_saved_station_password;
        ESP_LOGI(TAG, "Using station credentials saved from the setup page");
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Ignoring invalid saved station credentials: %s",
                 esp_err_to_name(err));
    }
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
        "/api/settings", "/api/turn-lock", "/api/brightness", "/api/headlight",
        "/api/sound", "/api/left-stick", "/api/right-stick",
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

    const httpd_uri_t wifi_page = {
        .uri = "/wifi",
        .method = HTTP_GET,
        .handler = wifi_setup_page_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &wifi_page);
    }

    const httpd_uri_t wifi_config = {
        .uri = "/api/wifi-config",
        .method = HTTP_POST,
        .handler = wifi_config_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &wifi_config);
    }

    const httpd_uri_t status = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = status_handler,
    };
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_server, &status);
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
        config->station_ssid == NULL || config->station_password == NULL ||
        config->station_timeout_ms == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = initialize_platform();
    if (err != ESP_OK) {
        return err;
    }

    esp32_wifi_drone_remote_config_t effective_config;
    apply_saved_station_config(config, &effective_config);
    s_api_handler = effective_config.api_handler;
    s_api_context = effective_config.api_context;

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
    s_wifi_mode = WIFI_MODE_NULL;
    return ESP_OK;
}
