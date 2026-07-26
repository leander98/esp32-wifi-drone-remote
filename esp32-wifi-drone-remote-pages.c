/**
 * @file esp32-wifi-drone-remote-pages.c
 * @brief HTTP handlers for pages embedded in the Wi-Fi remote firmware.
 */
#include "esp32-wifi-drone-remote-internal.h"

#include <stdint.h>

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
extern const uint8_t pid_settings_html_start[]
    asm("_binary_pid_settings_html_start");
extern const uint8_t esc_settings_html_start[]
    asm("_binary_esc_settings_html_start");
extern const uint8_t esc_pwm_channel_settings_html_start[]
    asm("_binary_esc_pwm_channel_settings_html_start");
extern const uint8_t esc_manual_html_start[]
    asm("_binary_esc_manual_html_start");
extern const uint8_t esc_programming_html_start[]
    asm("_binary_esc_programming_html_start");
extern const uint8_t esc_throttle_range_html_start[]
    asm("_binary_esc_throttle_range_html_start");

/** Cache policy used by every embedded application page. */
static const char *PAGE_CACHE_CONTROL =
    "no-store, no-cache, must-revalidate";

/**
 * @brief Send one linker-embedded HTML document.
 *
 * @param[in] request Active HTTP request.
 * @param[in] document NUL-terminated embedded document.
 * @return ESP_OK after sending the page, otherwise an HTTP-server error.
 */
static esp_err_t send_html(httpd_req_t *request, const uint8_t *document)
{
    httpd_resp_set_type(request, "text/html");
    httpd_resp_set_hdr(request, "Cache-Control", PAGE_CACHE_CONTROL);
    return httpd_resp_send(request, (const char *)document,
                           HTTPD_RESP_USE_STRLEN);
}

/** @brief Serve the embedded controller page. */
esp_err_t page_handler(httpd_req_t *request)
{
    return send_html(request, controller_html_start);
}

/** @brief Serve the embedded station configuration page. */
esp_err_t wifi_setup_page_handler(httpd_req_t *request)
{
    return send_html(request, wifi_setup_html_start);
}

/** @brief Serve the embedded settings menu. */
esp_err_t settings_page_handler(httpd_req_t *request)
{
    return send_html(request, settings_html_start);
}

/** @brief Serve the embedded Wi-Fi access-point settings page. */
esp_err_t ap_settings_page_handler(httpd_req_t *request)
{
    return send_html(request, wifi_ap_settings_html_start);
}

/** @brief Serve the embedded ISM330DLC settings page. */
esp_err_t imu_settings_page_handler(httpd_req_t *request)
{
    return send_html(request, imu_settings_html_start);
}

/** @brief Serve the flight-controller PID tuning page. */
esp_err_t pid_settings_page_handler(httpd_req_t *request)
{
    return send_html(request, pid_settings_html_start);
}

/** @brief Serve the XW30A settings menu. */
esp_err_t esc_settings_page_handler(httpd_req_t *request)
{
    return send_html(request, esc_settings_html_start);
}

/** @brief Serve the XW30A PWM-channel settings page. */
esp_err_t esc_pwm_channel_settings_page_handler(httpd_req_t *request)
{
    return send_html(request, esc_pwm_channel_settings_html_start);
}

/** @brief Serve the manual XW30A throttle-control page. */
esp_err_t esc_manual_page_handler(httpd_req_t *request)
{
    return send_html(request, esc_manual_html_start);
}

/** @brief Serve the guided XW30A programming page. */
esp_err_t esc_programming_page_handler(httpd_req_t *request)
{
    return send_html(request, esc_programming_html_start);
}

/** @brief Serve the guided XW30A throttle-range setting page. */
esp_err_t esc_throttle_range_page_handler(httpd_req_t *request)
{
    return send_html(request, esc_throttle_range_html_start);
}
