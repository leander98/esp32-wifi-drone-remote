/**
 * @file esp32-wifi-drone-remote-storage.c
 * @brief Persistent network configuration and legacy migration.
 */
#include "esp32-wifi-drone-remote-internal.h"

#include <esp_log.h>
#include <nvs.h>

static const char *TAG = "wifi-remote-storage";
static const char *WIFI_CREDENTIALS_PARTITION = "wifi_creds";
static const char *WIFI_NVS_NAMESPACE = "drone_remote";

static char s_saved_station_ssid[WIFI_REMOTE_SSID_SIZE];
static char s_saved_station_password[WIFI_REMOTE_PASSWORD_SIZE];
static char s_saved_ap_ssid[WIFI_REMOTE_SSID_SIZE];
static char s_saved_ap_password[WIFI_REMOTE_PASSWORD_SIZE];

/** @brief Persist station credentials in their dedicated NVS partition. */
esp_err_t save_station_credentials(const char *ssid, const char *password)
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
 * @brief Override compile-time network settings with values stored in NVS.
 *
 * @details Station credentials are read from their dedicated partition.
 * Values written by older firmware are migrated from the default partition.
 */
void apply_saved_network_config(
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
    uint32_t control_timeout_ms;
    uint16_t timeout_throttle_permille;
    if (nvs_get_u8(nvs, "ap_clients", &clients) == ESP_OK) {
        effective->ap_max_connections = clients;
    }
    if (nvs_get_i8(nvs, "ap_tx_power", &tx_power) == ESP_OK) {
        effective->ap_tx_power_quarter_dbm = tx_power;
    }
    if (nvs_get_u32(nvs, "ctrl_timeout", &control_timeout_ms) == ESP_OK) {
        effective->control_timeout_ms = control_timeout_ms;
    }
    if (nvs_get_u16(nvs, "timeout_thr", &timeout_throttle_permille) ==
        ESP_OK && timeout_throttle_permille <= 1000U) {
        effective->timeout_throttle =
            (float)timeout_throttle_permille / 1000.0f;
    }
    nvs_close(nvs);
}
