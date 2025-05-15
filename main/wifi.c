#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_wps.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "nvs.h" /* Added for NVS functions */
#include <string.h>


/*set wps mode via project configuration */
#define WPS_MODE WPS_TYPE_PBC

#define MAX_RETRY_ATTEMPTS     2

#ifndef PIN2STR
#define PIN2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5], (a)[6], (a)[7]
#define PINSTR "%c%c%c%c%c%c%c%c"
#endif

/* NVS definitions for Wi-Fi credentials */
#define NVS_NAMESPACE "wifi_creds"
#define NVS_KEY_SSID "ssid"
#define NVS_KEY_PASSWORD "password"

static const char *TAG = "example_wps";
static esp_wps_config_t config = WPS_CONFIG_INIT_DEFAULT(WPS_MODE);
static wifi_config_t wps_ap_creds[MAX_WPS_AP_CRED]; /* MAX_WPS_AP_CRED is usually 1 from sdkconfig */
static int s_ap_creds_num = 0;
static int s_retry_num = 0;
static bool s_tried_nvs_creds = false; /* Flag to track if we tried connecting with NVS credentials */

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    static int ap_idx = 1; /* Start from 1 for subsequent APs if wps_ap_creds[0] fails */

    switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WIFI_EVENT_STA_START");
            /* esp_wifi_connect() will be called either from wifi_start (NVS) or WPS logic */
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED");
            if (s_tried_nvs_creds) {
                ESP_LOGI(TAG, "Connection with saved NVS credentials failed.");
                s_tried_nvs_creds = false; /* Reset flag */
                s_retry_num = 0;           /* Reset retry for WPS */
                ap_idx = 1;                /* Reset AP index for WPS logic */
                s_ap_creds_num = 0;        /* Clear any old WPS creds count */

                /* Optionally, clear the failed NVS credentials */
                /*
                nvs_handle_t my_handle;
                esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
                if (err == ESP_OK) {
                    nvs_erase_key(my_handle, NVS_KEY_SSID);
                    nvs_erase_key(my_handle, NVS_KEY_PASSWORD);
                    nvs_commit(my_handle);
                    nvs_close(my_handle);
                    ESP_LOGI(TAG, "Cleared saved credentials from NVS.");
                }
                */

                ESP_LOGI(TAG, "Starting WPS to get new credentials...");
                ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_wps_disable()); /* Ensure WPS is reset */
                ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_wps_enable(&config));
                ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_wps_start(0));
            } else {
                /* This is a disconnect during WPS or subsequent retries with WPS-obtained credentials */
                ESP_LOGI(TAG, "Disconnected during WPS operation or retries.");
                if (s_retry_num < MAX_RETRY_ATTEMPTS) {
                    esp_wifi_connect(); /* Retry current creds */
                    s_retry_num++;
                    ESP_LOGI(TAG, "Retrying connection with current credentials, attempt %d/%d", s_retry_num, MAX_RETRY_ATTEMPTS);
                } else if (s_ap_creds_num > 0 && ap_idx < s_ap_creds_num) {
                    ESP_LOGI(TAG, "Trying next WPS AP credential (index %d of %d): SSID: %s",
                             ap_idx, s_ap_creds_num, (char*)wps_ap_creds[ap_idx].sta.ssid);
                    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_config(WIFI_IF_STA, &wps_ap_creds[ap_idx++]));
                    esp_wifi_connect();
                    s_retry_num = 0; /* Reset retry count for the new AP */
                } else {
                    ESP_LOGI(TAG, "All connection attempts for WPS credentials failed.");
                    /* WPS module might re-trigger WPS on ER_FAILED or ER_TIMEOUT events */
                }
            }
            break;
        case WIFI_EVENT_STA_WPS_ER_SUCCESS:
            ESP_LOGI(TAG, "WIFI_EVENT_STA_WPS_ER_SUCCESS");
            {
                wifi_event_sta_wps_er_success_t *evt =
                    (wifi_event_sta_wps_er_success_t *)event_data;
                int i;

                if (evt) {
                    s_ap_creds_num = evt->ap_cred_cnt;
                    for (i = 0; i < s_ap_creds_num; i++) {
                        memcpy(wps_ap_creds[i].sta.ssid, evt->ap_cred[i].ssid,
                               sizeof(evt->ap_cred[i].ssid));
                        memcpy(wps_ap_creds[i].sta.password, evt->ap_cred[i].passphrase,
                               sizeof(evt->ap_cred[i].passphrase));
                    }
                    /* If multiple AP credentials are received from WPS, connect with first one */
                    ESP_LOGI(TAG, "Connecting to SSID: %s, Passphrase: %s",
                             wps_ap_creds[0].sta.ssid, wps_ap_creds[0].sta.password);
                    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_config(WIFI_IF_STA, &wps_ap_creds[0]) );
                }
                /*
                 * If only one AP credential is received from WPS, there will be no event data and
                 * esp_wifi_set_config() is already called by WPS modules for backward compatibility
                 * with legacy apps. So directly attempt connection here.
                 */
                ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_wps_disable());
                esp_wifi_connect();
            }
            break;
        case WIFI_EVENT_STA_WPS_ER_FAILED:
            ESP_LOGI(TAG, "WIFI_EVENT_STA_WPS_ER_FAILED");
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_wps_disable());
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_wps_enable(&config));
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_wps_start(0));
            break;
        case WIFI_EVENT_STA_WPS_ER_TIMEOUT:
            ESP_LOGI(TAG, "WIFI_EVENT_STA_WPS_ER_TIMEOUT");
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_wps_disable());
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_wps_enable(&config));
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_wps_start(0));
            break;
        case WIFI_EVENT_STA_WPS_ER_PIN:
            ESP_LOGI(TAG, "WIFI_EVENT_STA_WPS_ER_PIN");
            /* display the PIN code */
            wifi_event_sta_wps_er_pin_t* event_pin = (wifi_event_sta_wps_er_pin_t*) event_data; /* Renamed to avoid conflict */
            ESP_LOGI(TAG, "WPS_PIN = " PINSTR, PIN2STR(event_pin->pin_code));
            break;
        default:
            break;
    }
}

static void got_ip_event_handler(void* arg, esp_event_base_t event_base,
                             int32_t event_id, void* event_data)
{
    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
    ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));

    /* Save the successful credentials to NVS */
    wifi_config_t current_config;
    esp_err_t err = esp_wifi_get_config(ESP_IF_WIFI_STA, &current_config);
    if (err == ESP_OK) {
        if (strlen((const char*)current_config.sta.ssid) > 0) { /* Ensure SSID is not empty */
            ESP_LOGI(TAG, "Successfully connected to SSID: %s/%s. Saving credentials to NVS.", (char*)current_config.sta.ssid, (char*)current_config.sta.password);
            nvs_handle_t my_handle;
            err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
            if (err == ESP_OK) {
                esp_err_t err_ssid = nvs_set_str(my_handle, NVS_KEY_SSID, (const char*)current_config.sta.ssid);
                esp_err_t err_pass = nvs_set_str(my_handle, NVS_KEY_PASSWORD, (const char*)current_config.sta.password);

                if (err_ssid == ESP_OK && err_pass == ESP_OK) {
                    err = nvs_commit(my_handle);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "Credentials saved to NVS successfully.");
                    } else {
                        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "NVS set_str for SSID or Password failed. SSID_err: %s, Pass_err: %s",
                             esp_err_to_name(err_ssid), esp_err_to_name(err_pass));
                }
                nvs_close(my_handle);
            } else {
                ESP_LOGE(TAG, "Error opening NVS to save credentials: %s", esp_err_to_name(err));
            }
        } else {
            ESP_LOGW(TAG, "Connected, but SSID is empty. Not saving credentials.");
        }
    } else {
        ESP_LOGE(TAG, "Error getting current Wi-Fi config to save: %s", esp_err_to_name(err));
    }
    s_tried_nvs_creds = false; /* Reset flag, as we are successfully connected (either via NVS or new WPS) */
}

/*init wifi as sta and start wps*/
void wifi_start(void)
{
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &got_ip_event_handler, NULL));

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_start());

    /* Attempt to load and connect with saved credentials */
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        wifi_config_t saved_config = {0};
        size_t ssid_len = sizeof(saved_config.sta.ssid);
        size_t pass_len = sizeof(saved_config.sta.password);

        err = nvs_get_str(my_handle, NVS_KEY_SSID, (char *)saved_config.sta.ssid, &ssid_len);
        if (err == ESP_OK && ssid_len > 1) { /* ssid_len includes null terminator */
            err = nvs_get_str(my_handle, NVS_KEY_PASSWORD, (char *)saved_config.sta.password, &pass_len);
            if (err == ESP_OK) { /* Password can be empty for open networks */
                ESP_LOGI(TAG, "Found saved credentials for SSID: %s. Attempting to connect.", saved_config.sta.ssid);
                ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_config(WIFI_IF_STA, &saved_config));
                s_tried_nvs_creds = true;
                s_retry_num = 0; /* Reset retry count for NVS attempt */
                esp_wifi_connect();
                nvs_close(my_handle);
                return; /* Exit wifi_start, connection attempt with NVS creds is in progress */
            } else {
                ESP_LOGI(TAG, "Failed to read saved password (err: %s). Proceeding to WPS.", esp_err_to_name(err));
            }
        } else {
            ESP_LOGI(TAG, "Failed to read saved SSID (err: %s, len: %d). Proceeding to WPS.", esp_err_to_name(err), ssid_len);
        }
        nvs_close(my_handle); /* Close NVS if opened but creds not fully read/used */
    } else {
        ESP_LOGI(TAG, "NVS open failed or no '%s' namespace (err: %s). Proceeding to WPS.", NVS_NAMESPACE, esp_err_to_name(err));
    }

    /* If NVS load/connect wasn't initiated, or failed, start WPS */
    ESP_LOGI(TAG, "Starting WPS...");
    s_tried_nvs_creds = false; /* Ensure flag is false if WPS is started */
    /* The original vTaskDelay(pdMS_TO_TICKS(500)) was here. If needed for WPS stability, keep it. */
    /* For now, assuming it's not strictly necessary before wps_enable. */
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_wps_enable(&config));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_wps_start(0));
}
