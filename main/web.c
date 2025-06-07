/*
 * web.c
 *
 *  Created on: May 29, 2025
 *      Author: GitHub Copilot
 */

#include "web.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "tuner/tuner.h"
#include "voltage.h" /* Added for voltage_select and voltage_get_current */
#include "cJSON.h"
#include <fcntl.h> /* Added for open/read */

static const char *TAG = "web";

#define WEB_MOUNT_POINT "/storage"
#define INDEX_HTML_PATH WEB_MOUNT_POINT "/index.htm"

static httpd_handle_t server = NULL;

/* HTTP GET handler for the root URL (serves index.html from SPIFFS) */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Serving %s", INDEX_HTML_PATH);

    /* Check if file exists */
    struct stat file_stat;
    if (stat(INDEX_HTML_PATH, &file_stat) == -1)
    {
        ESP_LOGE(TAG, "File %s not found", INDEX_HTML_PATH);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    int fd = open(INDEX_HTML_PATH, O_RDONLY, 0);
    if (fd == -1)
    {
        ESP_LOGE(TAG, "Failed to open file: %s", INDEX_HTML_PATH);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/html");

    /* Send file in chunks */
    char buffer[256];
    ssize_t bytes_read;
    while ((bytes_read = read(fd, buffer, sizeof(buffer))) > 0)
    {
        if (httpd_resp_send_chunk(req, buffer, bytes_read) != ESP_OK)
        {
            close(fd);
            ESP_LOGE(TAG, "File sending failed!");
            /* Abort sending file */
            httpd_resp_send_chunk(req, NULL, 0);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
    }
    close(fd);

    /* Respond with an empty chunk to signal HTTP response completion */
    httpd_resp_send_chunk(req, NULL, 0);
    ESP_LOGI(TAG, "File %s sent successfully", INDEX_HTML_PATH);
    return ESP_OK;
}

/* HTTP GET handler for /api/tuner/status */
static esp_err_t tuner_status_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "GET /api/tuner/status");
    cJSON *root = cJSON_CreateObject();
    if (root == NULL)
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    tuner_status_t status_data;
    esp_err_t ret = tuner_get_status_data(&status_data);

    if (ret == ESP_OK)
    {
        cJSON_AddBoolToObject(root, "initialized", status_data.initialized);
        cJSON_AddNumberToObject(root, "status_flags", status_data.status_flags);
        cJSON_AddBoolToObject(root, "has_signal", (status_data.status_flags & FE_HAS_SIGNAL) != 0);
        cJSON_AddBoolToObject(root, "has_carrier", (status_data.status_flags & FE_HAS_CARRIER) != 0);
        cJSON_AddBoolToObject(root, "has_viterbi", (status_data.status_flags & FE_HAS_VITERBI) != 0);
        cJSON_AddBoolToObject(root, "has_sync", (status_data.status_flags & FE_HAS_SYNC) != 0);
        cJSON_AddBoolToObject(root, "has_lock", (status_data.status_flags & FE_HAS_LOCK) != 0);
        cJSON_AddNumberToObject(root, "snr", status_data.snr);
        cJSON_AddNumberToObject(root, "strength", status_data.strength);
        cJSON_AddNumberToObject(root, "ber", status_data.ber);
        cJSON_AddNumberToObject(root, "uc_blocks", status_data.uc_blocks);
    }
    else
    {
        cJSON_AddStringToObject(root, "error", "Tuner not initialized or error reading status.");
        cJSON_AddBoolToObject(root, "initialized", false); /* Indicate not initialized if error occurs */
    }

    const char *sys_info = cJSON_PrintUnformatted(root);
    if (sys_info == NULL)
    {
        ESP_LOGE(TAG, "Failed to print cJSON to string.");
        cJSON_Delete(root);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, sys_info, strlen(sys_info));

    free((void *)sys_info);
    cJSON_Delete(root);
    return ESP_OK;
}

/* HTTP POST handler for /api/tuner/tune */
static esp_err_t tuner_tune_post_handler(httpd_req_t *req)
{
    char buf[256]; /* Increased buffer size */
    int ret, remaining = req->content_len;
    cJSON *json_resp = cJSON_CreateObject();

    ESP_LOGI(TAG, "POST /api/tuner/tune, content_len: %d", remaining);

    if (remaining >= sizeof(buf))
    {
        ESP_LOGE(TAG, "Request content too long: %d", remaining);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Request content too long");
        cJSON_Delete(json_resp);
        return ESP_FAIL;
    }

    ret = httpd_req_recv(req, buf, remaining);
    if (ret <= 0)
    { /* Error or empty data */
        if (ret == HTTPD_SOCK_ERR_TIMEOUT)
        {
            httpd_resp_send_408(req);
        }
        cJSON_Delete(json_resp);
        return ESP_FAIL;
    }
    buf[ret] = '\0'; /* Null-terminate the buffer */
    ESP_LOGI(TAG, "Received tune data: %s", buf);

    cJSON *root = cJSON_Parse(buf);
    if (root == NULL)
    {
        ESP_LOGE(TAG, "Failed to parse JSON: %s", cJSON_GetErrorPtr());
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON format");
        cJSON_Delete(json_resp);
        return ESP_FAIL;
    }

    tuner_tune_params_t params;
    /* Safely get values from JSON */
    cJSON *item;
    item = cJSON_GetObjectItem(root, "freq_khz");
    if (!cJSON_IsNumber(item))
    {
        ESP_LOGE(TAG, "Missing or invalid freq_khz");
        goto json_error;
    }
    params.freq_khz = item->valueint;

    item = cJSON_GetObjectItem(root, "sr_ksps");
    if (!cJSON_IsNumber(item))
    {
        ESP_LOGE(TAG, "Missing or invalid sr_ksps");
        goto json_error;
    }
    params.sr_ksps = item->valueint;

    item = cJSON_GetObjectItem(root, "fec");
    if (!cJSON_IsNumber(item))
    {
        ESP_LOGE(TAG, "Missing or invalid fec");
        goto json_error;
    }
    params.fec = item->valueint;

    item = cJSON_GetObjectItem(root, "inv");
    if (!cJSON_IsNumber(item))
    {
        ESP_LOGE(TAG, "Missing or invalid inv");
        goto json_error;
    }
    params.inv = item->valueint;

    item = cJSON_GetObjectItem(root, "delsys");
    if (!cJSON_IsNumber(item))
    {
        ESP_LOGE(TAG, "Missing or invalid delsys");
        goto json_error;
    }
    params.delsys = item->valueint;

    item = cJSON_GetObjectItem(root, "mod");
    if (!cJSON_IsNumber(item))
    {
        ESP_LOGE(TAG, "Missing or invalid mod");
        goto json_error;
    }
    params.mod = item->valueint;

    item = cJSON_GetObjectItem(root, "pilot");
    if (!cJSON_IsNumber(item))
    {
        ESP_LOGE(TAG, "Missing or invalid pilot");
        goto json_error;
    }
    params.pilot = item->valueint;

    item = cJSON_GetObjectItem(root, "rolloff");
    if (!cJSON_IsNumber(item))
    {
        ESP_LOGE(TAG, "Missing or invalid rolloff");
        goto json_error;
    }
    params.rolloff = item->valueint;

    esp_err_t tune_ret = tuner_web_tune(&params);

    if (tune_ret == ESP_OK)
    {
        cJSON_AddStringToObject(json_resp, "message", "Tune command successful");
        httpd_resp_set_type(req, "application/json");
        const char *resp_str = cJSON_PrintUnformatted(json_resp);
        httpd_resp_send(req, resp_str, strlen(resp_str));
        free((void *)resp_str);
    }
    else
    {
        ESP_LOGE(TAG, "Tuner tune command failed with error: %s", esp_err_to_name(tune_ret));
        cJSON_AddStringToObject(json_resp, "message", "Tune command failed");
        cJSON_AddStringToObject(json_resp, "error_code", esp_err_to_name(tune_ret));
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "application/json");
        const char *resp_str = cJSON_PrintUnformatted(json_resp);
        httpd_resp_send(req, resp_str, strlen(resp_str));
        free((void *)resp_str);
    }

    cJSON_Delete(root);
    cJSON_Delete(json_resp);
    return ESP_OK;

json_error:
    ESP_LOGE(TAG, "JSON parsing error during tune request.");
    cJSON_Delete(root);
    cJSON_AddStringToObject(json_resp, "message", "Invalid JSON payload or missing parameters.");
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_set_type(req, "application/json");
    const char *resp_str = cJSON_PrintUnformatted(json_resp);
    httpd_resp_send(req, resp_str, strlen(resp_str));
    free((void *)resp_str);
    cJSON_Delete(json_resp);
    return ESP_FAIL;
}

/* Handler to get tuner reset state */
static esp_err_t tuner_reset_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "reset_active", tuner_reset_get());
    const char *sys_info = cJSON_Print(root);
    httpd_resp_sendstr(req, sys_info);
    free((void *)sys_info);
    cJSON_Delete(root);
    return ESP_OK;
}

/* Handler to set tuner reset state */
static esp_err_t tuner_reset_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret, remaining = req->content_len;

    if (remaining > sizeof(buf) - 1)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content too long");
        return ESP_FAIL;
    }

    while (remaining > 0)
    {
        /* Read a chunk of data */
        if ((ret = httpd_req_recv(req, buf + (req->content_len - remaining), remaining)) <= 0)
        {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT)
            {
                /* Retry receiving if timeout occurred */
                continue;
            }
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive data");
            return ESP_FAIL;
        }
        remaining -= ret;
    }
    buf[req->content_len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (root == NULL)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *reset_json = cJSON_GetObjectItem(root, "reset");
    if (!cJSON_IsBool(reset_json))
    {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid 'reset' field");
        return ESP_FAIL;
    }

    bool reset_active = cJSON_IsTrue(reset_json);

    tuner_reset_set(reset_active);

    ESP_LOGI(TAG, "Tuner reset state set to: %s", reset_active ? "ON" : "OFF");

    cJSON_Delete(root);

    /* Send success response */
    httpd_resp_set_type(req, "application/json");
    cJSON *resp_root = cJSON_CreateObject();
    cJSON_AddStringToObject(resp_root, "message", "Reset state updated successfully");
    cJSON_AddBoolToObject(resp_root, "reset_state", reset_active);
    const char *resp_str = cJSON_Print(resp_root);
    httpd_resp_sendstr(req, resp_str);
    free((void *)resp_str);
    cJSON_Delete(resp_root);

    return ESP_OK;
}

/* Handler to set tuner init */
static esp_err_t tuner_init_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret, remaining = req->content_len;

    if (remaining > sizeof(buf) - 1)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content too long");
        return ESP_FAIL;
    }

    while (remaining > 0)
    {
        /* Read a chunk of data */
        if ((ret = httpd_req_recv(req, buf + (req->content_len - remaining), remaining)) <= 0)
        {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT)
            {
                /* Retry receiving if timeout occurred */
                continue;
            }
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive data");
            return ESP_FAIL;
        }
        remaining -= ret;
    }
    buf[req->content_len] = '\0';

    ret = tuner_init(0);

    ESP_LOGI(TAG, "Tuner init");

    /* Send success response */
    httpd_resp_set_type(req, "application/json");
    cJSON *resp_root = cJSON_CreateObject();
    if (ret == 0)
    {
        cJSON_AddStringToObject(resp_root, "message", "Tuner initialized successfully");
    }
    else
    {
        cJSON_AddStringToObject(resp_root, "message", "Failed to init");
    }
    cJSON_AddBoolToObject(resp_root, "success", ret == 0);
    const char *resp_str = cJSON_Print(resp_root);
    httpd_resp_sendstr(req, resp_str);
    free((void *)resp_str);
    cJSON_Delete(resp_root);

    return ESP_OK;
}

/* Function to get current tuner polarization as a string for web API */
static esp_err_t tuner_get_polarization_str(char *pol_str, size_t str_len) {
    if (!pol_str || str_len < 2) { /* Need space for at least "H" or "V" + null terminator */
        return ESP_ERR_INVALID_ARG;
    }

    voltage_t current_pol = voltage_get_current();
    switch (current_pol) {
        case VOLTAGE_13V:
            strncpy(pol_str, "V", str_len -1);
            break;
        case VOLTAGE_18V:
            strncpy(pol_str, "H", str_len -1);
            break;
        case VOLTAGE_OFF:
            strncpy(pol_str, "OFF", str_len -1);
            break;
        default:
            strncpy(pol_str, "ERR", str_len -1); /* Should not happen */
            return ESP_FAIL;
    }
    pol_str[str_len -1] = '\0'; /* Ensure null termination */
    return ESP_OK;
}

/* Handler to get tuner polarization state */
static esp_err_t tuner_polarization_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    cJSON *root = cJSON_CreateObject();
    if (root == NULL)
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char pol_str[4]; /* "H", "V", or "OFF" */
    esp_err_t ret = tuner_get_polarization_str(pol_str, sizeof(pol_str));

    if (ret == ESP_OK)
    {
        cJSON_AddStringToObject(root, "polarization", pol_str);
    }
    else
    {
        cJSON_AddStringToObject(root, "polarization", "Error"); /* Indicate error */
    }

    const char *json_str = cJSON_PrintUnformatted(root);
    if (json_str == NULL)
    {
        ESP_LOGE(TAG, "Failed to print cJSON to string for polarization GET.");
        cJSON_Delete(root);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, json_str);
    free((void *)json_str);
    cJSON_Delete(root);
    return ESP_OK;
}

/* Handler to set tuner polarization state */
static esp_err_t tuner_polarization_post_handler(httpd_req_t *req)
{
    bool success = true;
    char buf[100];
    int ret, remaining = req->content_len;
    cJSON *resp_root = cJSON_CreateObject();

    if (resp_root == NULL)
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    if (remaining >= sizeof(buf))
    {
        ESP_LOGE(TAG, "Polarization request content too long: %d", remaining);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content too long");
        cJSON_Delete(resp_root);
        return ESP_FAIL;
    }

    ret = httpd_req_recv(req, buf, remaining);
    if (ret <= 0)
    {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT)
        {
            httpd_resp_send_408(req);
        }
        else
        {
            httpd_resp_send_500(req);
        }
        cJSON_Delete(resp_root);
        return ESP_FAIL;
    }
    buf[ret] = '\0';


    cJSON *json_req = cJSON_Parse(buf);
    if (json_req == NULL)
    {
        ESP_LOGE(TAG, "Failed to parse polarization JSON: %s", cJSON_GetErrorPtr());
        cJSON_AddStringToObject(resp_root, "message", "Invalid JSON format");
        cJSON_AddStringToObject(resp_root, "error_detail", cJSON_GetErrorPtr());
        httpd_resp_set_status(req, "400 Bad Request");
        goto send_response;
    }

    cJSON *pol_json = cJSON_GetObjectItem(json_req, "polarization");
    if (!cJSON_IsString(pol_json) || (pol_json->valuestring == NULL))
    {
        cJSON_Delete(json_req);
        cJSON_AddStringToObject(resp_root, "message", "Missing or invalid 'polarization' field (must be string 'H' or 'V')");
        httpd_resp_set_status(req, "400 Bad Request");
        goto send_response;
    }

    const char *pol_str = pol_json->valuestring;
    char *message = NULL;

    if (strcmp(pol_str, "H") == 0)
    {
        voltage_select(VOLTAGE_18V);
        message = "Polarization set to Horizontal (18V)";
        ESP_LOGI(TAG, "%s", message);
    }
    else if (strcmp(pol_str, "V") == 0)
    {
        voltage_select(VOLTAGE_13V);
        message = "Polarization set to Vertical (13V)";
        ESP_LOGI(TAG, "%s", message);
    }
    else
    {
        success = false;
        message = "Invalid polarization value. Expected 'H' or 'V'.";
        ESP_LOGE(TAG, "Invalid polarization value: %s. Expected 'H' or 'V'.", pol_str);
        httpd_resp_set_status(req, "400 Bad Request");
    }

    cJSON_Delete(json_req);
    cJSON_AddStringToObject(resp_root, "message", message);
    cJSON_AddStringToObject(resp_root, "polarization_set", success ? pol_str : "Error");

send_response:
    httpd_resp_set_type(req, "application/json");
    const char *resp_str = cJSON_PrintUnformatted(resp_root);
    if (resp_str)
    {
        httpd_resp_sendstr(req, resp_str);
        free((void *)resp_str);
    }
    cJSON_Delete(resp_root);
    return success ? ESP_OK : ESP_FAIL;
}

static const httpd_uri_t root_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler,
    .user_ctx = NULL};

static const httpd_uri_t tuner_status_uri = {
    .uri = "/api/tuner/status",
    .method = HTTP_GET,
    .handler = tuner_status_get_handler,
    .user_ctx = NULL};

static const httpd_uri_t tuner_reset_get_uri = {
    .uri = "/api/tuner/reset",
    .method = HTTP_GET,
    .handler = tuner_reset_get_handler,
    .user_ctx = NULL};

static const httpd_uri_t tuner_reset_post_uri = {
    .uri = "/api/tuner/reset",
    .method = HTTP_POST,
    .handler = tuner_reset_post_handler,
    .user_ctx = NULL};

static const httpd_uri_t tuner_init_post_uri = {
    .uri = "/api/tuner/init",
    .method = HTTP_POST,
    .handler = tuner_init_post_handler,
    .user_ctx = NULL};

static const httpd_uri_t tuner_polarization_get_uri = {
    .uri = "/api/tuner/polarization",
    .method = HTTP_GET,
    .handler = tuner_polarization_get_handler,
    .user_ctx = NULL};

static const httpd_uri_t tuner_polarization_post_uri = {
    .uri = "/api/tuner/polarization",
    .method = HTTP_POST,
    .handler = tuner_polarization_post_handler,
    .user_ctx = NULL};

static const httpd_uri_t tuner_tune_uri = {
    .uri = "/api/tuner/tune",
    .method = HTTP_POST,
    .handler = tuner_tune_post_handler,
    .user_ctx = NULL};

/* Function to start the web server */
esp_err_t web_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true; /* Enable LRU purge to free up resources */
    config.max_open_sockets = 7;    /* Default is 7, adjust if needed */
    config.stack_size = 8192;       /* Increase stack size for handlers if complex */

    ESP_LOGI(TAG, "Starting httpd server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &tuner_status_uri);
        httpd_register_uri_handler(server, &tuner_tune_uri);
        httpd_register_uri_handler(server, &tuner_reset_get_uri);
        httpd_register_uri_handler(server, &tuner_reset_post_uri);
        httpd_register_uri_handler(server, &tuner_init_post_uri);
        httpd_register_uri_handler(server, &tuner_polarization_get_uri);
        httpd_register_uri_handler(server, &tuner_polarization_post_uri);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Error starting server!");
    return ESP_FAIL;
}

/* Function to stop the web server */
void web_stop(void)
{
    if (server)
    {
        httpd_stop(server);
        server = NULL;
    }
}
