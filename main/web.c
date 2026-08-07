/*
 * web.c
 *
 *  Created on: May 29, 2025
 *      Author: GitHub Copilot
 */

#include "web.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "tuner/tuner.h"
#include "voltage.h" /* Added for voltage_select and voltage_get_current */
#include "cJSON.h"
#include <fcntl.h>    /* Added for open/read */
#include <sys/stat.h> /* Added for stat */
#include <string.h>   /* Added for strrchr, strcmp */

static const char *TAG = "web";

#define WEB_MOUNT_POINT "/storage/www"
#define INDEX_HTML_PATH WEB_MOUNT_POINT "/index.htm"
#define UPLOAD_HTML_PATH WEB_MOUNT_POINT "/upload.htm"
#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN)
#define MAX_FILE_SIZE (200 * 1024) // 200 KB
#define MAX_FILE_SIZE_STR "200KB"
#define SCRATCH_BUFSIZE 8192

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

struct file_server_data
{
    /* Base path of file storage */
    char base_path[ESP_VFS_PATH_MAX + 1];

    /* Scratch buffer for temporary storage during file transfer */
    char scratch[SCRATCH_BUFSIZE];
};

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
    item = cJSON_GetObjectItem(root, "freq_mhz");
    if (!cJSON_IsNumber(item))
    {
        ESP_LOGE(TAG, "Missing or invalid freq_mhz");
        goto json_error;
    }
    params.freq_mhz = item->valueint;

    item = cJSON_GetObjectItem(root, "sr_ksps");
    if (!cJSON_IsNumber(item))
    {
        ESP_LOGE(TAG, "Missing or invalid sr_ksps");
        goto json_error;
    }
    params.sr_ksps = item->valueint;

    item = cJSON_GetObjectItem(root, "pol");
    if (!cJSON_IsNumber(item))
    {
        ESP_LOGE(TAG, "Missing or invalid pol");
        goto json_error;
    }
    params.pol = item->valueint;

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
static esp_err_t tuner_get_polarization_str(char *pol_str, size_t str_len)
{
    if (!pol_str || str_len < 2)
    { /* Need space for at least "H" or "V" + null terminator */
        return ESP_ERR_INVALID_ARG;
    }

    voltage_t current_pol = voltage_get_current();
    switch (current_pol)
    {
    case VOLTAGE_13V:
        strncpy(pol_str, "V", str_len - 1);
        break;
    case VOLTAGE_18V:
        strncpy(pol_str, "H", str_len - 1);
        break;
    case VOLTAGE_OFF:
        strncpy(pol_str, "OFF", str_len - 1);
        break;
    default:
        strncpy(pol_str, "ERR", str_len - 1); /* Should not happen */
        return ESP_FAIL;
    }
    pol_str[str_len - 1] = '\0'; /* Ensure null termination */
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

/* Send HTTP response with a run-time generated html consisting of
 * a list of all files and folders under the requested path.
 * In case of SPIFFS this returns empty list when path is any
 * string other than '/', since SPIFFS doesn't support directories */
static esp_err_t http_resp_dir_html(httpd_req_t *req, const char *dirpath)
{
    char entrypath[FILE_PATH_MAX];
    char entrysize[16];
    const char *entrytype;

    struct dirent *entry;
    struct stat entry_stat;

    DIR *dir = opendir(dirpath);
    const size_t dirpath_len = strlen(dirpath);

    /* Retrieve the base path of file storage to construct the full path */
    strlcpy(entrypath, dirpath, sizeof(entrypath));

    if (!dir)
    {
        ESP_LOGE(TAG, "Failed to stat dir : %s", dirpath);
        /* Respond with 404 Not Found */
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Directory does not exist");
        return ESP_FAIL;
    }

    /* Send HTML file header */
    httpd_resp_sendstr_chunk(req, "<!DOCTYPE html><html><body>");

    /* Get handle to embedded file upload script */
    FILE *fp = fopen(UPLOAD_HTML_PATH, "rb");
    if (!fp)
    {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Upload script does not exist");
        return ESP_FAIL;
    }

    fseek(fp, 0, SEEK_END);
    long length = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    if (length <= 0)
    {
        ESP_LOGE(TAG, "Upload script is empty or invalid size: %s", UPLOAD_HTML_PATH);
        fclose(fp);
        return 1;
    }

    uint8_t *script_data = (uint8_t *)malloc(length);
    if (!script_data)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for Upload script (%ld bytes)", length);
        fclose(fp);
        return 1;
    }

    size_t read_bytes = fread(script_data, 1, length, fp);
    fclose(fp);

    /* Add file upload form and script which on execution sends a POST request to /upload */
    httpd_resp_send_chunk(req, (const char *)script_data, read_bytes);

    /* Send file-list table definition and column labels */
    httpd_resp_sendstr_chunk(req,
                             "<table class=\"fixed\" border=\"1\">"
                             "<col width=\"800px\" /><col width=\"300px\" /><col width=\"300px\" /><col width=\"100px\" />"
                             "<thead><tr><th>Name</th><th>Type</th><th>Size (Bytes)</th><th>Delete</th></tr></thead>"
                             "<tbody>");

    /* Iterate over all files / folders and fetch their names and sizes */
    while ((entry = readdir(dir)) != NULL)
    {
        entrytype = (entry->d_type == DT_DIR ? "directory" : "file");

        strlcpy(entrypath + dirpath_len, entry->d_name, sizeof(entrypath) - dirpath_len);
        if (stat(entrypath, &entry_stat) == -1)
        {
            ESP_LOGE(TAG, "Failed to stat %s : %s", entrytype, entry->d_name);
            continue;
        }
        sprintf(entrysize, "%ld", entry_stat.st_size);
        ESP_LOGI(TAG, "Found %s : %s (%s bytes)", entrytype, entry->d_name, entrysize);

        /* Send chunk of HTML file containing table entries with file name and size */
        httpd_resp_sendstr_chunk(req, "<tr><td><a href=\"");
        httpd_resp_sendstr_chunk(req, req->uri);
        httpd_resp_sendstr_chunk(req, entry->d_name);
        if (entry->d_type == DT_DIR)
        {
            httpd_resp_sendstr_chunk(req, "/");
        }
        httpd_resp_sendstr_chunk(req, "\">");
        httpd_resp_sendstr_chunk(req, entry->d_name);
        httpd_resp_sendstr_chunk(req, "</a></td><td>");
        httpd_resp_sendstr_chunk(req, entrytype);
        httpd_resp_sendstr_chunk(req, "</td><td>");
        httpd_resp_sendstr_chunk(req, entrysize);
        httpd_resp_sendstr_chunk(req, "</td><td>");
        httpd_resp_sendstr_chunk(req, "<form method=\"post\" action=\"/delete");
        httpd_resp_sendstr_chunk(req, req->uri);
        httpd_resp_sendstr_chunk(req, entry->d_name);
        httpd_resp_sendstr_chunk(req, "\"><button type=\"submit\">Delete</button></form>");
        httpd_resp_sendstr_chunk(req, "</td></tr>\n");
    }
    closedir(dir);

    /* Finish the file list table */
    httpd_resp_sendstr_chunk(req, "</tbody></table>");

    /* Send remaining chunk of HTML file to complete it */
    httpd_resp_sendstr_chunk(req, "</body></html>");

    /* Send empty chunk to signal HTTP response completion */
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

#define IS_FILE_EXT(filename, ext) \
    (strcasecmp(&filename[strlen(filename) - sizeof(ext) + 1], ext) == 0)

/* Set HTTP response content type according to file extension */
static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename)
{
    if (IS_FILE_EXT(filename, ".pdf"))
    {
        return httpd_resp_set_type(req, "application/pdf");
    }
    else if (IS_FILE_EXT(filename, ".html"))
    {
        return httpd_resp_set_type(req, "text/html");
    }
    else if (IS_FILE_EXT(filename, ".jpeg"))
    {
        return httpd_resp_set_type(req, "image/jpeg");
    }
    else if (IS_FILE_EXT(filename, ".ico"))
    {
        return httpd_resp_set_type(req, "image/x-icon");
    }
    /* This is a limited set only */
    /* For any other type always set as plain text */
    return httpd_resp_set_type(req, "text/plain");
}

/* Copies the full path into destination buffer and returns
 * pointer to path (skipping the preceding base path) */
static const char *get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize)
{
    const size_t base_pathlen = strlen(base_path);
    size_t pathlen = strlen(uri);

    const char *quest = strchr(uri, '?');
    if (quest)
    {
        pathlen = MIN(pathlen, quest - uri);
    }
    const char *hash = strchr(uri, '#');
    if (hash)
    {
        pathlen = MIN(pathlen, hash - uri);
    }

    if (base_pathlen + pathlen + 1 > destsize)
    {
        /* Full path string won't fit into destination buffer */
        return NULL;
    }

    /* Construct full path (base + path) */
    strcpy(dest, base_path);
    strlcpy(dest + base_pathlen, uri, pathlen + 1);

    /* Return pointer to path, skipping the base */
    return dest + base_pathlen;
}

/* Handler to redirect incoming GET request for /index.html to /
 * This can be overridden by uploading file with same name */
static esp_err_t index_html_get_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "307 Temporary Redirect");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0); // Response body can be empty
    return ESP_OK;
}

/* Handler to respond with an icon file embedded in flash.
 * Browsers expect to GET website icon at URI /favicon.ico.
 * This can be overridden by uploading file with same name */
static esp_err_t favicon_get_handler(httpd_req_t *req)
{
    extern const unsigned char favicon_ico_start[] asm("_binary_favicon_ico_start");
    extern const unsigned char favicon_ico_end[] asm("_binary_favicon_ico_end");
    const size_t favicon_ico_size = (favicon_ico_end - favicon_ico_start);
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_send(req, (const char *)favicon_ico_start, favicon_ico_size);
    return ESP_OK;
}

/* Handler to download a file kept on the server */
static esp_err_t download_get_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    FILE *fd = NULL;
    struct stat file_stat;

    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,
                                             req->uri, sizeof(filepath));
    if (!filename)
    {
        ESP_LOGE(TAG, "Filename is too long");
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    /* If name has trailing '/', respond with directory contents */
    if (filename[strlen(filename) - 1] == '/')
    {
        return http_resp_dir_html(req, filepath);
    }

    if (stat(filepath, &file_stat) == -1)
    {
        /* If file not present on SPIFFS check if URI
         * corresponds to one of the hardcoded paths */
        if (strcmp(filename, "/index.html") == 0)
        {
            return index_html_get_handler(req);
        }
        else if (strcmp(filename, "/favicon.ico") == 0)
        {
            return favicon_get_handler(req);
        }
        ESP_LOGE(TAG, "Failed to stat file : %s", filepath);
        /* Respond with 404 Not Found */
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File does not exist");
        return ESP_FAIL;
    }

    fd = fopen(filepath, "r");
    if (!fd)
    {
        ESP_LOGE(TAG, "Failed to read existing file : %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sending file : %s (%ld bytes)...", filename, file_stat.st_size);
    set_content_type_from_file(req, filename);

    /* Retrieve the pointer to scratch buffer for temporary storage */
    char *chunk = ((struct file_server_data *)req->user_ctx)->scratch;
    size_t chunksize;
    do
    {
        /* Read file in chunks into the scratch buffer */
        chunksize = fread(chunk, 1, SCRATCH_BUFSIZE, fd);

        if (chunksize > 0)
        {
            /* Send the buffer contents as HTTP response chunk */
            if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK)
            {
                fclose(fd);
                ESP_LOGE(TAG, "File sending failed!");
                /* Abort sending file */
                httpd_resp_sendstr_chunk(req, NULL);
                /* Respond with 500 Internal Server Error */
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
                return ESP_FAIL;
            }
        }

        /* Keep looping till the whole file is sent */
    } while (chunksize != 0);

    /* Close file after sending complete */
    fclose(fd);
    ESP_LOGI(TAG, "File sending complete");

    /* Respond with an empty chunk to signal HTTP response completion */
#ifdef CONFIG_EXAMPLE_HTTPD_CONN_CLOSE_HEADER
    httpd_resp_set_hdr(req, "Connection", "close");
#endif
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* Handler to upload a file onto the server */
static esp_err_t upload_post_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    FILE *fd = NULL;
    struct stat file_stat;

    /* Skip leading "/upload" from URI to get filename */
    /* Note sizeof() counts NULL termination hence the -1 */
    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,
                                             req->uri + sizeof("/upload") - 1, sizeof(filepath));
    if (!filename)
    {
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    /* Filename cannot have a trailing '/' */
    if (filename[strlen(filename) - 1] == '/')
    {
        ESP_LOGE(TAG, "Invalid filename : %s", filename);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid filename");
        return ESP_FAIL;
    }

    if (stat(filepath, &file_stat) == 0)
    {
        ESP_LOGE(TAG, "File already exists : %s", filepath);
        /* Respond with 400 Bad Request */
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File already exists");
        return ESP_FAIL;
    }

    /* File cannot be larger than a limit */
    if (req->content_len > MAX_FILE_SIZE)
    {
        ESP_LOGE(TAG, "File too large : %d bytes", req->content_len);
        /* Respond with 400 Bad Request */
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "File size must be less than " MAX_FILE_SIZE_STR "!");
        /* Return failure to close underlying connection else the
         * incoming file content will keep the socket busy */
        return ESP_FAIL;
    }

    fd = fopen(filepath, "w");
    if (!fd)
    {
        ESP_LOGE(TAG, "Failed to create file : %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to create file");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Receiving file : %s...", filename);

    /* Retrieve the pointer to scratch buffer for temporary storage */
    char *buf = ((struct file_server_data *)req->user_ctx)->scratch;
    int received;

    /* Content length of the request gives
     * the size of the file being uploaded */
    int remaining = req->content_len;

    while (remaining > 0)
    {

        ESP_LOGI(TAG, "Remaining size : %d", remaining);
        /* Receive the file part by part into a buffer */
        if ((received = httpd_req_recv(req, buf, MIN(remaining, SCRATCH_BUFSIZE))) <= 0)
        {
            if (received == HTTPD_SOCK_ERR_TIMEOUT)
            {
                /* Retry if timeout occurred */
                continue;
            }

            /* In case of unrecoverable error,
             * close and delete the unfinished file*/
            fclose(fd);
            unlink(filepath);

            ESP_LOGE(TAG, "File reception failed!");
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive file");
            return ESP_FAIL;
        }

        /* Write buffer content to file on storage */
        if (received && (received != fwrite(buf, 1, received, fd)))
        {
            /* Couldn't write everything to file!
             * Storage may be full? */
            fclose(fd);
            unlink(filepath);

            ESP_LOGE(TAG, "File write failed!");
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to write file to storage");
            return ESP_FAIL;
        }

        /* Keep track of remaining size of
         * the file left to be uploaded */
        remaining -= received;
    }

    /* Close file upon upload completion */
    fclose(fd);
    ESP_LOGI(TAG, "File reception complete");

    /* Redirect onto root to see the updated file list */
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
#ifdef CONFIG_EXAMPLE_HTTPD_CONN_CLOSE_HEADER
    httpd_resp_set_hdr(req, "Connection", "close");
#endif
    httpd_resp_sendstr(req, "File uploaded successfully");
    return ESP_OK;
}

/* Handler to delete a file from the server */
static esp_err_t delete_post_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    struct stat file_stat;

    /* Skip leading "/delete" from URI to get filename */
    /* Note sizeof() counts NULL termination hence the -1 */
    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,
                                             req->uri + sizeof("/delete") - 1, sizeof(filepath));
    if (!filename)
    {
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    /* Filename cannot have a trailing '/' */
    if (filename[strlen(filename) - 1] == '/')
    {
        ESP_LOGE(TAG, "Invalid filename : %s", filename);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid filename");
        return ESP_FAIL;
    }

    if (stat(filepath, &file_stat) == -1)
    {
        ESP_LOGE(TAG, "File does not exist : %s", filename);
        /* Respond with 400 Bad Request */
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File does not exist");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Deleting file : %s", filename);
    /* Delete file */
    unlink(filepath);

    /* Redirect onto root to see the updated file list */
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
#ifdef CONFIG_EXAMPLE_HTTPD_CONN_CLOSE_HEADER
    httpd_resp_set_hdr(req, "Connection", "close");
#endif
    httpd_resp_sendstr(req, "File deleted successfully");
    return ESP_OK;
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
    static struct file_server_data *server_data = NULL;

    if (server_data)
    {
        ESP_LOGE(TAG, "File server already started");
        return ESP_ERR_INVALID_STATE;
    }

    /* Allocate memory for server data */
    server_data = calloc(1, sizeof(struct file_server_data));
    if (!server_data)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for server data");
        return ESP_ERR_NO_MEM;
    }
    strlcpy(server_data->base_path, WEB_MOUNT_POINT, sizeof(server_data->base_path));

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true; /* Enable LRU purge to free up resources */
    config.max_open_sockets = 7;    /* Default is 7, adjust if needed */
    config.stack_size = 8192;       /* Increase stack size for handlers if complex */
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.max_uri_handlers = 16;

    ESP_LOGI(TAG, "Starting httpd server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) != ESP_OK)
    {

        ESP_LOGE(TAG, "Error starting server!");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(server, &root_uri);
    httpd_register_uri_handler(server, &tuner_status_uri);
    httpd_register_uri_handler(server, &tuner_tune_uri);
    httpd_register_uri_handler(server, &tuner_reset_get_uri);
    httpd_register_uri_handler(server, &tuner_reset_post_uri);
    httpd_register_uri_handler(server, &tuner_init_post_uri);
    httpd_register_uri_handler(server, &tuner_polarization_get_uri);
    httpd_register_uri_handler(server, &tuner_polarization_post_uri);

    /* URI handler for getting uploaded files */
    httpd_uri_t file_download = {
        .uri = "*", // Match all URIs of type /path/to/file
        .method = HTTP_GET,
        .handler = download_get_handler,
        .user_ctx = server_data // Pass server data as context
    };
    httpd_register_uri_handler(server, &file_download);

    /* URI handler for uploading files to server */
    httpd_uri_t file_upload = {
        .uri = "/upload/*", // Match all URIs of type /upload/path/to/file
        .method = HTTP_POST,
        .handler = upload_post_handler,
        .user_ctx = server_data // Pass server data as context
    };
    httpd_register_uri_handler(server, &file_upload);

    /* URI handler for deleting files from server */
    httpd_uri_t file_delete = {
        .uri = "/delete/*", // Match all URIs of type /delete/path/to/file
        .method = HTTP_POST,
        .handler = delete_post_handler,
        .user_ctx = server_data // Pass server data as context
    };
    httpd_register_uri_handler(server, &file_delete);

    return ESP_OK;
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
