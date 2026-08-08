#include "web.h"

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "i2s.h"
#include "tuner/tuner.h"
#include "voltage.h"

#define INDEX_HTML_PATH "/storage/www/index.htm"
#define CHANNELS_JSON_PATH "/storage/www/chan.jsn"
#define API_BODY_MAX 512

static const char *TAG = "web";
static httpd_handle_t server;

static esp_err_t send_json(httpd_req_t *req, cJSON *body, const char *status)
{
    char *json = cJSON_PrintUnformatted(body);
    if (!json)
    {
        cJSON_Delete(body);
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }

    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    esp_err_t result = httpd_resp_sendstr(req, json);
    cJSON_free(json);
    cJSON_Delete(body);
    return result;
}

static esp_err_t send_result(httpd_req_t *req, const char *status, bool ok, const char *error)
{
    cJSON *body = cJSON_CreateObject();
    if (!body)
    {
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddBoolToObject(body, "ok", ok);
    if (error)
    {
        cJSON_AddStringToObject(body, "error", error);
    }
    return send_json(req, body, status);
}

static cJSON *receive_json(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len >= API_BODY_MAX)
    {
        return NULL;
    }

    char body[API_BODY_MAX];
    size_t received = 0;
    while (received < req->content_len)
    {
        int result = httpd_req_recv(req, body + received, req->content_len - received);
        if (result <= 0)
        {
            return NULL;
        }
        received += result;
    }
    body[received] = '\0';
    return cJSON_Parse(body);
}

static esp_err_t send_file(httpd_req_t *req, const char *path, const char *content_type)
{
    int file = open(path, O_RDONLY);
    if (file < 0)
    {
        ESP_LOGE(TAG, "Cannot open %s", path);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, content_type);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    char buffer[512];
    ssize_t length;
    while ((length = read(file, buffer, sizeof(buffer))) > 0)
    {
        if (httpd_resp_send_chunk(req, buffer, length) != ESP_OK)
        {
            close(file);
            httpd_resp_send_chunk(req, NULL, 0);
            return ESP_FAIL;
        }
    }

    close(file);
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    return send_file(req, INDEX_HTML_PATH, "text/html");
}

static esp_err_t channels_get_handler(httpd_req_t *req)
{
    return send_file(req, CHANNELS_JSON_PATH, "application/json");
}

static esp_err_t favicon_get_handler(httpd_req_t *req)
{
    extern const unsigned char favicon_ico_start[] asm("_binary_favicon_ico_start");
    extern const unsigned char favicon_ico_end[] asm("_binary_favicon_ico_end");
    httpd_resp_set_type(req, "image/x-icon");
    return httpd_resp_send(req, (const char *)favicon_ico_start,
                           favicon_ico_end - favicon_ico_start);
}

static const char *current_polarization(void)
{
    switch (voltage_get_current())
    {
    case VOLTAGE_13V:
        return "V";
    case VOLTAGE_18V:
        return "H";
    default:
        return "OFF";
    }
}

static void add_optional_number(cJSON *body, const char *name, bool available, double value)
{
    if (available)
    {
        cJSON_AddNumberToObject(body, name, value);
    }
    else
    {
        cJSON_AddNullToObject(body, name);
    }
}

static void add_optional_bool(cJSON *body, const char *name, bool available, bool value)
{
    if (available)
    {
        cJSON_AddBoolToObject(body, name, value);
    }
    else
    {
        cJSON_AddNullToObject(body, name);
    }
}

static void add_optional_string(cJSON *body, const char *name, bool available, const char *value)
{
    if (available)
    {
        cJSON_AddStringToObject(body, name, value);
    }
    else
    {
        cJSON_AddNullToObject(body, name);
    }
}

static const char *delivery_name(tuner_delivery_system_t delivery)
{
    return delivery == TUNER_DELIVERY_DVB_S ? "DVB-S" : "DVB-S2";
}

static const char *modulation_name(tuner_modulation_t modulation)
{
    return modulation == TUNER_MODULATION_8PSK ? "8PSK" : "QPSK";
}

static const char *fec_name(tuner_fec_t fec)
{
    static const char *const names[] = {
        "NONE", "1/2", "2/3", "3/4", "4/5", "5/6", "6/7",
        "7/8", "8/9", "AUTO", "3/5", "9/10", "2/5"};
    return fec <= TUNER_FEC_2_5 ? names[fec] : "UNKNOWN";
}

static esp_err_t tuner_get_handler(httpd_req_t *req)
{
    tuner_status_t status = {0};
    i2s_fpga_statistics_t fpga_statistics = {0};
    i2s_fpga_pid_report_t fpga_pid_report = {0};
    i2s_fpga_pat_t fpga_pat = {0};
    char pat_hex[I2S_FPGA_PAT_SIZE * 2 + 1];
    char firmware_version[16] = {0};
    char chip_id[8] = {0};
    bool initialized = tuner_is_initialized();
    if (initialized && tuner_get_status(&status) != ESP_OK)
    {
        return send_result(req, "500 Internal Server Error", false, "status_read_failed");
    }

    cJSON *body = cJSON_CreateObject();
    if (!body)
    {
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddBoolToObject(body, "initialized", initialized);
    cJSON_AddBoolToObject(body, "reset", tuner_get_reset());
    cJSON_AddStringToObject(body, "polarization", current_polarization());
    cJSON_AddNumberToObject(body, "flags", status.flags);
    cJSON_AddBoolToObject(body, "signal", (status.flags & TUNER_STATUS_SIGNAL) != 0);
    cJSON_AddBoolToObject(body, "carrier", (status.flags & TUNER_STATUS_CARRIER) != 0);
    cJSON_AddBoolToObject(body, "viterbi", (status.flags & TUNER_STATUS_VITERBI) != 0);
    cJSON_AddBoolToObject(body, "sync", (status.flags & TUNER_STATUS_SYNC) != 0);
    cJSON_AddBoolToObject(body, "lock", (status.flags & TUNER_STATUS_LOCK) != 0);
    add_optional_number(body, "strength", status.strength_available, status.strength);
    add_optional_number(body, "snr", status.snr_available, status.snr);
    add_optional_number(body, "ber", status.ber_available, status.ber);
    add_optional_number(body, "uncorrected_blocks", status.uncorrected_blocks_available,
                        status.uncorrected_blocks);
    add_optional_number(body, "esno_db", status.esno_available,
                        status.esno_tenth_db / 10.0);
    add_optional_string(body, "acquired_delivery", status.acquired_mode_available,
                        delivery_name(status.acquired_delivery_system));
    add_optional_string(body, "acquired_modulation", status.acquired_mode_available,
                        modulation_name(status.acquired_modulation));
    add_optional_string(body, "acquired_fec", status.acquired_mode_available,
                        fec_name(status.acquired_fec));
    add_optional_string(body, "acquired_pilot", status.acquired_mode_available,
                        status.acquired_pilot_on ? "ON" : "OFF");
    add_optional_bool(body, "lnb_ready", status.lnb_ready_available, status.lnb_ready);
    add_optional_bool(body, "firmware_running", status.firmware_running_available,
                      status.firmware_running);

    if (status.firmware_version_available)
    {
        snprintf(firmware_version, sizeof(firmware_version), "%u.%u.%u.%u",
                 status.firmware_version[0], status.firmware_version[1],
                 status.firmware_version[2], status.firmware_version[3]);
    }
    add_optional_string(body, "firmware_version", status.firmware_version_available,
                        firmware_version);

    if (status.chip_id_available)
    {
        snprintf(chip_id, sizeof(chip_id), "0x%04X", status.chip_id);
    }
    add_optional_string(body, "chip_id", status.chip_id_available, chip_id);

    i2s_get_fpga_statistics(&fpga_statistics);
    cJSON *fpga_body = cJSON_AddObjectToObject(body, "fpga_statistics");
    if (!fpga_body)
    {
        cJSON_Delete(body);
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }
    add_optional_number(fpga_body, "sync_count", fpga_statistics.available,
                        fpga_statistics.sync_count);
    add_optional_number(fpga_body, "data_bytes_received", fpga_statistics.available,
                        fpga_statistics.data_bytes_received);
    add_optional_number(fpga_body, "sync_byte_errors", fpga_statistics.available,
                        fpga_statistics.sync_byte_errors);

    i2s_get_fpga_pid_report(&fpga_pid_report);
    cJSON *pid_body = cJSON_AddObjectToObject(body, "fpga_pid_report");
    if (!pid_body)
    {
        cJSON_Delete(body);
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }
    add_optional_bool(pid_body, "overflow", fpga_pid_report.available,
                      fpga_pid_report.overflow);
    if (fpga_pid_report.available)
    {
        cJSON *pids = cJSON_AddArrayToObject(pid_body, "pids");
        if (!pids)
        {
            cJSON_Delete(body);
            httpd_resp_send_500(req);
            return ESP_ERR_NO_MEM;
        }
        for (uint8_t index = 0; index < fpga_pid_report.count; index++)
        {
            cJSON_AddItemToArray(pids, cJSON_CreateNumber(fpga_pid_report.pids[index]));
        }
    }
    else
    {
        cJSON_AddNullToObject(pid_body, "pids");
    }

    i2s_get_fpga_pat(&fpga_pat);
    cJSON *pat_body = cJSON_AddObjectToObject(body, "fpga_pat");
    if (!pat_body)
    {
        cJSON_Delete(body);
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }
    if (fpga_pat.available)
    {
        static const char hex[] = "0123456789abcdef";
        for (size_t index = 0; index < I2S_FPGA_PAT_SIZE; index++)
        {
            pat_hex[index * 2] = hex[fpga_pat.data[index] >> 4];
            pat_hex[index * 2 + 1] = hex[fpga_pat.data[index] & 0x0f];
        }
        pat_hex[sizeof(pat_hex) - 1] = '\0';
        cJSON_AddNumberToObject(pat_body, "length", I2S_FPGA_PAT_SIZE);
        cJSON_AddStringToObject(pat_body, "hex", pat_hex);
    }
    else
    {
        cJSON_AddNullToObject(pat_body, "length");
        cJSON_AddNullToObject(pat_body, "hex");
    }
    return send_json(req, body, "200 OK");
}

static esp_err_t tuner_init_handler(httpd_req_t *req)
{
    esp_err_t result = tuner_initialize();
    if (result != ESP_OK)
    {
        return send_result(req, "500 Internal Server Error", false, "initialization_failed");
    }
    return send_result(req, "200 OK", true, NULL);
}

static bool json_string_equals(const cJSON *root, const char *name, const char *value)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);
    return cJSON_IsString(item) && strcmp(item->valuestring, value) == 0;
}

static bool parse_fec(const cJSON *root, tuner_fec_t *fec)
{
    static const struct {
        const char *name;
        tuner_fec_t value;
    } values[] = {
        {"NONE", TUNER_FEC_NONE}, {"1/2", TUNER_FEC_1_2}, {"2/3", TUNER_FEC_2_3},
        {"3/4", TUNER_FEC_3_4}, {"4/5", TUNER_FEC_4_5}, {"5/6", TUNER_FEC_5_6},
        {"6/7", TUNER_FEC_6_7}, {"7/8", TUNER_FEC_7_8}, {"8/9", TUNER_FEC_8_9},
        {"AUTO", TUNER_FEC_AUTO}, {"3/5", TUNER_FEC_3_5}, {"9/10", TUNER_FEC_9_10},
        {"2/5", TUNER_FEC_2_5},
    };

    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, "fec");
    if (!cJSON_IsString(item))
    {
        return false;
    }
    for (size_t index = 0; index < sizeof(values) / sizeof(values[0]); ++index)
    {
        if (strcmp(item->valuestring, values[index].name) == 0)
        {
            *fec = values[index].value;
            return true;
        }
    }
    return false;
}

static bool parse_tune_request(const cJSON *root, tuner_tune_request_t *request)
{
    const cJSON *frequency = cJSON_GetObjectItemCaseSensitive(root, "frequency_mhz");
    const cJSON *symbol_rate = cJSON_GetObjectItemCaseSensitive(root, "symbol_rate_ksps");
    if (!cJSON_IsNumber(frequency) || !cJSON_IsNumber(symbol_rate) ||
        !parse_fec(root, &request->fec))
    {
        return false;
    }

    request->frequency_mhz = frequency->valueint;
    request->symbol_rate_ksps = symbol_rate->valueint;

    if (json_string_equals(root, "polarization", "V"))
        request->polarization = TUNER_POLARIZATION_VERTICAL;
    else if (json_string_equals(root, "polarization", "H"))
        request->polarization = TUNER_POLARIZATION_HORIZONTAL;
    else
        return false;

    if (json_string_equals(root, "delivery", "DVB-S"))
        request->delivery_system = TUNER_DELIVERY_DVB_S;
    else if (json_string_equals(root, "delivery", "DVB-S2"))
        request->delivery_system = TUNER_DELIVERY_DVB_S2;
    else
        return false;

    if (json_string_equals(root, "modulation", "QPSK"))
        request->modulation = TUNER_MODULATION_QPSK;
    else if (json_string_equals(root, "modulation", "8PSK"))
        request->modulation = TUNER_MODULATION_8PSK;
    else
        return false;

    if (json_string_equals(root, "inversion", "OFF"))
        request->inversion = TUNER_INVERSION_OFF;
    else if (json_string_equals(root, "inversion", "ON"))
        request->inversion = TUNER_INVERSION_ON;
    else if (json_string_equals(root, "inversion", "AUTO"))
        request->inversion = TUNER_INVERSION_AUTO;
    else
        return false;

    if (json_string_equals(root, "pilot", "ON"))
        request->pilot = TUNER_PILOT_ON;
    else if (json_string_equals(root, "pilot", "OFF"))
        request->pilot = TUNER_PILOT_OFF;
    else if (json_string_equals(root, "pilot", "AUTO"))
        request->pilot = TUNER_PILOT_AUTO;
    else
        return false;

    if (json_string_equals(root, "rolloff", "0.35"))
        request->rolloff = TUNER_ROLLOFF_35;
    else if (json_string_equals(root, "rolloff", "0.20"))
        request->rolloff = TUNER_ROLLOFF_20;
    else if (json_string_equals(root, "rolloff", "0.25"))
        request->rolloff = TUNER_ROLLOFF_25;
    else
        return false;

    return true;
}

static esp_err_t tuner_tune_handler(httpd_req_t *req)
{
    if (!tuner_is_initialized())
    {
        return send_result(req, "409 Conflict", false, "not_initialized");
    }

    cJSON *body = receive_json(req);
    tuner_tune_request_t request;
    if (!body || !parse_tune_request(body, &request))
    {
        cJSON_Delete(body);
        return send_result(req, "400 Bad Request", false, "invalid_tune_request");
    }
    cJSON_Delete(body);

    esp_err_t result = tuner_tune(&request);
    if (result == ESP_ERR_INVALID_ARG)
    {
        return send_result(req, "400 Bad Request", false, "unsupported_parameters");
    }
    if (result != ESP_OK)
    {
        return send_result(req, "502 Bad Gateway", false, "tune_failed");
    }
    return send_result(req, "200 OK", true, NULL);
}

static esp_err_t tuner_reset_handler(httpd_req_t *req)
{
    cJSON *body = receive_json(req);
    const cJSON *active = body ? cJSON_GetObjectItemCaseSensitive(body, "active") : NULL;
    if (!cJSON_IsBool(active))
    {
        cJSON_Delete(body);
        return send_result(req, "400 Bad Request", false, "invalid_reset_request");
    }

    bool reset_active = cJSON_IsTrue(active);
    cJSON_Delete(body);
    tuner_set_reset(reset_active);
    return send_result(req, "200 OK", true, NULL);
}

static const httpd_uri_t routes[] = {
    {.uri = "/", .method = HTTP_GET, .handler = root_get_handler},
    {.uri = "/chan.jsn", .method = HTTP_GET, .handler = channels_get_handler},
    {.uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_get_handler},
    {.uri = "/api/v1/tuner", .method = HTTP_GET, .handler = tuner_get_handler},
    {.uri = "/api/v1/tuner/init", .method = HTTP_POST, .handler = tuner_init_handler},
    {.uri = "/api/v1/tuner/tune", .method = HTTP_POST, .handler = tuner_tune_handler},
    {.uri = "/api/v1/tuner/reset", .method = HTTP_POST, .handler = tuner_reset_handler},
};

esp_err_t web_start(void)
{
    if (server)
    {
        return ESP_ERR_INVALID_STATE;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_open_sockets = 4;
    config.max_uri_handlers = sizeof(routes) / sizeof(routes[0]);
    config.stack_size = 6144;

    esp_err_t result = httpd_start(&server, &config);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "HTTP server start failed: %s", esp_err_to_name(result));
        return result;
    }

    for (size_t index = 0; index < sizeof(routes) / sizeof(routes[0]); ++index)
    {
        result = httpd_register_uri_handler(server, &routes[index]);
        if (result != ESP_OK)
        {
            ESP_LOGE(TAG, "Route registration failed for %s", routes[index].uri);
            web_stop();
            return result;
        }
    }

    ESP_LOGI(TAG, "HTTP tuner console started");
    return ESP_OK;
}

void web_stop(void)
{
    if (server)
    {
        httpd_stop(server);
        server = NULL;
    }
}