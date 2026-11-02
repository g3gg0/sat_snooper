#include "web.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "cJSON.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbedtls/base64.h"
#include "i2s.h"
#include "channel_config.h"
#include "tuner/tuner.h"
#include "ts_stream.h"
#include "voltage.h"

#define INDEX_HTML_PATH "/storage/www/index.htm"
#define CHANNELS_JSON_PATH "/storage/www/chan.jsn"
#define API_BODY_MAX 512

static const char *TAG = "web";
static httpd_handle_t server;

static void packet_to_hex(const uint8_t *packet, char *output);
static bool packet_to_base64(const uint8_t *packet, char *output, size_t output_size);

/* The UI may be opened from a local file, which sends Origin: null */
static void set_cors_headers(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_set_hdr(req, "Access-Control-Max-Age", "600");
}

static esp_err_t options_handler(httpd_req_t *req)
{
    set_cors_headers(req);
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

/* Shadow the IDF inline helpers so error replies carry the CORS headers too */
#define httpd_resp_send_404(req) \
    (set_cors_headers(req), httpd_resp_send_err((req), HTTPD_404_NOT_FOUND, NULL))
#define httpd_resp_send_500(req) \
    (set_cors_headers(req), httpd_resp_send_err((req), HTTPD_500_INTERNAL_SERVER_ERROR, NULL))

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
    set_cors_headers(req);
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
    set_cors_headers(req);

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
    set_cors_headers(req);
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
    i2s_retained_pid_buffer_t retained_pid_buffers[I2S_RETAINED_PID_SLOT_COUNT] = {0};
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
    add_optional_number(fpga_body, "filtered_drops", fpga_statistics.available,
                        fpga_statistics.filtered_drops);

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

    i2s_get_retained_pid_buffers(retained_pid_buffers);
    cJSON *control_buffers_body = cJSON_AddArrayToObject(body, "retained_pid_buffers");
    if (!control_buffers_body)
    {
        cJSON_Delete(body);
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }
    for (size_t index = 0; index < I2S_RETAINED_PID_SLOT_COUNT; index++)
    {
        if (!retained_pid_buffers[index].available)
        {
            continue;
        }
        cJSON *buffer_body = cJSON_CreateObject();
        if (!buffer_body)
        {
            cJSON_Delete(body);
            httpd_resp_send_500(req);
            return ESP_ERR_NO_MEM;
        }
        cJSON_AddNumberToObject(buffer_body, "pid", retained_pid_buffers[index].pid);
        cJSON_AddNumberToObject(buffer_body, "retained_packets",
                    retained_pid_buffers[index].packet_count);
        cJSON_AddNumberToObject(buffer_body, "packets_received",
                    retained_pid_buffers[index].packets_received);
        cJSON_AddNumberToObject(buffer_body, "continuity_errors",
                    retained_pid_buffers[index].continuity_errors);
        cJSON_AddItemToArray(control_buffers_body, buffer_body);
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

    cJSON *heap_body = cJSON_AddObjectToObject(body, "heap");
    if (!heap_body)
    {
        cJSON_Delete(body);
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddNumberToObject(heap_body, "free_internal",
                            heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    cJSON_AddNumberToObject(heap_body, "min_free_internal",
                            heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
    cJSON_AddNumberToObject(heap_body, "largest_block_internal",
                            heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    cJSON_AddNumberToObject(heap_body, "free_spiram",
                            heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    cJSON_AddNumberToObject(heap_body, "min_free_spiram",
                            heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM));
    cJSON_AddNumberToObject(heap_body, "largest_block_spiram",
                            heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
    cJSON_AddNumberToObject(heap_body, "uptime_s", esp_timer_get_time() / 1000000);
    cJSON_AddNumberToObject(heap_body, "min_task_stack",
                            uxTaskGetStackHighWaterMark(NULL));

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

static esp_err_t tuner_tune_handler(httpd_req_t *req)
{
    if (!tuner_is_initialized())
    {
        return send_result(req, "409 Conflict", false, "not_initialized");
    }

    cJSON *body = receive_json(req);
    tuner_tune_request_t request;
    if (!body || !channel_config_parse_tune_request(body, &request))
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

static esp_err_t retained_pid_get_handler(httpd_req_t *req)
{
    char query[16];
    char pid_value[6];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "pid", pid_value, sizeof(pid_value)) != ESP_OK)
    {
        return send_result(req, "400 Bad Request", false, "missing_pid");
    }

    char *end = NULL;
    unsigned long parsed_pid = strtoul(pid_value, &end, 0);
    if (!end || *end != '\0' || parsed_pid > 0x1fff)
    {
        return send_result(req, "400 Bad Request", false, "invalid_pid");
    }

    uint8_t *packets = heap_caps_malloc(I2S_RETAINED_PID_PACKET_CAPACITY * TS_PACKET_SIZE,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!packets)
    {
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }

    size_t packet_count = i2s_copy_retained_pid_packets(parsed_pid, packets,
                                                         I2S_RETAINED_PID_PACKET_CAPACITY);
    char header[80];
    int header_length = snprintf(header, sizeof(header),
                                 "{\"pid\":%lu,\"retained_packets\":%u,\"packets\":[",
                                 parsed_pid, (unsigned int)packet_count);
    if (header_length < 0 || header_length >= sizeof(header))
    {
        free(packets);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "application/json");
    set_cors_headers(req);
    if (httpd_resp_send_chunk(req, header, header_length) != ESP_OK)
    {
        free(packets);
        httpd_resp_send_chunk(req, NULL, 0);
        return ESP_FAIL;
    }

    for (size_t index = 0; index < packet_count; index++)
    {
        char packet_base64[4 * ((TS_PACKET_SIZE + 2) / 3) + 1];
        const char *prefix = index == 0 ? "\"" : ",\"";
        if (!packet_to_base64(packets + index * TS_PACKET_SIZE, packet_base64,
                              sizeof(packet_base64)))
        {
            free(packets);
            httpd_resp_send_chunk(req, NULL, 0);
            return ESP_FAIL;
        }
        if (httpd_resp_send_chunk(req, prefix, strlen(prefix)) != ESP_OK ||
            httpd_resp_send_chunk(req, packet_base64, strlen(packet_base64)) != ESP_OK ||
            httpd_resp_send_chunk(req, "\"", 1) != ESP_OK)
        {
            free(packets);
            httpd_resp_send_chunk(req, NULL, 0);
            return ESP_FAIL;
        }
    }
    free(packets);
    if (httpd_resp_send_chunk(req, "]}", 2) != ESP_OK)
    {
        httpd_resp_send_chunk(req, NULL, 0);
        return ESP_FAIL;
    }
    return httpd_resp_send_chunk(req, NULL, 0);
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

static void packet_to_hex(const uint8_t *packet, char *output)
{
    static const char hex[] = "0123456789abcdef";
    for (size_t index = 0; index < TS_PACKET_SIZE; index++)
    {
        output[index * 2] = hex[packet[index] >> 4];
        output[index * 2 + 1] = hex[packet[index] & 0x0f];
    }
    output[TS_PACKET_SIZE * 2] = '\0';
}

static bool packet_to_base64(const uint8_t *packet, char *output, size_t output_size)
{
    size_t output_length = 0;
    /* mbedtls needs room for its own terminator, so pass the full buffer size */
    if (mbedtls_base64_encode((unsigned char *)output, output_size, &output_length,
                              packet, TS_PACKET_SIZE) != 0)
    {
        return false;
    }
    output[output_length] = '\0';
    return true;
}

static esp_err_t filters_get_handler(httpd_req_t *req)
{
    ts_stream_status_t status = {0};
    ts_stream_get_status(&status);
    cJSON *body = cJSON_CreateObject();
    cJSON *filters = body ? cJSON_AddArrayToObject(body, "filters") : NULL;
    if (!body || !filters)
    {
        cJSON_Delete(body);
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddNumberToObject(body, "udp_port", TS_UDP_PORT);
    cJSON_AddBoolToObject(body, "udp_enabled", status.udp_enabled);
    cJSON_AddNumberToObject(body, "udp_max_rate_kib_s", status.udp_max_rate_kib_s);
    cJSON_AddNumberToObject(body, "udp_datagrams", status.udp_datagram_count);
    cJSON_AddNumberToObject(body, "udp_packets", status.udp_packet_count);
    cJSON_AddNumberToObject(body, "udp_send_errors", status.udp_send_error_count);
    cJSON_AddNumberToObject(body, "udp_throttle_drops", status.udp_throttle_drop_count);
    cJSON_AddNumberToObject(body, "queue_drops", status.queue_drop_count);

    for (size_t index = 0; index < FPGA_PID_FILTER_COUNT; index++)
    {
        const ts_filter_status_t *filter_status = &status.filters[index];
        cJSON *filter = cJSON_CreateObject();
        if (!filter)
        {
            cJSON_Delete(body);
            httpd_resp_send_500(req);
            return ESP_ERR_NO_MEM;
        }
        cJSON_AddItemToArray(filters, filter);
        cJSON_AddBoolToObject(filter, "enabled", filter_status->config.enabled);
        cJSON_AddNumberToObject(filter, "pid", filter_status->config.pid);
        cJSON_AddNumberToObject(filter, "mask", filter_status->config.mask);
        cJSON_AddNumberToObject(filter, "packet_count", filter_status->packet_count);
        cJSON_AddNumberToObject(filter, "byte_count", filter_status->byte_count);
        cJSON_AddNumberToObject(filter, "packets_per_second", filter_status->packets_per_second);
        cJSON_AddNumberToObject(filter, "bytes_per_second", filter_status->bytes_per_second);
        if (filter_status->latest_available)
        {
            char packet_hex[TS_PACKET_SIZE * 2 + 1];
            packet_to_hex(filter_status->latest_packet, packet_hex);
            cJSON_AddStringToObject(filter, "latest_packet", packet_hex);
        }
        else
        {
            cJSON_AddNullToObject(filter, "latest_packet");
        }
    }
    return send_json(req, body, "200 OK");
}

static bool parse_filter_request(const cJSON *root,
                                 fpga_pid_filter_t filters[FPGA_PID_FILTER_COUNT])
{
    const cJSON *items = cJSON_GetObjectItemCaseSensitive(root, "filters");
    if (!cJSON_IsArray(items) || cJSON_GetArraySize(items) > FPGA_PID_FILTER_COUNT)
    {
        return false;
    }

    size_t index = 0;
    const cJSON *item;
    cJSON_ArrayForEach(item, items)
    {
        const cJSON *enabled = cJSON_GetObjectItemCaseSensitive(item, "enabled");
        const cJSON *pid = cJSON_GetObjectItemCaseSensitive(item, "pid");
        const cJSON *mask = cJSON_GetObjectItemCaseSensitive(item, "mask");
        if (!cJSON_IsBool(enabled) || !cJSON_IsNumber(pid) || !cJSON_IsNumber(mask) ||
            pid->valuedouble != pid->valueint || mask->valuedouble != mask->valueint ||
            pid->valueint < 0 || pid->valueint > 0x1fff ||
            mask->valueint < 0 || mask->valueint > 0x1fff)
        {
            return false;
        }
        filters[index].enabled = cJSON_IsTrue(enabled);
        filters[index].pid = pid->valueint;
        filters[index].mask = mask->valueint;
        index++;
    }
    return true;
}

static esp_err_t filters_post_handler(httpd_req_t *req)
{
    cJSON *body = receive_json(req);
    fpga_pid_filter_t filters[FPGA_PID_FILTER_COUNT] = {0};
    if (!body || !parse_filter_request(body, filters))
    {
        cJSON_Delete(body);
        return send_result(req, "400 Bad Request", false, "invalid_filter_request");
    }
    cJSON_Delete(body);

    esp_err_t result = ts_stream_configure(filters);
    if (result != ESP_OK)
    {
        return send_result(req, "502 Bad Gateway", false, "fpga_filter_write_failed");
    }
    return send_result(req, "200 OK", true, NULL);
}

static esp_err_t udp_post_handler(httpd_req_t *req)
{
    cJSON *body = receive_json(req);
    const cJSON *enabled = body ? cJSON_GetObjectItemCaseSensitive(body, "enabled") : NULL;
    const cJSON *rate = body ? cJSON_GetObjectItemCaseSensitive(body, "max_rate_kib_s") : NULL;
    if (!cJSON_IsBool(enabled) ||
        (rate && (!cJSON_IsNumber(rate) || rate->valuedouble < 0 || rate->valuedouble > 100000)))
    {
        cJSON_Delete(body);
        return send_result(req, "400 Bad Request", false, "invalid_udp_request");
    }

    bool stream_enabled = cJSON_IsTrue(enabled);
    uint32_t max_rate = rate ? (uint32_t)rate->valuedouble : 0;
    cJSON_Delete(body);

    if (ts_stream_set_udp(stream_enabled, max_rate) != ESP_OK)
    {
        return send_result(req, "500 Internal Server Error", false, "udp_config_failed");
    }
    return send_result(req, "200 OK", true, NULL);
}

static const httpd_uri_t routes[] = {
    {.uri = "/", .method = HTTP_GET, .handler = root_get_handler},
    {.uri = "/chan.jsn", .method = HTTP_GET, .handler = channels_get_handler},
    {.uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_get_handler},
    {.uri = "/api/v1/tuner", .method = HTTP_GET, .handler = tuner_get_handler},
    {.uri = "/api/v1/tuner/retained-pid", .method = HTTP_GET, .handler = retained_pid_get_handler},
    {.uri = "/api/v1/tuner/init", .method = HTTP_POST, .handler = tuner_init_handler},
    {.uri = "/api/v1/tuner/tune", .method = HTTP_POST, .handler = tuner_tune_handler},
    {.uri = "/api/v1/tuner/reset", .method = HTTP_POST, .handler = tuner_reset_handler},
    {.uri = "/api/v1/tuner/filters", .method = HTTP_GET, .handler = filters_get_handler},
    {.uri = "/api/v1/tuner/filters", .method = HTTP_POST, .handler = filters_post_handler},
    {.uri = "/api/v1/tuner/udp", .method = HTTP_POST, .handler = udp_post_handler},
    {.uri = "/*", .method = HTTP_OPTIONS, .handler = options_handler},
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
    config.uri_match_fn = httpd_uri_match_wildcard;

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