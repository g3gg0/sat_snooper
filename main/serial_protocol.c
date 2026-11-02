#include "serial_protocol.h"

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_private/usb_console.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "i2s.h"
#include "channel_config.h"
#include "ts_stream.h"
#include "tuner/tuner.h"
#include "voltage.h"

#define SERIAL_MAGIC_FIRST 'S'
#define SERIAL_MAGIC_SECOND 'S'
#define SERIAL_MAX_COMMAND_SIZE 512
#define SERIAL_MAX_FPGA_FRAME_SIZE 257
#define SERIAL_TX_QUEUE_LENGTH 64
#define SERIAL_TASK_STACK_SIZE 6144
#define SERIAL_TASK_PRIORITY 4

static SemaphoreHandle_t serial_write_lock;
static QueueHandle_t serial_tx_queue;
static volatile bool stream_enabled;

typedef struct {
    uint16_t length;
    uint8_t payload[SERIAL_MAX_FPGA_FRAME_SIZE];
} serial_tx_item_t;

static int discard_log_output(const char *format, va_list arguments)
{
    (void)format;
    (void)arguments;
    return 0;
}

/* Log output would corrupt the binary stream, so mute it only while streaming */
static void serial_protocol_mute_logs(void)
{
    esp_log_set_vprintf(discard_log_output);
}

static bool serial_write_bytes(const uint8_t *data, size_t length)
{
    size_t offset = 0;
    while (offset < length)
    {
        ssize_t result = esp_usb_console_write_buf((const char *)data + offset,
                                                   length - offset);
        if (result < 0)
        {
            return false;
        }
        if (result == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        offset += result;
    }
    return true;
}

static bool serial_flush(void)
{
    while (true)
    {
        ssize_t result = esp_usb_console_flush();
        if (result < 0)
        {
            return false;
        }
        if (result > 0 || esp_usb_console_write_available())
        {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void serial_send_frame(uint8_t type, const uint8_t *payload, uint16_t length)
{
    uint8_t header[] = {
        SERIAL_MAGIC_FIRST,
        SERIAL_MAGIC_SECOND,
        type,
        (uint8_t)(length & 0xff),
        (uint8_t)(length >> 8),
    };
    if (!serial_write_lock || xSemaphoreTake(serial_write_lock, portMAX_DELAY) != pdTRUE)
    {
        return;
    }

    bool complete = serial_write_bytes(header, sizeof(header)) &&
                    serial_write_bytes(payload, length);
    if (complete)
    {
        serial_flush();
    }
    xSemaphoreGive(serial_write_lock);
}

static void serial_send_response(uint32_t id, bool ok, cJSON *data, const char *error)
{
    cJSON *response = cJSON_CreateObject();
    if (!response)
    {
        cJSON_Delete(data);
        return;
    }
    cJSON_AddNumberToObject(response, "id", id);
    cJSON_AddBoolToObject(response, "ok", ok);
    if (data)
    {
        cJSON_AddItemToObject(response, "data", data);
    }
    if (error)
    {
        cJSON_AddStringToObject(response, "error", error);
    }

    char *text = cJSON_PrintUnformatted(response);
    cJSON_Delete(response);
    if (!text)
    {
        return;
    }
    serial_send_frame(ok ? SERIAL_PROTOCOL_TYPE_RESPONSE : SERIAL_PROTOCOL_TYPE_ERROR,
                      (const uint8_t *)text, strlen(text));
    cJSON_free(text);
}

static void serial_tx_task(void *arg)
{
    (void)arg;
    serial_tx_item_t item;
    while (true)
    {
        if (xQueueReceive(serial_tx_queue, &item, portMAX_DELAY) == pdTRUE)
        {
            serial_send_frame(SERIAL_PROTOCOL_TYPE_FPGA_FRAME, item.payload,
                              item.length);
        }
    }
}

static void add_optional_number(cJSON *object, const char *name, bool available, double value)
{
    if (available)
    {
        cJSON_AddNumberToObject(object, name, value);
    }
    else
    {
        cJSON_AddNullToObject(object, name);
    }
}

static void add_optional_bool(cJSON *object, const char *name, bool available, bool value)
{
    if (available)
    {
        cJSON_AddBoolToObject(object, name, value);
    }
    else
    {
        cJSON_AddNullToObject(object, name);
    }
}

static const char *polarization_name(void)
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

static cJSON *build_status(void)
{
    tuner_status_t tuner_status = {0};
    i2s_fpga_statistics_t fpga_statistics = {0};
    i2s_fpga_pid_report_t pid_report = {0};
    i2s_fpga_pat_t pat = {0};
    i2s_retained_pid_buffer_t buffers[I2S_RETAINED_PID_SLOT_COUNT] = {0};
    bool initialized = tuner_is_initialized();
    if (initialized && tuner_get_status(&tuner_status) != ESP_OK)
    {
        return NULL;
    }

    cJSON *status = cJSON_CreateObject();
    if (!status)
    {
        return NULL;
    }
    cJSON_AddBoolToObject(status, "initialized", initialized);
    cJSON_AddBoolToObject(status, "reset", tuner_get_reset());
    cJSON_AddStringToObject(status, "polarization", polarization_name());
    cJSON_AddNumberToObject(status, "flags", tuner_status.flags);
    cJSON_AddBoolToObject(status, "signal", (tuner_status.flags & TUNER_STATUS_SIGNAL) != 0);
    cJSON_AddBoolToObject(status, "carrier", (tuner_status.flags & TUNER_STATUS_CARRIER) != 0);
    cJSON_AddBoolToObject(status, "viterbi", (tuner_status.flags & TUNER_STATUS_VITERBI) != 0);
    cJSON_AddBoolToObject(status, "sync", (tuner_status.flags & TUNER_STATUS_SYNC) != 0);
    cJSON_AddBoolToObject(status, "lock", (tuner_status.flags & TUNER_STATUS_LOCK) != 0);
    add_optional_number(status, "strength", tuner_status.strength_available, tuner_status.strength);
    add_optional_number(status, "snr", tuner_status.snr_available, tuner_status.snr);
    add_optional_number(status, "ber", tuner_status.ber_available, tuner_status.ber);
    add_optional_number(status, "uncorrected_blocks", tuner_status.uncorrected_blocks_available,
                        tuner_status.uncorrected_blocks);
    add_optional_number(status, "esno_db", tuner_status.esno_available,
                        tuner_status.esno_tenth_db / 10.0);
    if (tuner_status.acquired_mode_available)
    {
        cJSON_AddStringToObject(status, "acquired_delivery", delivery_name(tuner_status.acquired_delivery_system));
        cJSON_AddStringToObject(status, "acquired_modulation", modulation_name(tuner_status.acquired_modulation));
        cJSON_AddStringToObject(status, "acquired_fec", fec_name(tuner_status.acquired_fec));
        cJSON_AddStringToObject(status, "acquired_pilot", tuner_status.acquired_pilot_on ? "ON" : "OFF");
    }
    else
    {
        cJSON_AddNullToObject(status, "acquired_delivery");
        cJSON_AddNullToObject(status, "acquired_modulation");
        cJSON_AddNullToObject(status, "acquired_fec");
        cJSON_AddNullToObject(status, "acquired_pilot");
    }
    add_optional_bool(status, "lnb_ready", tuner_status.lnb_ready_available, tuner_status.lnb_ready);
    add_optional_bool(status, "firmware_running", tuner_status.firmware_running_available,
                      tuner_status.firmware_running);

    i2s_get_fpga_statistics(&fpga_statistics);
    cJSON *fpga = cJSON_AddObjectToObject(status, "fpga_statistics");
    add_optional_number(fpga, "sync_count", fpga_statistics.available, fpga_statistics.sync_count);
    add_optional_number(fpga, "data_bytes_received", fpga_statistics.available,
                        fpga_statistics.data_bytes_received);
    add_optional_number(fpga, "sync_byte_errors", fpga_statistics.available,
                        fpga_statistics.sync_byte_errors);
    add_optional_number(fpga, "filtered_drops", fpga_statistics.available,
                        fpga_statistics.filtered_drops);

    i2s_get_fpga_pid_report(&pid_report);
    cJSON *pid_body = cJSON_AddObjectToObject(status, "fpga_pid_report");
    add_optional_bool(pid_body, "overflow", pid_report.available, pid_report.overflow);
    cJSON *pids = cJSON_AddArrayToObject(pid_body, "pids");
    if (pid_report.available)
    {
        for (uint8_t index = 0; index < pid_report.count; index++)
        {
            cJSON_AddItemToArray(pids, cJSON_CreateNumber(pid_report.pids[index]));
        }
    }

    i2s_get_fpga_pat(&pat);
    cJSON *pat_body = cJSON_AddObjectToObject(status, "fpga_pat");
    if (pat.available)
    {
        char hex[I2S_FPGA_PAT_SIZE * 2 + 1];
        packet_to_hex(pat.data, hex);
        cJSON_AddNumberToObject(pat_body, "length", I2S_FPGA_PAT_SIZE);
        cJSON_AddStringToObject(pat_body, "hex", hex);
    }
    else
    {
        cJSON_AddNullToObject(pat_body, "length");
        cJSON_AddNullToObject(pat_body, "hex");
    }

    i2s_get_retained_pid_buffers(buffers);
    cJSON *retained = cJSON_AddArrayToObject(status, "retained_pid_buffers");
    for (size_t index = 0; index < I2S_RETAINED_PID_SLOT_COUNT; index++)
    {
        if (!buffers[index].available)
        {
            continue;
        }
        cJSON *buffer = cJSON_CreateObject();
        cJSON_AddNumberToObject(buffer, "pid", buffers[index].pid);
        cJSON_AddNumberToObject(buffer, "retained_packets", buffers[index].packet_count);
        cJSON_AddNumberToObject(buffer, "packets_received", buffers[index].packets_received);
        cJSON_AddNumberToObject(buffer, "continuity_errors", buffers[index].continuity_errors);
        cJSON_AddItemToArray(retained, buffer);
    }

    cJSON *heap = cJSON_AddObjectToObject(status, "heap");
    if (heap)
    {
        cJSON_AddNumberToObject(heap, "free_internal",
                                heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        cJSON_AddNumberToObject(heap, "min_free_internal",
                                heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
        cJSON_AddNumberToObject(heap, "largest_block_internal",
                                heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        cJSON_AddNumberToObject(heap, "free_spiram",
                                heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        cJSON_AddNumberToObject(heap, "min_free_spiram",
                                heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM));
        cJSON_AddNumberToObject(heap, "largest_block_spiram",
                                heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
        cJSON_AddNumberToObject(heap, "uptime_s", esp_timer_get_time() / 1000000);
        cJSON_AddNumberToObject(heap, "min_task_stack", uxTaskGetStackHighWaterMark(NULL));
    }
    return status;
}

static cJSON *build_filters(void)
{
    ts_stream_status_t stream_status = {0};
    ts_stream_get_status(&stream_status);
    cJSON *result = cJSON_CreateObject();
    cJSON *filters = result ? cJSON_AddArrayToObject(result, "filters") : NULL;
    if (!result || !filters)
    {
        cJSON_Delete(result);
        return NULL;
    }
    cJSON_AddNumberToObject(result, "udp_port", TS_UDP_PORT);
    cJSON_AddBoolToObject(result, "udp_enabled", stream_status.udp_enabled);
    cJSON_AddNumberToObject(result, "udp_max_rate_kib_s", stream_status.udp_max_rate_kib_s);
    cJSON_AddNumberToObject(result, "udp_throttle_drops", stream_status.udp_throttle_drop_count);
    for (size_t index = 0; index < FPGA_PID_FILTER_COUNT; index++)
    {
        const ts_filter_status_t *source = &stream_status.filters[index];
        cJSON *filter = cJSON_CreateObject();
        cJSON_AddBoolToObject(filter, "enabled", source->config.enabled);
        cJSON_AddNumberToObject(filter, "pid", source->config.pid);
        cJSON_AddNumberToObject(filter, "mask", source->config.mask);
        cJSON_AddNumberToObject(filter, "packet_count", source->packet_count);
        cJSON_AddNumberToObject(filter, "byte_count", source->byte_count);
        cJSON_AddNumberToObject(filter, "packets_per_second", source->packets_per_second);
        cJSON_AddNumberToObject(filter, "bytes_per_second", source->bytes_per_second);
        cJSON_AddNullToObject(filter, "latest_packet");
        cJSON_AddItemToArray(filters, filter);
    }
    return result;
}

static cJSON *build_retained_pid(uint16_t pid)
{
    uint8_t *packets = malloc(I2S_RETAINED_PID_PACKET_CAPACITY * TS_PACKET_SIZE);
    if (!packets)
    {
        return NULL;
    }
    size_t count = i2s_copy_retained_pid_packets(pid, packets, I2S_RETAINED_PID_PACKET_CAPACITY);
    cJSON *result = cJSON_CreateObject();
    cJSON *list = result ? cJSON_AddArrayToObject(result, "packets") : NULL;
    if (!result || !list)
    {
        free(packets);
        cJSON_Delete(result);
        return NULL;
    }
    cJSON_AddNumberToObject(result, "pid", pid);
    cJSON_AddNumberToObject(result, "retained_packets", count);
    for (size_t index = 0; index < count; index++)
    {
        char hex[TS_PACKET_SIZE * 2 + 1];
        packet_to_hex(packets + index * TS_PACKET_SIZE, hex);
        cJSON_AddItemToArray(list, cJSON_CreateString(hex));
    }
    free(packets);
    return result;
}

static bool parse_filters(const cJSON *root, fpga_pid_filter_t filters[FPGA_PID_FILTER_COUNT])
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

static void handle_command(const char *text)
{
    cJSON *command = cJSON_Parse(text);
    const cJSON *id = command ? cJSON_GetObjectItemCaseSensitive(command, "id") : NULL;
    const cJSON *path = command ? cJSON_GetObjectItemCaseSensitive(command, "path") : NULL;
    const cJSON *method = command ? cJSON_GetObjectItemCaseSensitive(command, "method") : NULL;
    const cJSON *body = command ? cJSON_GetObjectItemCaseSensitive(command, "body") : NULL;
    uint32_t request_id = cJSON_IsNumber(id) ? id->valueint : 0;
    if (!cJSON_IsString(path) || !cJSON_IsString(method))
    {
        serial_send_response(request_id, false, NULL, "invalid_command");
        cJSON_Delete(command);
        return;
    }

    if (strcmp(path->valuestring, "") == 0 && strcmp(method->valuestring, "GET") == 0)
    {
        cJSON *status = build_status();
        serial_send_response(request_id, status != NULL, status, status ? NULL : "status_read_failed");
    }
    else if (strcmp(path->valuestring, "/filters") == 0 && strcmp(method->valuestring, "GET") == 0)
    {
        cJSON *filters = build_filters();
        serial_send_response(request_id, filters != NULL, filters, filters ? NULL : "status_read_failed");
    }
    else if (strncmp(path->valuestring, "/retained-pid?pid=", 18) == 0 &&
             strcmp(method->valuestring, "GET") == 0)
    {
        char *end = NULL;
        unsigned long pid = strtoul(path->valuestring + 18, &end, 0);
        cJSON *retained = end && *end == '\0' && pid <= 0x1fff ? build_retained_pid(pid) : NULL;
        serial_send_response(request_id, retained != NULL, retained, retained ? NULL : "invalid_pid");
    }
    else if (strcmp(path->valuestring, "/init") == 0 && strcmp(method->valuestring, "POST") == 0)
    {
        esp_err_t result = tuner_initialize();
        serial_send_response(request_id, result == ESP_OK, cJSON_CreateObject(),
                             result == ESP_OK ? NULL : "initialization_failed");
    }
    else if (strcmp(path->valuestring, "/reset") == 0 && strcmp(method->valuestring, "POST") == 0)
    {
        const cJSON *active = body ? cJSON_GetObjectItemCaseSensitive(body, "active") : NULL;
        if (!cJSON_IsBool(active))
        {
            serial_send_response(request_id, false, NULL, "invalid_reset_request");
        }
        else
        {
            tuner_set_reset(cJSON_IsTrue(active));
            serial_send_response(request_id, true, cJSON_CreateObject(), NULL);
        }
    }
    else if (strcmp(path->valuestring, "/tune") == 0 && strcmp(method->valuestring, "POST") == 0)
    {
        tuner_tune_request_t request;
        if (!tuner_is_initialized())
        {
            serial_send_response(request_id, false, NULL, "not_initialized");
        }
        else if (!body || !channel_config_parse_tune_request(body, &request))
        {
            serial_send_response(request_id, false, NULL, "invalid_tune_request");
        }
        else
        {
            esp_err_t result = tuner_tune(&request);
            serial_send_response(request_id, result == ESP_OK, cJSON_CreateObject(),
                                 result == ESP_OK ? NULL : "tune_failed");
        }
    }
    else if (strcmp(path->valuestring, "/filters") == 0 && strcmp(method->valuestring, "POST") == 0)
    {
        fpga_pid_filter_t filters[FPGA_PID_FILTER_COUNT] = {0};
        if (!body || !parse_filters(body, filters))
        {
            serial_send_response(request_id, false, NULL, "invalid_filter_request");
        }
        else
        {
            esp_err_t result = ts_stream_configure(filters);
            serial_send_response(request_id, result == ESP_OK, cJSON_CreateObject(),
                                 result == ESP_OK ? NULL : "fpga_filter_write_failed");
        }
    }
    else if (strcmp(path->valuestring, "/udp") == 0 && strcmp(method->valuestring, "POST") == 0)
    {
        const cJSON *enabled = body ? cJSON_GetObjectItemCaseSensitive(body, "enabled") : NULL;
        const cJSON *rate = body ? cJSON_GetObjectItemCaseSensitive(body, "max_rate_kib_s") : NULL;
        if (!cJSON_IsBool(enabled) ||
            (rate && (!cJSON_IsNumber(rate) || rate->valuedouble < 0 || rate->valuedouble > 100000)))
        {
            serial_send_response(request_id, false, NULL, "invalid_udp_request");
        }
        else
        {
            ts_stream_set_udp(cJSON_IsTrue(enabled), rate ? (uint32_t)rate->valuedouble : 0);
            serial_send_response(request_id, true, cJSON_CreateObject(), NULL);
        }
    }
    else if (strcmp(path->valuestring, "/stream") == 0 && strcmp(method->valuestring, "POST") == 0)
    {
        const cJSON *enabled = body ? cJSON_GetObjectItemCaseSensitive(body, "enabled") : NULL;
        if (!cJSON_IsBool(enabled))
        {
            serial_send_response(request_id, false, NULL, "invalid_stream_request");
        }
        else
        {
            if (!cJSON_IsTrue(enabled) && stream_enabled)
            {
                serial_send_response(request_id, false, NULL, "stream_mode_latched");
            }
            else
            {
                stream_enabled = cJSON_IsTrue(enabled);
                if (stream_enabled)
                {
                    serial_protocol_mute_logs();
                }
                serial_send_response(request_id, true, cJSON_CreateObject(), NULL);
            }
        }
    }
    else
    {
        serial_send_response(request_id, false, NULL, "unsupported_request");
    }
    cJSON_Delete(command);
}

static void serial_protocol_task(void *arg)
{
    (void)arg;
    uint8_t header[5] = {0};
    uint8_t header_index = 0;
    uint8_t payload[SERIAL_MAX_COMMAND_SIZE + 1];
    size_t payload_index = 0;
    uint16_t payload_length = 0;
    bool reading_payload = false;

    while (true)
    {
        uint8_t byte;
        if (esp_usb_console_read_buf((char *)&byte, 1) != 1)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (!reading_payload)
        {
            if (header_index == 0 && byte != SERIAL_MAGIC_FIRST)
            {
                continue;
            }
            if (header_index == 1 && byte != SERIAL_MAGIC_SECOND)
            {
                header_index = byte == SERIAL_MAGIC_FIRST ? 1 : 0;
                continue;
            }
            header[header_index++] = byte;
            if (header_index != sizeof(header))
            {
                continue;
            }
            payload_length = (uint16_t)header[3] | ((uint16_t)header[4] << 8);
            if (header[2] != SERIAL_PROTOCOL_TYPE_COMMAND || payload_length == 0 ||
                payload_length > SERIAL_MAX_COMMAND_SIZE)
            {
                header_index = 0;
                continue;
            }
            payload_index = 0;
            reading_payload = true;
            continue;
        }

        payload[payload_index++] = byte;
        if (payload_index == payload_length)
        {
            payload[payload_length] = '\0';
            handle_command((const char *)payload);
            header_index = 0;
            reading_payload = false;
        }
    }
}

esp_err_t serial_protocol_start(void)
{
    if (serial_write_lock)
    {
        return ESP_ERR_INVALID_STATE;
    }
    serial_write_lock = xSemaphoreCreateMutex();
    if (!serial_write_lock)
    {
        return ESP_ERR_NO_MEM;
    }
    serial_tx_queue = xQueueCreate(SERIAL_TX_QUEUE_LENGTH, sizeof(serial_tx_item_t));
    if (!serial_tx_queue)
    {
        vSemaphoreDelete(serial_write_lock);
        serial_write_lock = NULL;
        return ESP_ERR_NO_MEM;
    }
    BaseType_t tx_created = xTaskCreate(serial_tx_task, "serial_tx",
                                        SERIAL_TASK_STACK_SIZE, NULL,
                                        SERIAL_TASK_PRIORITY, NULL);
    if (tx_created != pdPASS)
    {
        vQueueDelete(serial_tx_queue);
        serial_tx_queue = NULL;
        vSemaphoreDelete(serial_write_lock);
        serial_write_lock = NULL;
        return ESP_ERR_NO_MEM;
    }
    BaseType_t created = xTaskCreate(serial_protocol_task, "serial_protocol",
                                     SERIAL_TASK_STACK_SIZE, NULL,
                                     SERIAL_TASK_PRIORITY, NULL);
    return created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void serial_protocol_send_fpga_frame(uint8_t type, const uint8_t *payload, uint8_t length)
{
    if (!stream_enabled || !payload)
    {
        return;
    }
    serial_tx_item_t item = {
        .length = length + 2,
        .payload = {type, length},
    };
    memcpy(item.payload + 2, payload, length);
    xQueueSend(serial_tx_queue, &item, 0);
}

void serial_protocol_enable_streaming(void)
{
    stream_enabled = true;
    serial_protocol_mute_logs();
}

bool serial_protocol_streaming(void)
{
    return stream_enabled;
}
