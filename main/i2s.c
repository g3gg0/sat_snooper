/* I2S slave driver for receiving framed data from the FPGA. */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2s.h"
#include "esp_log.h"
#include "gpio_config.h" /* Provides GPIO definitions like GPIO_I2S_WS, etc. */
#include "serial_protocol.h"
#include "ts_stream.h"

/* I2S peripheral number */
#define I2S_PORT_NUM I2S_NUM_0
#define I2S_SAMPLE_RATE_HZ 500000
#define I2S_DMA_FRAMES_PER_BUFFER 188
#define I2S_DMA_BUF_COUNT 4
#define FPGA_FRAME_TYPE_STATISTICS 0
#define FPGA_FRAME_TYPE_PID_REPORT 1
#define FPGA_FRAME_TYPE_PAT 2
#define FPGA_FRAME_TYPE_FILTERED_TS 3
#define FPGA_STATISTICS_PAYLOAD_SIZE 14
#define FPGA_PID_REPORT_MAX_PAYLOAD_SIZE (1 + I2S_FPGA_MAX_PIDS * 2)
#define FPGA_FRAME_MAX_PAYLOAD_SIZE 255
#define I2S_DIAGNOSTIC_INTERVAL_MS 1000
#define I2S_READ_TIMEOUT_MS 250
#define I2S_DIAGNOSTIC_SAMPLE_SIZE 16

static const char *TAG = "I2S_SLAVE";
static i2s_chan_handle_t rx_chan; /* Static handle for the I2S RX channel */
static portMUX_TYPE statistics_lock = portMUX_INITIALIZER_UNLOCKED;
static i2s_fpga_statistics_t fpga_statistics;
static i2s_fpga_pid_report_t fpga_pid_report;
static i2s_fpga_pat_t fpga_pat;

typedef struct {
    bool available;
    uint16_t pid;
    uint8_t write_index;
    uint8_t packet_count;
    uint32_t packets_received;
    uint32_t continuity_errors;
    bool continuity_valid;
    uint8_t continuity_counter;
    uint32_t last_use;
} control_pid_ring_t;

static control_pid_ring_t control_pid_rings[I2S_RETAINED_PID_SLOT_COUNT];
static uint32_t control_pid_use_counter;
static uint8_t *control_pid_packet_storage;

typedef enum {
    FRAME_READ_TYPE,
    FRAME_READ_LENGTH,
    FRAME_READ_PAYLOAD,
} frame_parser_state_t;

typedef struct {
    frame_parser_state_t state;
    uint8_t type;
    uint8_t length;
    uint8_t payload_index;
    uint8_t payload[FPGA_FRAME_MAX_PAYLOAD_SIZE];
} frame_parser_t;

typedef struct {
    uint32_t read_count;
    uint32_t timeout_count;
    uint32_t byte_count;
    uint32_t non_idle_byte_count;
    uint32_t zero_byte_count;
    uint32_t length_byte_count;
    uint32_t header_candidate_count;
    uint32_t rejected_length_count;
    uint32_t statistics_frame_count;
    uint32_t pid_report_frame_count;
    uint32_t pat_frame_count;
    uint32_t filtered_ts_frame_count;
    uint8_t non_idle_sample[I2S_DIAGNOSTIC_SAMPLE_SIZE];
    uint8_t non_idle_sample_size;
} i2s_diagnostics_t;

static uint32_t read_le32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static void store_statistics_frame(const uint8_t *payload)
{
    portENTER_CRITICAL(&statistics_lock);
    fpga_statistics.sync_count = read_le32(payload);
    fpga_statistics.data_bytes_received = read_le32(payload + 4);
    fpga_statistics.sync_byte_errors = read_le32(payload + 8);
    fpga_statistics.filtered_drops = (uint16_t)payload[12] | ((uint16_t)payload[13] << 8);
    fpga_statistics.available = true;
    portEXIT_CRITICAL(&statistics_lock);
}

static void store_pid_report_frame(const uint8_t *payload, uint8_t length)
{
    uint8_t count = (length - 1) / 2;
    char pid_line[320];
    size_t offset = 0;

    portENTER_CRITICAL(&statistics_lock);
    fpga_pid_report.overflow = payload[0] != 0;
    fpga_pid_report.count = count;
    for (uint8_t index = 0; index < count; index++)
    {
        fpga_pid_report.pids[index] = (uint16_t)payload[1 + index * 2] |
                                      ((uint16_t)(payload[2 + index * 2] & 0x1f) << 8);
    }
    fpga_pid_report.available = true;
    portEXIT_CRITICAL(&statistics_lock);

    if(0)
    {
        offset += snprintf(pid_line + offset, sizeof(pid_line) - offset,
                        "Seen PIDs%s:", payload[0] ? " (truncated)" : "");
        for (uint8_t index = 0; index < count && offset < sizeof(pid_line); index++)
        {
            offset += snprintf(pid_line + offset, sizeof(pid_line) - offset,
                            " %04x", (unsigned int)fpga_pid_report.pids[index]);
        }
        ESP_LOGD(TAG, "%s", pid_line);
    }
}

static void store_pat_frame(const uint8_t *payload)
{
    portENTER_CRITICAL(&statistics_lock);
    memcpy(fpga_pat.data, payload, I2S_FPGA_PAT_SIZE);
    fpga_pat.available = true;
    portEXIT_CRITICAL(&statistics_lock);
}

static void retain_control_pid_packet(const uint8_t packet[TS_PACKET_SIZE])
{
    uint16_t pid = ((uint16_t)(packet[1] & 0x1f) << 8) | packet[2];
    if (!control_pid_packet_storage)
    {
        return;
    }

    uint8_t adaptation_control = (packet[3] >> 4) & 0x03;
    if (adaptation_control == 0)
    {
        return;
    }

    portENTER_CRITICAL(&statistics_lock);
    control_pid_ring_t *ring = NULL;
    control_pid_ring_t *free_ring = NULL;
    control_pid_ring_t *oldest_ring = &control_pid_rings[0];
    for (size_t index = 0; index < I2S_RETAINED_PID_SLOT_COUNT; index++)
    {
        if (control_pid_rings[index].available && control_pid_rings[index].pid == pid)
        {
            ring = &control_pid_rings[index];
            break;
        }
        if (!free_ring && !control_pid_rings[index].available)
        {
            free_ring = &control_pid_rings[index];
        }
        if (control_pid_rings[index].last_use < oldest_ring->last_use)
        {
            oldest_ring = &control_pid_rings[index];
        }
    }
    /* Slots are never freed, so recycle the stalest one for a newly filtered PID */
    if (!ring)
    {
        ring = free_ring ? free_ring : oldest_ring;
    }
    if (!ring->available || ring->pid != pid)
    {
        memset(ring, 0, sizeof(*ring));
        ring->available = true;
        ring->pid = pid;
    }
    ring->last_use = ++control_pid_use_counter;
    if (adaptation_control & 0x01)
    {
        uint8_t continuity_counter = packet[3] & 0x0f;
        if (ring->continuity_valid && continuity_counter != ring->continuity_counter &&
            continuity_counter != ((ring->continuity_counter + 1) & 0x0f))
        {
            ring->continuity_errors++;
        }
        ring->continuity_counter = continuity_counter;
        ring->continuity_valid = true;
    }
    size_t ring_index = ring - control_pid_rings;
    size_t packet_offset = (ring_index * I2S_RETAINED_PID_PACKET_CAPACITY +
                            ring->write_index) * TS_PACKET_SIZE;
    memcpy(control_pid_packet_storage + packet_offset, packet, TS_PACKET_SIZE);
    ring->write_index = (ring->write_index + 1) % I2S_RETAINED_PID_PACKET_CAPACITY;
    if (ring->packet_count < I2S_RETAINED_PID_PACKET_CAPACITY)
    {
        ring->packet_count++;
    }
    ring->packets_received++;
    portEXIT_CRITICAL(&statistics_lock);
}

static bool frame_length_is_valid(uint8_t type, uint8_t length)
{
    switch (type)
    {
    case FPGA_FRAME_TYPE_STATISTICS:
        return length == FPGA_STATISTICS_PAYLOAD_SIZE;
    case FPGA_FRAME_TYPE_PID_REPORT:
        return length >= 1 && length <= FPGA_PID_REPORT_MAX_PAYLOAD_SIZE &&
               ((length - 1) % 2) == 0;
    case FPGA_FRAME_TYPE_PAT:
        return length == I2S_FPGA_PAT_SIZE;
    case FPGA_FRAME_TYPE_FILTERED_TS:
        return length == TS_PACKET_SIZE;
    default:
        return false;
    }
}

static void parse_frame_byte(frame_parser_t *parser, i2s_diagnostics_t *diagnostics, uint8_t byte)
{
    switch (parser->state)
    {
    case FRAME_READ_TYPE:
        if (byte <= FPGA_FRAME_TYPE_FILTERED_TS)
        {
            diagnostics->header_candidate_count++;
            parser->type = byte;
            parser->state = FRAME_READ_LENGTH;
        }
        break;
    case FRAME_READ_LENGTH:
        if (frame_length_is_valid(parser->type, byte))
        {
            parser->length = byte;
            parser->payload_index = 0;
            parser->state = FRAME_READ_PAYLOAD;
        }
        else
        {
            diagnostics->rejected_length_count++;
            parser->state = FRAME_READ_TYPE;
        }
        break;
    case FRAME_READ_PAYLOAD:
        parser->payload[parser->payload_index] = byte;
        parser->payload_index++;
        if (parser->payload_index == parser->length)
        {
            switch (parser->type)
            {
            case FPGA_FRAME_TYPE_STATISTICS:
                store_statistics_frame(parser->payload);
                diagnostics->statistics_frame_count++;
                break;
            case FPGA_FRAME_TYPE_PID_REPORT:
                store_pid_report_frame(parser->payload, parser->length);
                diagnostics->pid_report_frame_count++;
                break;
            case FPGA_FRAME_TYPE_PAT:
                store_pat_frame(parser->payload);
                diagnostics->pat_frame_count++;
                ESP_LOGI(TAG, "Received PAT packet");
                break;
            case FPGA_FRAME_TYPE_FILTERED_TS:
                retain_control_pid_packet(parser->payload);
                ts_stream_submit_packet(parser->payload);
                diagnostics->filtered_ts_frame_count++;
                break;
            }
            serial_protocol_send_fpga_frame(parser->type, parser->payload,
                                            parser->length);
            parser->state = FRAME_READ_TYPE;
        }
        break;
    }
}

static void record_diagnostic_byte(i2s_diagnostics_t *diagnostics, uint8_t byte)
{
    diagnostics->byte_count++;
    if (byte == 0xff)
    {
        return;
    }

    diagnostics->non_idle_byte_count++;
    if (byte == FPGA_FRAME_TYPE_STATISTICS)
    {
        diagnostics->zero_byte_count++;
    }
    if (byte == FPGA_STATISTICS_PAYLOAD_SIZE)
    {
        diagnostics->length_byte_count++;
    }
    if (diagnostics->non_idle_sample_size < I2S_DIAGNOSTIC_SAMPLE_SIZE)
    {
        diagnostics->non_idle_sample[diagnostics->non_idle_sample_size++] = byte;
    }
}

static void log_i2s_diagnostics(const frame_parser_t *parser, i2s_diagnostics_t *diagnostics)
{
    i2s_fpga_statistics_t statistics;
    i2s_get_fpga_statistics(&statistics);

    ESP_LOGD(TAG,
             "RX reads=%u timeouts=%u bytes=%u non-idle=%u zero=%u len12=%u parser=%u candidates=%u rejected=%u frames=%u/%u/%u/%u",
             (unsigned int)diagnostics->read_count,
             (unsigned int)diagnostics->timeout_count,
             (unsigned int)diagnostics->byte_count,
             (unsigned int)diagnostics->non_idle_byte_count,
             (unsigned int)diagnostics->zero_byte_count,
             (unsigned int)diagnostics->length_byte_count,
             (unsigned int)parser->state,
             (unsigned int)diagnostics->header_candidate_count,
             (unsigned int)diagnostics->rejected_length_count,
             (unsigned int)diagnostics->statistics_frame_count,
             (unsigned int)diagnostics->pid_report_frame_count,
             (unsigned int)diagnostics->pat_frame_count,
             (unsigned int)diagnostics->filtered_ts_frame_count);

    if (diagnostics->non_idle_sample_size > 0)
    {
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, diagnostics->non_idle_sample,
                                 diagnostics->non_idle_sample_size, ESP_LOG_DEBUG);
    }

    if (statistics.available)
    {
        ESP_LOGD(TAG, "Latest FPGA statistics: sync=%u data-bytes=%u sync-errors=%u drops=%u",
                 (unsigned int)statistics.sync_count,
                 (unsigned int)statistics.data_bytes_received,
                 (unsigned int)statistics.sync_byte_errors,
                 (unsigned int)statistics.filtered_drops);
    }
    else
    {
        ESP_LOGW(TAG, "No complete FPGA statistics frame received yet");
    }

    memset(diagnostics, 0, sizeof(*diagnostics));
}

void i2s_get_fpga_statistics(i2s_fpga_statistics_t *statistics)
{
    if (!statistics)
    {
        return;
    }

    portENTER_CRITICAL(&statistics_lock);
    *statistics = fpga_statistics;
    portEXIT_CRITICAL(&statistics_lock);
}

void i2s_get_fpga_pid_report(i2s_fpga_pid_report_t *report)
{
    if (!report)
    {
        return;
    }

    portENTER_CRITICAL(&statistics_lock);
    *report = fpga_pid_report;
    portEXIT_CRITICAL(&statistics_lock);
}

void i2s_get_fpga_pat(i2s_fpga_pat_t *pat)
{
    if (!pat)
    {
        return;
    }

    portENTER_CRITICAL(&statistics_lock);
    *pat = fpga_pat;
    portEXIT_CRITICAL(&statistics_lock);
}

void i2s_get_retained_pid_buffers(i2s_retained_pid_buffer_t buffers[I2S_RETAINED_PID_SLOT_COUNT])
{
    if (!buffers)
    {
        return;
    }

    portENTER_CRITICAL(&statistics_lock);
    for (size_t index = 0; index < I2S_RETAINED_PID_SLOT_COUNT; index++)
    {
        buffers[index].available = control_pid_rings[index].available;
        buffers[index].pid = control_pid_rings[index].pid;
        buffers[index].packet_count = control_pid_rings[index].packet_count;
        buffers[index].packets_received = control_pid_rings[index].packets_received;
        buffers[index].continuity_errors = control_pid_rings[index].continuity_errors;
    }
    portEXIT_CRITICAL(&statistics_lock);
}

size_t i2s_copy_retained_pid_packets(uint16_t pid, uint8_t *packets, size_t capacity)
{
    if (!packets || !control_pid_packet_storage)
    {
        return 0;
    }

    portENTER_CRITICAL(&statistics_lock);
    const control_pid_ring_t *ring = NULL;
    size_t ring_index = 0;
    for (; ring_index < I2S_RETAINED_PID_SLOT_COUNT; ring_index++)
    {
        if (control_pid_rings[ring_index].available && control_pid_rings[ring_index].pid == pid)
        {
            ring = &control_pid_rings[ring_index];
            break;
        }
    }
    if (!ring)
    {
        portEXIT_CRITICAL(&statistics_lock);
        return 0;
    }
    size_t packet_count = ring->packet_count;
    if (capacity < packet_count)
    {
        portEXIT_CRITICAL(&statistics_lock);
        return 0;
    }

    uint8_t oldest_index = packet_count == I2S_RETAINED_PID_PACKET_CAPACITY
                               ? ring->write_index
                               : 0;
    for (size_t index = 0; index < packet_count; index++)
    {
        size_t source_offset = (ring_index * I2S_RETAINED_PID_PACKET_CAPACITY +
                    ((oldest_index + index) % I2S_RETAINED_PID_PACKET_CAPACITY)) *
                               TS_PACKET_SIZE;
        memcpy(packets + index * TS_PACKET_SIZE,
               control_pid_packet_storage + source_offset, TS_PACKET_SIZE);
    }
    portEXIT_CRITICAL(&statistics_lock);
    return packet_count;
}

/**
 * @brief Initializes the I2S peripheral in slave RX mode using the standard driver API.
 *
 * @return esp_err_t ESP_OK on success, or an error code from the I2S driver on failure.
 */
esp_err_t i2s_init(void)
{
    control_pid_packet_storage = heap_caps_malloc(
        I2S_RETAINED_PID_SLOT_COUNT * I2S_RETAINED_PID_PACKET_CAPACITY * TS_PACKET_SIZE,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!control_pid_packet_storage)
    {
        ESP_LOGE(TAG, "Failed to allocate control PID packet storage in PSRAM");
        return ESP_ERR_NO_MEM;
    }
    memset(control_pid_packet_storage, 0,
            I2S_RETAINED_PID_SLOT_COUNT * I2S_RETAINED_PID_PACKET_CAPACITY * TS_PACKET_SIZE);

    gpio_set_direction(GPIO_I2S_BCK, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_I2S_WS, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_I2S_DAT, GPIO_MODE_INPUT);

    /* RX channel configuration */
    i2s_chan_config_t rx_chan_cfg = {
        .id = I2S_PORT_NUM,
        .role = I2S_ROLE_SLAVE,
        .dma_desc_num = I2S_DMA_BUF_COUNT,
        .dma_frame_num = I2S_DMA_FRAMES_PER_BUFFER,
        .auto_clear = false, /* Not applicable for RX */
    };

    esp_err_t err;

    /* Allocate a new RX channel */
    err = i2s_new_channel(&rx_chan_cfg, NULL, &rx_chan); /* No TX channel, so tx_handle is NULL */
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2S new channel failed: %s", esp_err_to_name(err));
        rx_chan = NULL; /* Ensure handle is NULL on failure */
        return err;
    }

    /* Standard mode configuration for the RX channel */
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_8BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_I2S_BCK,
            .ws = GPIO_I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = GPIO_I2S_DAT,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    /* Initialize the channel for standard mode */
    err = i2s_channel_init_std_mode(rx_chan, &std_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2S channel init std mode failed: %s", esp_err_to_name(err));
        i2s_del_channel(rx_chan);
        rx_chan = NULL;
        return err;
    }

    /* Enable the RX channel */
    err = i2s_channel_enable(rx_chan);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2S channel enable failed: %s", esp_err_to_name(err));
        /* Attempt to disable before deleting, though it might not be strictly necessary if enable failed */
        i2s_channel_disable(rx_chan);
        i2s_del_channel(rx_chan);
        rx_chan = NULL;
        return err;
    }

    ESP_LOGI(TAG, "I2S slave driver initialized successfully using STD API on port %d.", (int)I2S_PORT_NUM);
    ESP_LOGI(TAG, "Pins: WS=%d, BCK=%d, DAT_IN=%d", (int)GPIO_I2S_WS, (int)GPIO_I2S_BCK, (int)GPIO_I2S_DAT);
    ESP_LOGI(TAG, "Expected timing: frame-rate=%u Hz, BCK=%u Hz",
             (unsigned int)I2S_SAMPLE_RATE_HZ,
             (unsigned int)(I2S_SAMPLE_RATE_HZ * 2 * 8));
    ESP_LOGI(TAG, "DMA Buffers: %d x %d frames.", I2S_DMA_BUF_COUNT, I2S_DMA_FRAMES_PER_BUFFER);
    ESP_LOGI(TAG, "Each frame: %d bytes (stereo 8-bit). Total DMA buffer size: %d bytes.",
             (int)(I2S_SLOT_MODE_STEREO * (I2S_DATA_BIT_WIDTH_8BIT / 8)),
             (int)(I2S_DMA_FRAMES_PER_BUFFER * I2S_SLOT_MODE_STEREO * (I2S_DATA_BIT_WIDTH_8BIT / 8)));
    ESP_LOGI(TAG, "Effectively %d bytes per WS phase.", (int)(I2S_DMA_FRAMES_PER_BUFFER * (I2S_DATA_BIT_WIDTH_8BIT / 8)));

    xTaskCreate(i2s_read_task,       /* Task function */
                "i2s_read_task",     /* Name of task */
                I2S_TASK_STACK_SIZE, /* Stack size of task */
                NULL,                /* Parameter of the task */
                I2S_TASK_PRIORITY,   /* Priority of the task */
                NULL);               /* Task handle (NULL if not needed) */

    return ESP_OK;
}

#define I2S_READ_BUF_SIZE 188

DRAM_ATTR uint8_t i2s_rx_buffer[I2S_READ_BUF_SIZE];
size_t bytes_read;

void i2s_read_task(void *arg)
{
    frame_parser_t parser = {0};
    i2s_diagnostics_t diagnostics = {0};
    TickType_t next_diagnostic_tick = xTaskGetTickCount() + pdMS_TO_TICKS(I2S_DIAGNOSTIC_INTERVAL_MS);

    if (!rx_chan)
    {
        ESP_LOGE(TAG, "I2S RX channel not initialized for read task.");
        vTaskDelete(NULL);
        return;
    }
    while (1)
    {
        esp_err_t read_err = i2s_channel_read(rx_chan, i2s_rx_buffer, I2S_READ_BUF_SIZE,
                              &bytes_read, I2S_READ_TIMEOUT_MS);

        if (read_err == ESP_OK)
        {
            diagnostics.read_count++;
            for (size_t index = 0; index < bytes_read; index++)
            {
                record_diagnostic_byte(&diagnostics, i2s_rx_buffer[index]);
                parse_frame_byte(&parser, &diagnostics, i2s_rx_buffer[index]);
            }

        }
        else if (read_err == ESP_ERR_TIMEOUT)
        {
            diagnostics.timeout_count++;
        }
        else
        {
            ESP_LOGE(TAG, "I2S channel read error: %s", esp_err_to_name(read_err));
        }

        if ((int32_t)(xTaskGetTickCount() - next_diagnostic_tick) >= 0)
        {
            log_i2s_diagnostics(&parser, &diagnostics);
            next_diagnostic_tick = xTaskGetTickCount() + pdMS_TO_TICKS(I2S_DIAGNOSTIC_INTERVAL_MS);
        }
    }
}
