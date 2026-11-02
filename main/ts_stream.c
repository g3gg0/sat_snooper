#include "ts_stream.h"

#include <errno.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#define TS_QUEUE_LENGTH 32
#define TS_UDP_PACKETS_PER_DATAGRAM 7
#define TS_UDP_TASK_STACK_SIZE 4096
#define TS_UDP_TASK_PRIORITY 5

typedef struct
{
    uint8_t data[TS_PACKET_SIZE];
} queued_ts_packet_t;

static const char *TAG = "ts_stream";
static QueueHandle_t packet_queue;
static SemaphoreHandle_t transmission_mutex;
static portMUX_TYPE stream_lock = portMUX_INITIALIZER_UNLOCKED;
static ts_stream_status_t stream_status;
static uint32_t previous_packet_count[FPGA_PID_FILTER_COUNT];
static int64_t previous_rate_time_us;
static volatile bool udp_enabled;
static volatile uint32_t udp_max_rate_kib_s;

/* Leaky bucket evaluated per datagram, no refill task */
static bool rate_limit_allows(size_t bytes)
{
    static int64_t bucket_time_us;
    static int64_t bucket_bytes;

    uint32_t rate_kib_s = udp_max_rate_kib_s;
    int64_t now_us = esp_timer_get_time();
    if (rate_kib_s == 0)
    {
        bucket_time_us = now_us;
        bucket_bytes = 0;
        return true;
    }

    int64_t bytes_per_second = (int64_t)rate_kib_s * 1024;
    int64_t capacity = bytes_per_second / 4;
    if (capacity < (int64_t)bytes)
    {
        capacity = bytes;
    }

    int64_t elapsed_us = now_us - bucket_time_us;
    if (elapsed_us < 0 || elapsed_us > 1000000)
    {
        elapsed_us = 1000000;
    }
    bucket_bytes += elapsed_us * bytes_per_second / 1000000;
    bucket_time_us = now_us;
    if (bucket_bytes > capacity)
    {
        bucket_bytes = capacity;
    }

    if (bucket_bytes < (int64_t)bytes)
    {
        return false;
    }
    bucket_bytes -= bytes;
    return true;
}

static int create_broadcast_socket(void)
{
    int socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (socket_fd < 0)
    {
        return -1;
    }

    int enabled = 1;
    if (setsockopt(socket_fd, SOL_SOCKET, SO_BROADCAST, &enabled, sizeof(enabled)) < 0)
    {
        close(socket_fd);
        return -1;
    }
    return socket_fd;
}

static void udp_broadcast_task(void *arg)
{
    (void)arg;
    uint8_t datagram[TS_PACKET_SIZE * TS_UDP_PACKETS_PER_DATAGRAM];
    struct sockaddr_in destination = {
        .sin_family = AF_INET,
        .sin_port = htons(TS_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_BROADCAST),
    };
    int socket_fd = -1;

    while (true)
    {
        queued_ts_packet_t packet;
        if (xQueueReceive(packet_queue, &packet, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        size_t packet_count = 1;
        memcpy(datagram, packet.data, TS_PACKET_SIZE);
        while (packet_count < TS_UDP_PACKETS_PER_DATAGRAM &&
               xQueueReceive(packet_queue, &packet, pdMS_TO_TICKS(2)) == pdTRUE)
        {
            memcpy(datagram + packet_count * TS_PACKET_SIZE, packet.data, TS_PACKET_SIZE);
            packet_count++;
        }

        size_t datagram_size = packet_count * TS_PACKET_SIZE;
        if (!udp_enabled || !rate_limit_allows(datagram_size))
        {
            portENTER_CRITICAL(&stream_lock);
            stream_status.udp_throttle_drop_count += packet_count;
            portEXIT_CRITICAL(&stream_lock);
            continue;
        }

        xSemaphoreTake(transmission_mutex, portMAX_DELAY);
        if (socket_fd < 0)
        {
            socket_fd = create_broadcast_socket();
        }

        ssize_t sent = socket_fd < 0 ? -1 :
            sendto(socket_fd, datagram, datagram_size, 0,
                   (struct sockaddr *)&destination, sizeof(destination));

        portENTER_CRITICAL(&stream_lock);
        if (sent == (ssize_t)datagram_size)
        {
            stream_status.udp_datagram_count++;
            stream_status.udp_packet_count += packet_count;
        }
        else
        {
            stream_status.udp_send_error_count++;
        }
        portEXIT_CRITICAL(&stream_lock);

        if (sent < 0 && socket_fd >= 0)
        {
            ESP_LOGW(TAG, "UDP broadcast failed: errno %d", errno);
            close(socket_fd);
            socket_fd = -1;
        }
        xSemaphoreGive(transmission_mutex);

        if (sent < 0)
        {
            vTaskDelay(pdMS_TO_TICKS(250));
        }
    }
}

esp_err_t ts_stream_init(void)
{
    if (packet_queue)
    {
        return ESP_ERR_INVALID_STATE;
    }

    transmission_mutex = xSemaphoreCreateMutex();
    if (!transmission_mutex)
    {
        return ESP_ERR_NO_MEM;
    }

    packet_queue = xQueueCreate(TS_QUEUE_LENGTH, sizeof(queued_ts_packet_t));
    if (!packet_queue)
    {
        vSemaphoreDelete(transmission_mutex);
        transmission_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    previous_rate_time_us = esp_timer_get_time();
    if (xTaskCreate(udp_broadcast_task, "ts_udp", TS_UDP_TASK_STACK_SIZE,
                    NULL, TS_UDP_TASK_PRIORITY, NULL) != pdPASS)
    {
        vQueueDelete(packet_queue);
        packet_queue = NULL;
        vSemaphoreDelete(transmission_mutex);
        transmission_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "MPEG-TS UDP broadcast ready on port %u (disabled)", TS_UDP_PORT);
    return ESP_OK;
}

esp_err_t ts_stream_set_udp(bool enabled, uint32_t max_rate_kib_s)
{
    udp_max_rate_kib_s = max_rate_kib_s;
    udp_enabled = enabled;
    portENTER_CRITICAL(&stream_lock);
    stream_status.udp_enabled = enabled;
    stream_status.udp_max_rate_kib_s = max_rate_kib_s;
    portEXIT_CRITICAL(&stream_lock);
    ESP_LOGI(TAG, "UDP stream %s, limit %" PRIu32 " KiB/s",
             enabled ? "enabled" : "disabled", max_rate_kib_s);
    return ESP_OK;
}

esp_err_t ts_stream_configure(const fpga_pid_filter_t filters[FPGA_PID_FILTER_COUNT])
{
    if (!filters)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result = fpga_set_pid_filters(filters);
    if (result != ESP_OK)
    {
        return result;
    }

    portENTER_CRITICAL(&stream_lock);
    for (size_t index = 0; index < FPGA_PID_FILTER_COUNT; index++)
    {
        stream_status.filters[index].config = filters[index];
        stream_status.filters[index].packet_count = 0;
        stream_status.filters[index].byte_count = 0;
        stream_status.filters[index].packets_per_second = 0;
        stream_status.filters[index].bytes_per_second = 0;
        stream_status.filters[index].latest_available = false;
        previous_packet_count[index] = 0;
    }
    previous_rate_time_us = esp_timer_get_time();
    portEXIT_CRITICAL(&stream_lock);
    return ESP_OK;
}

void ts_stream_submit_packet(const uint8_t packet[TS_PACKET_SIZE])
{
    if (!packet || packet[0] != 0x47)
    {
        return;
    }

    uint16_t pid = ((uint16_t)(packet[1] & 0x1f) << 8) | packet[2];
    portENTER_CRITICAL(&stream_lock);
    for (size_t index = 0; index < FPGA_PID_FILTER_COUNT; index++)
    {
        ts_filter_status_t *filter = &stream_status.filters[index];
        if (filter->config.enabled &&
            (pid & filter->config.mask) == (filter->config.pid & filter->config.mask))
        {
            filter->packet_count++;
            filter->byte_count += TS_PACKET_SIZE;
            memcpy(filter->latest_packet, packet, TS_PACKET_SIZE);
            filter->latest_available = true;
        }
    }
    portEXIT_CRITICAL(&stream_lock);

    if (!packet_queue)
    {
        return;
    }

    if (!udp_enabled)
    {
        return;
    }

    queued_ts_packet_t queued_packet;
    memcpy(queued_packet.data, packet, TS_PACKET_SIZE);
    if (xQueueSend(packet_queue, &queued_packet, 0) != pdTRUE)
    {
        portENTER_CRITICAL(&stream_lock);
        stream_status.queue_drop_count++;
        portEXIT_CRITICAL(&stream_lock);
    }
}

void ts_stream_get_status(ts_stream_status_t *status)
{
    if (!status)
    {
        return;
    }

    int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&stream_lock);
    int64_t elapsed_us = now_us - previous_rate_time_us;
    if (elapsed_us > 0)
    {
        for (size_t index = 0; index < FPGA_PID_FILTER_COUNT; index++)
        {
            uint32_t packet_delta = stream_status.filters[index].packet_count -
                                    previous_packet_count[index];
            stream_status.filters[index].packets_per_second =
                (uint32_t)((uint64_t)packet_delta * 1000000 / elapsed_us);
            stream_status.filters[index].bytes_per_second =
                stream_status.filters[index].packets_per_second * TS_PACKET_SIZE;
            previous_packet_count[index] = stream_status.filters[index].packet_count;
        }
        previous_rate_time_us = now_us;
    }
    *status = stream_status;
    portEXIT_CRITICAL(&stream_lock);
}