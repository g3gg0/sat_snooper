#ifndef TS_STREAM_H
#define TS_STREAM_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "fpga.h"

#define TS_PACKET_SIZE 188
#define TS_UDP_PORT 1234

typedef struct
{
    fpga_pid_filter_t config;
    uint32_t packet_count;
    uint32_t byte_count;
    uint32_t packets_per_second;
    uint32_t bytes_per_second;
    bool latest_available;
    uint8_t latest_packet[TS_PACKET_SIZE];
} ts_filter_status_t;

typedef struct
{
    ts_filter_status_t filters[FPGA_PID_FILTER_COUNT];
    uint32_t udp_datagram_count;
    uint32_t udp_packet_count;
    uint32_t udp_send_error_count;
    uint32_t udp_throttle_drop_count;
    uint32_t queue_drop_count;
    bool udp_enabled;
    uint32_t udp_max_rate_kib_s;
} ts_stream_status_t;

esp_err_t ts_stream_init(void);
esp_err_t ts_stream_configure(const fpga_pid_filter_t filters[FPGA_PID_FILTER_COUNT]);
/* max_rate_kib_s of 0 means unlimited */
esp_err_t ts_stream_set_udp(bool enabled, uint32_t max_rate_kib_s);
void ts_stream_submit_packet(const uint8_t packet[TS_PACKET_SIZE]);
void ts_stream_get_status(ts_stream_status_t *status);

#endif