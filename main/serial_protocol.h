#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define SERIAL_PROTOCOL_TYPE_FPGA_FRAME 0x01
#define SERIAL_PROTOCOL_TYPE_RESPONSE 0x02
#define SERIAL_PROTOCOL_TYPE_ERROR 0x03
#define SERIAL_PROTOCOL_TYPE_COMMAND 0x80

esp_err_t serial_protocol_start(void);
void serial_protocol_enable_streaming(void);
void serial_protocol_send_fpga_frame(uint8_t type, const uint8_t *payload, uint8_t length);
bool serial_protocol_streaming(void);
