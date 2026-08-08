/*
 * Header file for I2S slave driver.
 */
#ifndef I2S_SLAVE_H
#define I2S_SLAVE_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2S_TASK_STACK_SIZE 6188
#define I2S_TASK_PRIORITY 5
#define I2S_FPGA_MAX_PIDS 32
#define I2S_FPGA_PAT_SIZE 188

typedef struct {
	bool available;
	uint32_t sync_count;
	uint32_t data_bytes_received;
	uint32_t sync_byte_errors;
} i2s_fpga_statistics_t;

typedef struct {
	bool available;
	bool overflow;
	uint8_t count;
	uint16_t pids[I2S_FPGA_MAX_PIDS];
} i2s_fpga_pid_report_t;

typedef struct {
	bool available;
	uint8_t data[I2S_FPGA_PAT_SIZE];
} i2s_fpga_pat_t;

/**
 * @brief Initializes the I2S peripheral in slave RX mode.
 * 
 * Configures I2S to receive 8-bit data, using the GPIO pins defined in gpio_config.h.
 * DMA is set up to handle 188-byte data packets per WS toggle (stereo configuration).
 * Each full DMA buffer will contain 376 bytes (188 bytes for WS phase 1, 188 bytes for WS phase 2).
 *
 * @return esp_err_t ESP_OK on success, or an error code from the I2S driver on failure.
 */
esp_err_t i2s_init(void);
void i2s_read_task(void *arg);
void i2s_get_fpga_statistics(i2s_fpga_statistics_t *statistics);
void i2s_get_fpga_pid_report(i2s_fpga_pid_report_t *report);
void i2s_get_fpga_pat(i2s_fpga_pat_t *pat);

#ifdef __cplusplus
}
#endif

#endif /* I2S_SLAVE_H */
