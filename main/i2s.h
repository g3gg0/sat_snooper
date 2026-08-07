/*
 * Header file for I2S slave driver.
 */
#ifndef I2S_SLAVE_H
#define I2S_SLAVE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2S_TASK_STACK_SIZE 6188
#define I2S_TASK_PRIORITY 5

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

#ifdef __cplusplus
}
#endif

#endif /* I2S_SLAVE_H */
