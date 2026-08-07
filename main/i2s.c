/*
 * I2S slave driver for receiving 188-byte packets.
 */
#include <string.h>
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2s.h"
#include "esp_log.h"
#include "gpio_config.h" /* Provides GPIO definitions like GPIO_I2S_WS, etc. */

/* I2S peripheral number */
#define I2S_PORT_NUM I2S_NUM_0
#define I2S_DMA_FRAMES_PER_BUFFER 188
#define I2S_DMA_BUF_COUNT 4

static const char *TAG = "I2S_SLAVE";
static i2s_chan_handle_t rx_chan; /* Static handle for the I2S RX channel */

/**
 * @brief Initializes the I2S peripheral in slave RX mode using the standard driver API.
 *
 * @return esp_err_t ESP_OK on success, or an error code from the I2S driver on failure.
 */
esp_err_t i2s_init(void)
{
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
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(10000000/128),
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_8BIT,
            .slot_bit_width = I2S_DATA_BIT_WIDTH_8BIT,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .left_align = false,
            .big_endian = false,
            .bit_order_lsb = false},
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_I2S_BCK,
            .ws = I2S_GPIO_UNUSED,
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

/*
 * Example of how to read data:
 */

#define I2S_READ_BUF_SIZE 188

uint8_t i2s_rx_buffer[I2S_READ_BUF_SIZE];
size_t bytes_read;

void i2s_read_task(void *arg)
{
    if (!rx_chan)
    {
        ESP_LOGE(TAG, "I2S RX channel not initialized for read task.");
        vTaskDelete(NULL);
        return;
    }
    while (1)
    {
        memset(i2s_rx_buffer, 0x00, I2S_READ_BUF_SIZE);
        esp_err_t read_err = i2s_channel_read(rx_chan, i2s_rx_buffer, I2S_READ_BUF_SIZE, &bytes_read, portMAX_DELAY);

        if (read_err == ESP_OK)
        {
            if (bytes_read == I2S_READ_BUF_SIZE)
            {
                ESP_LOGW(TAG, "Read %d bytes, full buffer!", (int)bytes_read);
                ESP_LOG_BUFFER_HEX(TAG, i2s_rx_buffer, I2S_READ_BUF_SIZE);
            }
            else if (bytes_read > 0)
            {
                ESP_LOGW(TAG, "Read %d bytes, expected %d. Partial read?", (int)bytes_read, (int)I2S_READ_BUF_SIZE);
            }
        }
        else
        {
            ESP_LOGE(TAG, "I2S channel read error: %s", esp_err_to_name(read_err));
        }
    }
}
