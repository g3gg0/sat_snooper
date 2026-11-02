#include "boot_button.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpio_config.h"
#include "serial_protocol.h"
#include "tuner/tuner.h"
#include "channel_config.h"

#define CHANNELS_JSON_PATH "/storage/www/chan.jsn"
#define BOOT_BUTTON_TASK_STACK_SIZE 8192
#define BOOT_BUTTON_TASK_PRIORITY 4
#define BOOT_BUTTON_POLL_MS 20
#define BOOT_BUTTON_DEBOUNCE_MS 50

static const char *TAG = "boot_button";

static void tune_first_channel(void)
{
    tuner_tune_request_t request = {0};
    char name[96] = {0};
    esp_err_t result = channel_config_load_first(CHANNELS_JSON_PATH, &request,
                                                 name, sizeof(name));
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Unable to load first channel from %s: %s",
                 CHANNELS_JSON_PATH, esp_err_to_name(result));
        return;
    }

    ESP_LOGI(TAG, "IO0 pressed; initializing tuner for %s", name);
    result = tuner_initialize();
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Tuner initialization failed: %s", esp_err_to_name(result));
        return;
    }

    result = tuner_tune(&request);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Unable to tune %s: %s", name, esp_err_to_name(result));
        return;
    }
    ESP_LOGI(TAG, "Tuned first channel: %s", name);
}

static void boot_button_task(void *arg)
{
    (void)arg;
    bool boot_button_released = gpio_get_level(GPIO_BOOT_BUTTON) != 0;
    bool serial_stream_button_released = gpio_get_level(GPIO_SERIAL_FPGA_STREAM_BUTTON) != 0;
    bool serial_stream_enabled = false;

    while (true)
    {
        if (gpio_get_level(GPIO_BOOT_BUTTON) != 0)
        {
            boot_button_released = true;
        }
        else if (boot_button_released)
        {
            vTaskDelay(pdMS_TO_TICKS(BOOT_BUTTON_DEBOUNCE_MS));
            if (gpio_get_level(GPIO_BOOT_BUTTON) == 0)
            {
                boot_button_released = false;
                tune_first_channel();
            }
        }

        if (gpio_get_level(GPIO_SERIAL_FPGA_STREAM_BUTTON) != 0)
        {
            serial_stream_button_released = true;
        }
        else if (serial_stream_button_released)
        {
            vTaskDelay(pdMS_TO_TICKS(BOOT_BUTTON_DEBOUNCE_MS));
            if (gpio_get_level(GPIO_SERIAL_FPGA_STREAM_BUTTON) == 0)
            {
                serial_stream_button_released = false;
                if (!serial_stream_enabled)
                {
                    serial_stream_enabled = true;
                    serial_protocol_enable_streaming();
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(BOOT_BUTTON_POLL_MS));
    }
}

esp_err_t boot_button_init(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = (1ULL << GPIO_BOOT_BUTTON) | (1ULL << GPIO_SERIAL_FPGA_STREAM_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t result = gpio_config(&config);
    if (result != ESP_OK)
    {
        return result;
    }

    BaseType_t created = xTaskCreate(boot_button_task, "boot_button",
                                     BOOT_BUTTON_TASK_STACK_SIZE, NULL,
                                     BOOT_BUTTON_TASK_PRIORITY, NULL);
    return created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}