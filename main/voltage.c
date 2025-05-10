#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_console.h"

#include "voltage.h"
#include "gpio_config.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "argtable3/argtable3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void voltage_select(voltage_t voltage)
{
    switch (voltage)
    {
    case VOLTAGE_OFF:
        gpio_set_level(GPIO_LNB_ENABLE, 0);
        gpio_set_level(GPIO_TUNER_3V3_SW, 0);
        gpio_set_level(GPIO_TUNER_RESET, 0);
        break;
    case VOLTAGE_13V:
        gpio_set_level(GPIO_LNB_13_18V, 0);
        gpio_set_level(GPIO_LNB_ENABLE, 1);
        gpio_set_level(GPIO_TUNER_3V3_SW, 1);
        gpio_set_level(GPIO_TUNER_RESET, 1);
        break;
    case VOLTAGE_18V:
        gpio_set_level(GPIO_LNB_13_18V, 1);
        gpio_set_level(GPIO_LNB_ENABLE, 1);
        gpio_set_level(GPIO_TUNER_3V3_SW, 1);
        gpio_set_level(GPIO_TUNER_RESET, 1);
        break;
    }
}


// Arguments structure for argtable3
static struct {
    struct arg_str *level;
    struct arg_end *end;
} voltage_args;


// Command handler function
static int voltage_cmd_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &voltage_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, voltage_args.end, argv[0]);
        return 1;
    }

    const char *level_str = voltage_args.level->sval[0];
    voltage_t selected_voltage = VOLTAGE_OFF; // Default to OFF

    if (strcmp(level_str, "off") == 0) {
        selected_voltage = VOLTAGE_OFF;
    } else if (strcmp(level_str, "13") == 0) {
        selected_voltage = VOLTAGE_13V;
    } else if (strcmp(level_str, "18") == 0) {
        selected_voltage = VOLTAGE_18V;
    } else {
        ESP_LOGE("voltage_cmd", "Invalid voltage level: %s. Use 'off', '13', or '18'.", level_str);
        return 1;
    }

    voltage_select(selected_voltage);
    ESP_LOGI("voltage_cmd", "Voltage set to %s", level_str);
    return 0;
}

// Function to register the command
void register_voltage_commands(void)
{
    voltage_args.level = arg_str1(NULL, NULL, "<off|13|18>", "Voltage level to set");
    voltage_args.end = arg_end(2); // Maximum 2 errors printed

    const esp_console_cmd_t cmd = {
        .command = "v",
        .help = "Set LNA voltage (off, 13V, 18V)",
        .hint = NULL,
        .func = &voltage_cmd_handler,
        .argtable = &voltage_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void voltage_init()
{
    gpio_set_direction(GPIO_LNB_ENABLE, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_LNB_13_18V, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_TUNER_RESET, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_TUNER_3V3_SW, GPIO_MODE_OUTPUT);
    voltage_select(VOLTAGE_OFF);
    
    register_voltage_commands();

    //xTaskCreate(voltage_cycle_task, "voltage_cycle_task", 2048, NULL, 5, NULL);
    //ESP_LOGI("voltage", "Voltage cycling task started");
}
