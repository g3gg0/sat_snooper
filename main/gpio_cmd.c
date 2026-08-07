/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "esp_log.h"
#include "esp_console.h"
#include "driver/gpio.h"
#include "gpio_config.h"
#include "gpio_cmd.h"
#include <string.h>
#include <stdlib.h>

static int gpio_handler(int argc, char **argv, int gpio_num, const char *gpio_name)
{
    if (argc == 1)
    {
        int level = gpio_get_level(gpio_num);
        ESP_LOGI("gpio_cmd", "%s is %d", gpio_name, level);
    }
    else if (argc == 2)
    {
        char *end;
        long level = strtol(argv[1], &end, 10);

        if (*end != '\0' || (level != 0 && level != 1))
        {
            ESP_LOGE("gpio_cmd", "Invalid level for %s. Use 0 or 1.", gpio_name);
            return 1;
        }
        gpio_set_level(gpio_num, (int)level);
        ESP_LOGI("gpio_cmd", "Set %s to %d", gpio_name, (int)level);
    }
    else
    {
        ESP_LOGE("gpio_cmd", "Usage: %s [0|1]", argv[0]);
        return 1;
    }
    return 0;
}

static int lnb_en_18v_handler(int argc, char **argv)
{
    return gpio_handler(argc, argv, GPIO_LNB_EN_18V, "GPIO_LNB_EN_18V");
}

static int lnb_en_high_handler(int argc, char **argv)
{
    return gpio_handler(argc, argv, GPIO_LNB_EN_HIGH, "GPIO_LNB_EN_HIGH");
}

static int tuner_en_vcc3_handler(int argc, char **argv)
{
    return gpio_handler(argc, argv, GPIO_TUNER_EN_VCC3, "GPIO_TUNER_EN_VCC3");
}

static int tuner_en_vcore_handler(int argc, char **argv)
{
    return gpio_handler(argc, argv, GPIO_TUNER_EN_VCORE, "GPIO_TUNER_EN_VCORE");
}

void register_gpio_commands(void)
{
    const esp_console_cmd_t cmd_lnb_en_18v = {
        .command = "lnb_en_18v",
        .help = "Get/Set LNB_EN_18V GPIO. No args to get, 0|1 to set.",
        .hint = NULL,
        .func = &lnb_en_18v_handler,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_lnb_en_18v));

    const esp_console_cmd_t cmd_lnb_en_high = {
        .command = "lnb_en_high",
        .help = "Get/Set LNB_EN_HIGH GPIO. No args to get, 0|1 to set.",
        .hint = NULL,
        .func = &lnb_en_high_handler,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_lnb_en_high));

    const esp_console_cmd_t cmd_tuner_en_vcc3 = {
        .command = "tuner_en_vcc3",
        .help = "Get/Set TUNER_EN_VCC3 GPIO. No args to get, 0|1 to set.",
        .hint = NULL,
        .func = &tuner_en_vcc3_handler,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_tuner_en_vcc3));

    const esp_console_cmd_t cmd_tuner_en_vcore = {
        .command = "tuner_en_vcore",
        .help = "Get/Set TUNER_EN_VCORE GPIO. No args to get, 0|1 to set.",
        .hint = NULL,
        .func = &tuner_en_vcore_handler,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_tuner_en_vcore));
}
