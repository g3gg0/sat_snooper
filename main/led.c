#include "led.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "led_strip.h"
#include "gpio_config.h"

static led_strip_handle_t s_led_strip;

void led_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    ESP_ERROR_CHECK(led_strip_set_pixel(s_led_strip, 0, r, g, b));
    ESP_ERROR_CHECK(led_strip_refresh(s_led_strip));
}

static struct {
    struct arg_int *r;
    struct arg_int *g;
    struct arg_int *b;
    struct arg_end *end;
} led_args;

static int led_cmd_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&led_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, led_args.end, argv[0]);
        return 1;
    }

    int r = led_args.r->ival[0];
    int g = led_args.g->ival[0];
    int b = led_args.b->ival[0];

    led_set_color(r, g, b);
    ESP_LOGI("led_cmd", "LED color set to R:%d G:%d B:%d", r, g, b);

    return 0;
}

void register_led_commands(void)
{
    led_args.r = arg_int1(NULL, NULL, "<r>", "Red value (0-255)");
    led_args.g = arg_int1(NULL, NULL, "<g>", "Green value (0-255)");
    led_args.b = arg_int1(NULL, NULL, "<b>", "Blue value (0-255)");
    led_args.end = arg_end(3);

    const esp_console_cmd_t cmd = {
        .command = "led",
        .help = "Set WS2812 LED color",
        .hint = NULL,
        .func = &led_cmd_handler,
        .argtable = &led_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void led_init(void)
{
    led_strip_config_t led_strip_conf = {
        .strip_gpio_num = GPIO_WS2812,
        .max_leds = 1, 
    };
    led_strip_rmt_config_t rmt_conf = {
        .resolution_hz = 10000000, // 10MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&led_strip_conf, &rmt_conf, &s_led_strip));
    led_set_color(0, 0, 0);
    register_led_commands();
}
