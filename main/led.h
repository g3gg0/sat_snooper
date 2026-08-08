#pragma once

#include <stdbool.h>
#include <stdint.h>

void led_init(void);
void led_set_color(uint8_t r, uint8_t g, uint8_t b);
void led_set_wifi_status(bool connected, int8_t rssi);
void led_set_tuner_status(bool initialized, bool carrier, bool viterbi);
