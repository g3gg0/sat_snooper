#pragma once

#include "cJSON.h"
#include "esp_err.h"
#include "tuner/tuner.h"

bool channel_config_parse_tune_request(const cJSON *channel,
                                       tuner_tune_request_t *request);
esp_err_t channel_config_load_first(const char *path,
                                    tuner_tune_request_t *request,
                                    char *name,
                                    size_t name_size);