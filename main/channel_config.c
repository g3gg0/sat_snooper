#include "channel_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHANNEL_CONFIG_MAX_SIZE (64 * 1024)

static bool json_string_equals(const cJSON *root, const char *name, const char *value)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);
    return cJSON_IsString(item) && strcmp(item->valuestring, value) == 0;
}

static bool parse_fec(const cJSON *root, tuner_fec_t *fec)
{
    static const struct {
        const char *name;
        tuner_fec_t value;
    } values[] = {
        {"NONE", TUNER_FEC_NONE}, {"1/2", TUNER_FEC_1_2}, {"2/3", TUNER_FEC_2_3},
        {"3/4", TUNER_FEC_3_4}, {"4/5", TUNER_FEC_4_5}, {"5/6", TUNER_FEC_5_6},
        {"6/7", TUNER_FEC_6_7}, {"7/8", TUNER_FEC_7_8}, {"8/9", TUNER_FEC_8_9},
        {"AUTO", TUNER_FEC_AUTO}, {"3/5", TUNER_FEC_3_5}, {"9/10", TUNER_FEC_9_10},
        {"2/5", TUNER_FEC_2_5},
    };

    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, "fec");
    if (!cJSON_IsString(item))
    {
        return false;
    }
    for (size_t index = 0; index < sizeof(values) / sizeof(values[0]); ++index)
    {
        if (strcmp(item->valuestring, values[index].name) == 0)
        {
            *fec = values[index].value;
            return true;
        }
    }
    return false;
}

bool channel_config_parse_tune_request(const cJSON *channel,
                                       tuner_tune_request_t *request)
{
    if (!cJSON_IsObject(channel) || !request)
    {
        return false;
    }

    const cJSON *frequency = cJSON_GetObjectItemCaseSensitive(channel, "frequency_mhz");
    const cJSON *symbol_rate = cJSON_GetObjectItemCaseSensitive(channel, "symbol_rate_ksps");
    if (!cJSON_IsNumber(frequency) || !cJSON_IsNumber(symbol_rate) ||
        !parse_fec(channel, &request->fec))
    {
        return false;
    }

    request->frequency_mhz = frequency->valueint;
    request->symbol_rate_ksps = symbol_rate->valueint;

    if (json_string_equals(channel, "polarization", "V"))
        request->polarization = TUNER_POLARIZATION_VERTICAL;
    else if (json_string_equals(channel, "polarization", "H"))
        request->polarization = TUNER_POLARIZATION_HORIZONTAL;
    else
        return false;

    if (json_string_equals(channel, "delivery", "DVB-S"))
        request->delivery_system = TUNER_DELIVERY_DVB_S;
    else if (json_string_equals(channel, "delivery", "DVB-S2"))
        request->delivery_system = TUNER_DELIVERY_DVB_S2;
    else
        return false;

    if (json_string_equals(channel, "modulation", "QPSK"))
        request->modulation = TUNER_MODULATION_QPSK;
    else if (json_string_equals(channel, "modulation", "8PSK"))
        request->modulation = TUNER_MODULATION_8PSK;
    else
        return false;

    const cJSON *inversion = cJSON_GetObjectItemCaseSensitive(channel, "inversion");
    if (!inversion)
        request->inversion = TUNER_INVERSION_AUTO;
    else if (json_string_equals(channel, "inversion", "OFF"))
        request->inversion = TUNER_INVERSION_OFF;
    else if (json_string_equals(channel, "inversion", "ON"))
        request->inversion = TUNER_INVERSION_ON;
    else if (json_string_equals(channel, "inversion", "AUTO"))
        request->inversion = TUNER_INVERSION_AUTO;
    else
        return false;

    const cJSON *pilot = cJSON_GetObjectItemCaseSensitive(channel, "pilot");
    if (!pilot)
        request->pilot = TUNER_PILOT_AUTO;
    else if (json_string_equals(channel, "pilot", "ON"))
        request->pilot = TUNER_PILOT_ON;
    else if (json_string_equals(channel, "pilot", "OFF"))
        request->pilot = TUNER_PILOT_OFF;
    else if (json_string_equals(channel, "pilot", "AUTO"))
        request->pilot = TUNER_PILOT_AUTO;
    else
        return false;

    if (json_string_equals(channel, "rolloff", "0.35"))
        request->rolloff = TUNER_ROLLOFF_35;
    else if (json_string_equals(channel, "rolloff", "0.20"))
        request->rolloff = TUNER_ROLLOFF_20;
    else if (json_string_equals(channel, "rolloff", "0.25"))
        request->rolloff = TUNER_ROLLOFF_25;
    else
        return false;

    return true;
}

esp_err_t channel_config_load_first(const char *path,
                                    tuner_tune_request_t *request,
                                    char *name,
                                    size_t name_size)
{
    if (!path || !request)
    {
        return ESP_ERR_INVALID_ARG;
    }

    FILE *file = fopen(path, "rb");
    if (!file)
    {
        return ESP_ERR_NOT_FOUND;
    }
    if (fseek(file, 0, SEEK_END) != 0)
    {
        fclose(file);
        return ESP_FAIL;
    }

    long file_size = ftell(file);
    if (file_size <= 0 || file_size > CHANNEL_CONFIG_MAX_SIZE ||
        fseek(file, 0, SEEK_SET) != 0)
    {
        fclose(file);
        return ESP_ERR_INVALID_SIZE;
    }

    char *contents = malloc((size_t)file_size + 1);
    if (!contents)
    {
        fclose(file);
        return ESP_ERR_NO_MEM;
    }
    size_t bytes_read = fread(contents, 1, (size_t)file_size, file);
    fclose(file);
    if (bytes_read != (size_t)file_size)
    {
        free(contents);
        return ESP_FAIL;
    }
    contents[bytes_read] = '\0';

    cJSON *channels = cJSON_ParseWithLength(contents, bytes_read);
    free(contents);
    if (!cJSON_IsArray(channels))
    {
        cJSON_Delete(channels);
        return ESP_ERR_INVALID_RESPONSE;
    }

    const cJSON *first = cJSON_GetArrayItem(channels, 0);
    bool valid = channel_config_parse_tune_request(first, request);
    if (valid && name && name_size > 0)
    {
        const cJSON *channel_name = cJSON_GetObjectItemCaseSensitive(first, "name");
        snprintf(name, name_size, "%s",
                 cJSON_IsString(channel_name) ? channel_name->valuestring : "first channel");
    }
    cJSON_Delete(channels);
    return valid ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}