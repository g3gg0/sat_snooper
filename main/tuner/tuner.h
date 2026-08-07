/*
 * CX24116/CX24118 DVB-S/S2 Frontend Driver Interface
 *
 * This file provides the public API for controlling the Conexant
 * CX24116/CX24118 demodulator.
 */
#pragma once

#include "esp_err.h"
#include "frontend.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Note: Include for DVB API enums like fe_status, fe_code_rate, etc.,
 * (e.g., linux_adapter.h) should be handled within tuner.c, 
 * not exposed in this public header if it causes include issues.
 * The integer types used in the structs below should be mapped to those
 * enums within tuner.c.
 */

/* Structure to hold all tuning parameters for web API */
typedef struct {
    uint32_t freq_mhz;          /* Tuner frequency in kHz */
    uint32_t sr_ksps;           /* Symbol rate in kilo symbols per second */
    int32_t  pol;               /* 0 = V, 1 = H */
    int32_t  fec;               /* FEC (e.g., maps to enum fe_code_rate in tuner.c) */
    int32_t  inv;               /* Inversion (e.g., maps to enum fe_spectral_inversion in tuner.c) */
    int32_t  delsys;            /* Delivery system (e.g., maps to enum fe_delivery_system in tuner.c) */
    int32_t  mod;               /* Modulation (e.g., maps to enum fe_modulation in tuner.c) */
    int32_t  pilot;             /* Pilot (e.g., maps to enum fe_pilot in tuner.c) */
    int32_t  rolloff;           /* Rolloff (e.g., maps to enum fe_rolloff in tuner.c) */
} tuner_tune_params_t;

/* Structure to hold tuner status for web API */
typedef struct {
    bool initialized;
    uint32_t status_flags; /* Raw status flags (e.g., maps from enum fe_status in tuner.c) */
    uint16_t snr;          /* Signal to Noise Ratio */
    uint16_t strength;     /* Signal Strength */
    uint32_t ber;          /* Bit Error Rate */
    uint32_t uc_blocks;    /* Uncorrected Blocks */
} tuner_status_t;

/**
 * @brief Register console commands for CX24116 control.
 */
void tuner_init_base(void);

int tuner_init();

/**
 * @brief Get current tuner status for web API.
 *
 * @param status_data Pointer to a tuner_status_t struct to be filled.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if tuner not initialized, or other error codes.
 */
esp_err_t tuner_get_status_data(tuner_status_t *status_data);

/**
 * @brief Perform tuning based on parameters from web API.
 *
 * @param params Pointer to a tuner_tune_params_t struct with tuning parameters.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if tuner not initialized, or other error codes from tune operation.
 */
esp_err_t tuner_web_tune(const tuner_tune_params_t *params);

void tuner_reset_set(bool state);
bool tuner_reset_get();



