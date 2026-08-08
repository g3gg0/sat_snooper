/*
 * DVB-S/S2 satellite tuner interface
 *
 * Hardware-specific frontend details are kept behind this API.
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    TUNER_POLARIZATION_VERTICAL = 0,
    TUNER_POLARIZATION_HORIZONTAL = 1,
} tuner_polarization_t;

typedef enum {
    TUNER_FEC_NONE = 0,
    TUNER_FEC_1_2 = 1,
    TUNER_FEC_2_3 = 2,
    TUNER_FEC_3_4 = 3,
    TUNER_FEC_4_5 = 4,
    TUNER_FEC_5_6 = 5,
    TUNER_FEC_6_7 = 6,
    TUNER_FEC_7_8 = 7,
    TUNER_FEC_8_9 = 8,
    TUNER_FEC_AUTO = 9,
    TUNER_FEC_3_5 = 10,
    TUNER_FEC_9_10 = 11,
    TUNER_FEC_2_5 = 12,
} tuner_fec_t;

typedef enum {
    TUNER_INVERSION_OFF = 0,
    TUNER_INVERSION_ON = 1,
    TUNER_INVERSION_AUTO = 2,
} tuner_inversion_t;

typedef enum {
    TUNER_DELIVERY_DVB_S = 5,
    TUNER_DELIVERY_DVB_S2 = 6,
} tuner_delivery_system_t;

typedef enum {
    TUNER_MODULATION_QPSK = 0,
    TUNER_MODULATION_8PSK = 9,
} tuner_modulation_t;

typedef enum {
    TUNER_PILOT_ON = 0,
    TUNER_PILOT_OFF = 1,
    TUNER_PILOT_AUTO = 2,
} tuner_pilot_t;

typedef enum {
    TUNER_ROLLOFF_35 = 0,
    TUNER_ROLLOFF_20 = 1,
    TUNER_ROLLOFF_25 = 2,
    TUNER_ROLLOFF_AUTO = 3,
} tuner_rolloff_t;

typedef struct {
    uint32_t frequency_mhz;
    uint32_t symbol_rate_ksps;
    tuner_polarization_t polarization;
    tuner_fec_t fec;
    tuner_inversion_t inversion;
    tuner_delivery_system_t delivery_system;
    tuner_modulation_t modulation;
    tuner_pilot_t pilot;
    tuner_rolloff_t rolloff;
} tuner_tune_request_t;

typedef enum {
    TUNER_STATUS_SIGNAL = 0x01,
    TUNER_STATUS_CARRIER = 0x02,
    TUNER_STATUS_VITERBI = 0x04,
    TUNER_STATUS_SYNC = 0x08,
    TUNER_STATUS_LOCK = 0x10,
    TUNER_STATUS_TIMEOUT = 0x20,
    TUNER_STATUS_REINITIALIZED = 0x40,
} tuner_status_flag_t;

typedef struct {
    bool initialized;
    uint32_t flags;
    bool snr_available;
    uint16_t snr;
    bool strength_available;
    uint16_t strength;
    bool ber_available;
    uint32_t ber;
    bool uncorrected_blocks_available;
    uint32_t uncorrected_blocks;
    bool esno_available;
    uint16_t esno_tenth_db;
    bool acquired_mode_available;
    tuner_delivery_system_t acquired_delivery_system;
    tuner_modulation_t acquired_modulation;
    tuner_fec_t acquired_fec;
    bool acquired_pilot_on;
    bool lnb_ready_available;
    bool lnb_ready;
    bool firmware_running_available;
    bool firmware_running;
    bool firmware_version_available;
    uint8_t firmware_version[4];
    bool chip_id_available;
    uint16_t chip_id;
} tuner_status_t;

/* Console registration is separate from hardware initialization. */
void tuner_register_console(void);
esp_err_t tuner_initialize(void);
bool tuner_is_initialized(void);
esp_err_t tuner_tune(const tuner_tune_request_t *request);
esp_err_t tuner_get_status(tuner_status_t *status);
void tuner_set_reset(bool active);
bool tuner_get_reset(void);



