/*
 * CX24116/CX24118 DVB-S/S2 Frontend Driver Interface
 *
 * This file provides the public API for controlling the Conexant
 * CX24116/CX24118 demodulator.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Simplified DVB-S/S2 parameter enums (based on Linux DVB API) */
typedef enum {
    CX24116_SYS_DVBS,
    CX24116_SYS_DVBS2
} cx24116_delivery_system_t;

typedef enum {
    CX24116_MOD_QPSK,
    CX24116_MOD_PSK_8
    /* CX24116_MOD_APSK_16, CX24116_MOD_APSK_32 - if supported and needed */
} cx24116_modulation_t;

typedef enum {
    CX24116_FEC_NONE = 0, /* Represents AUTO for DVB-S in some contexts */
    CX24116_FEC_1_2,
    CX24116_FEC_2_3,
    CX24116_FEC_3_4,
    CX24116_FEC_4_5,  /* DVB-S (?), DVB-S2 */
    CX24116_FEC_5_6,
    CX24116_FEC_6_7,  /* DVB-S */
    CX24116_FEC_7_8,  /* DVB-S */
    CX24116_FEC_8_9,  /* DVB-S (?), DVB-S2 */
    CX24116_FEC_9_10, /* DVB-S2 */
    CX24116_FEC_3_5,  /* DVB-S2 */
    CX24116_FEC_AUTO  /* Generic AUTO, driver tries to resolve */
} cx24116_fec_t;

typedef enum {
    CX24116_INVERSION_OFF,
    CX24116_INVERSION_ON,
    CX24116_INVERSION_AUTO
} cx24116_inversion_t;

typedef enum {
    CX24116_PILOT_OFF,
    CX24116_PILOT_ON,
    CX24116_PILOT_AUTO /* Driver attempts to determine if not specified for S2 */
} cx24116_pilot_t;

typedef enum {
    CX24116_ROLLOFF_20, /* DVB-S2: 0.20 */
    CX24116_ROLLOFF_25, /* DVB-S2: 0.25 */
    CX24116_ROLLOFF_35  /* DVB-S & DVB-S2: 0.35 */
    /* CX24116_ROLLOFF_AUTO - Not directly supported by HW for S2, must be explicit */
} cx24116_rolloff_t;

typedef struct {
    uint32_t frequency_khz;     /* Tuner frequency in kHz */
    uint32_t symbol_rate_sps;   /* Symbol rate in symbols per second */
    cx24116_fec_t fec;          /* Forward Error Correction rate */
    cx24116_inversion_t inversion; /* Spectral inversion setting */
    cx24116_delivery_system_t delivery_system; /* DVB-S or DVB-S2 */
    cx24116_modulation_t modulation; /* QPSK, 8PSK, etc. */
    cx24116_pilot_t pilot;      /* Pilot tones for DVB-S2 */
    cx24116_rolloff_t rolloff;  /* Rolloff factor for DVB-S2 */
} cx24116_tune_params_t;

/* Status flags, mirroring fe_status from Linux DVB */
typedef enum {
    CX24116_STATUS_IDLE = 0,
    CX24116_STATUS_HAS_SIGNAL  = 0x01, /* Found something above the noise floor */
    CX24116_STATUS_HAS_CARRIER = 0x02, /* Found a DVB signal */
    CX24116_STATUS_HAS_VITERBI = 0x04, /* FEC lock */
    CX24116_STATUS_HAS_SYNC    = 0x08, /* Found sync bytes */
    CX24116_STATUS_HAS_LOCK    = 0x10, /* Signal, Carrier, Viterbi and Sync found */
    /* Combined status for convenience, matching typical FE_HAS_LOCK */
    CX24116_STATUS_LOCKED_ALL = (CX24116_STATUS_HAS_SIGNAL | CX24116_STATUS_HAS_CARRIER | \
                                 CX24116_STATUS_HAS_VITERBI | CX24116_STATUS_HAS_SYNC | CX24116_STATUS_HAS_LOCK)
} cx24116_status_mask_t;


/**
 * @brief Probe for the CX24116 device.
 *
 * @param i2c_addr The I2C address of the CX24116.
 * @return 0 if device is found and identified, error code otherwise.
 */
int cx24116_probe(uint8_t i2c_addr);

/**
 * @brief Initialize the CX24116 device and load firmware.
 * This function should be called once before any other operations.
 *
 * @param i2c_addr The I2C address of the CX24116.
 * @param fw_data Pointer to the firmware binary data.
 * @param fw_size Size of the firmware data in bytes.
 * @param mpeg_clk_pos_pol MPEG clock polarity (0x00-0xFF, e.g., 0x02 from reference).
 * @return 0 on success, negative error code on failure.
 */
int cx24116_init_and_load_firmware(uint8_t i2c_addr, const uint8_t *fw_data, size_t fw_size, uint8_t mpeg_clk_pos_pol);

/**
 * @brief Tune the CX24116 to a specific transponder.
 *
 * @param i2c_addr The I2C address of the CX24116.
 * @param params Pointer to the tuning parameters structure.
 * @return 0 on success, negative error code on failure.
 */
int cx24116_tune(uint8_t i2c_addr, const cx24116_tune_params_t *params);

/**
 * @brief Read the lock status from the CX24116.
 *
 * @param i2c_addr The I2C address of the CX24116.
 * @param status Pointer to store the status flags (combination of cx24116_status_mask_t).
 * @return 0 on success, negative error code on failure.
 */
int cx24116_read_status(uint8_t i2c_addr, uint8_t *status);

/**
 * @brief Read Signal-to-Noise Ratio (SNR).
 * Returns the raw 16-bit Es/No value from registers 0xA3 (LSB) and 0xBC (MSB - QSTATUS upper bits for S2, or 0xA3 for S1).
 * The reference driver uses 0xD5 (CX24116_REG_QUALITY0) and 0xA3 (CX24116_REG_QUALITY8).
 *
 * @param i2c_addr The I2C address of the CX24116.
 * @param snr Pointer to store the SNR value (raw EsNo * 10 or similar).
 * @return 0 on success, negative error code on failure.
 */
int cx24116_read_snr(uint8_t i2c_addr, uint16_t *snr);

/**
 * @brief Read Signal Strength (AGC level).
 * Returns a raw AGC value. Lower values typically mean stronger signal.
 *
 * @param i2c_addr The I2C address of the CX24116.
 * @param strength Pointer to store the signal strength value.
 * @return 0 on success, negative error code on failure.
 */
int cx24116_read_signal_strength(uint8_t i2c_addr, uint16_t *strength);

/**
 * @brief Set LNB voltage.
 *
 * @param i2c_addr I2C address of the CX24116.
 * @param voltage 0 for 13V, 1 for 18V.
 * @return 0 on success, error code otherwise.
 */
int cx24116_set_voltage(uint8_t i2c_addr, uint8_t voltage_is_18v);

/**
 * @brief Set LNB continuous 22kHz tone.
 *
 * @param i2c_addr I2C address of the CX24116.
 * @param tone_on 0 for OFF, 1 for ON.
 * @return 0 on success, error code otherwise.
 */
int cx24116_set_tone(uint8_t i2c_addr, uint8_t tone_on);

/**
 * @brief Put the CX24116 into sleep mode.
 *
 * @param i2c_addr The I2C address of the CX24116.
 * @return 0 on success, negative error code on failure.
 */
int cx24116_sleep(uint8_t i2c_addr);

/**
 * @brief Wake up the CX24116 from sleep mode and perform basic initialization.
 *
 * @param i2c_addr The I2C address of the CX24116.
 * @return 0 on success, error code otherwise.
 */
int cx24116_wakeup_and_init(uint8_t i2c_addr);

/**
 * @brief Register console commands for CX24116 control.
 */
void register_cx24116_commands(void);

