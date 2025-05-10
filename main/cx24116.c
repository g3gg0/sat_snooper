/*
 * CX24116/CX24118 DVB-S/S2 Frontend Driver Implementation
 *
 * lnb: https://docs.televes.com/DS11180009_004_54728.pdf
 */
#include "cx24116.h"
#include "i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h> /* For memcpy */
#include <stdlib.h> /* For NULL */

/* Additional includes for console commands */
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include <stdio.h> /* For fopen, fread, etc. */

/* Includes for Linux-style driver integration */
#include "linux_adapter.h" /* For KERN_INFO, printk, etc. and potentially stubs */
#include "cx24116_linux.h" /* For cx24116_attach, struct cx24116_config, struct dvb_frontend, struct i2c_adapter */

static const char *TAG_CX = "cx24116";

#define CX24116_DEFAULT_ADDRESS 0x55

/* --- Register Definitions (from cx24116-elixir.c) --- */
#define CX24116_REG_COMMAND (0x00)      /* command args 0x00..0x1e */
#define CX24116_REG_EXECUTE (0x1f)      /* execute command */
#define CX24116_REG_RESET   (0x20)      /* reset status > 0     */
#define CX24116_REG_MAILBOX (0x96)      /* FW or multipurpose mailbox? */
#define CX24116_REG_SIGNAL  (0x9e)      /* signal low           */
#define CX24116_REG_SSTATUS (0x9d)      /* signal high / status */
#define CX24116_REG_QUALITY8 (0xa3)     /* SNR MSB (for EsNo mode) / Quality for S1 */
#define CX24116_REG_QSTATUS (0xbc)      /* LNB Ready, etc. */
#define CX24116_REG_QUALITY0 (0xd5)     /* SNR LSB (for EsNo mode) / Quality for S1 */
#define CX24116_REG_BER0    (0xc9)
#define CX24116_REG_BER8    (0xc8)
#define CX24116_REG_BER16   (0xc7)
#define CX24116_REG_BER24   (0xc6)
#define CX24116_REG_UCB0    (0xcb)
#define CX24116_REG_UCB8    (0xca)
#define CX24116_REG_CLKDIV  (0xf3)
#define CX24116_REG_RATEDIV (0xf9)
#define CX24116_REG_ID_MSB  (0xFF)      /* Device ID MSB */
#define CX24116_REG_ID_LSB  (0xFE)      /* Device ID LSB */

/* --- Command Definitions (from cx24116-elixir.c) --- */
enum cx24116_cmds {
    CMD_SET_VCO     = 0x10,
    CMD_TUNEREQUEST = 0x11,
    CMD_MPEGCONFIG  = 0x13,
    CMD_TUNERINIT   = 0x14,
    CMD_BANDWIDTH   = 0x15,
    CMD_GETAGC      = 0x19,
    CMD_LNBCONFIG   = 0x20,
    CMD_LNBSEND     = 0x21, /* Formerly CMD_SEND_DISEQC */
    CMD_LNBDCLEVEL  = 0x22,
    CMD_SET_TONE    = 0x23,
    CMD_UPDFWVERS   = 0x35,
    CMD_TUNERSLEEP  = 0x36,
    CMD_AGCCONTROL  = 0x3b, /* Unknown */
};

#define CX24116_ARGLEN (0x1e)

/* --- Helper Functions --- */

/**
 * @brief Write a single byte to a CX24116 register.
 */
static int cx24116_write_reg(uint8_t i2c_addr, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return i2c_write(i2c_addr, buf, 2);
}

/**
 * @brief Write multiple bytes to the CX24116, starting at a given register address.
 * Used for firmware loading.
 */
static int cx24116_write_reg_n(uint8_t i2c_addr, uint8_t start_reg, const uint8_t *data, uint16_t len) {
    if (len == 0) return 0;
    uint8_t *buf = malloc(len + 1);
    if (!buf) {
        ESP_LOGE(TAG_CX, "Failed to allocate memory for reg_n write");
        return -1; /* Indicate memory allocation failure */
    }
    buf[0] = start_reg;
    memcpy(buf + 1, data, len);
    int ret = i2c_write(i2c_addr, buf, len + 1);
    free(buf);
    return ret;
}

/**
 * @brief Read a single byte from a CX24116 register.
 */
static int cx24116_read_reg(uint8_t i2c_addr, uint8_t reg, uint8_t *data) {
    return i2c_write_read(i2c_addr, &reg, 1, data, 1);
}

/**
 * @brief Execute a command on the CX24116.
 */
static int cx24116_cmd_execute_internal(uint8_t i2c_addr, uint8_t *cmd_args, uint8_t cmd_len) {
    int ret;
    for (uint8_t i = 0; i < cmd_len; i++) {
        ret = cx24116_write_reg(i2c_addr, CX24116_REG_COMMAND + i, cmd_args[i]);
        if (ret != 0) {
            ESP_LOGE(TAG_CX, "Cmd execute: Error writing arg %d (0x%02x)", i, cmd_args[i]);
            return -1;
        }
    }

    ret = cx24116_write_reg(i2c_addr, CX24116_REG_EXECUTE, 0x01);
    if (ret != 0) {
        ESP_LOGE(TAG_CX, "Cmd execute: Error writing execute reg");
        return -1;
    }

    uint8_t exec_status;
    int attempts = 0;
    do {
        vTaskDelay(pdMS_TO_TICKS(10));
        ret = cx24116_read_reg(i2c_addr, CX24116_REG_EXECUTE, &exec_status);
        if (ret != 0) {
            ESP_LOGE(TAG_CX, "Cmd execute: Error reading execute status");
            return -1;
        }
        if (attempts++ > 64) { /* Timeout after ~640ms */
            ESP_LOGE(TAG_CX, "Cmd execute: Firmware not responding (timeout)");
            return -2; /* Indicate timeout */
        }
    } while (exec_status != 0x00);

    return 0;
}

/* --- Public API Implementation --- */

int cx24116_probe(uint8_t i2c_addr) {
    uint8_t id_msb, id_lsb;
    if (cx24116_read_reg(i2c_addr, CX24116_REG_ID_MSB, &id_msb) != 0) return -1;
    if (cx24116_read_reg(i2c_addr, CX24116_REG_ID_LSB, &id_lsb) != 0) return -1;

    uint16_t device_id = (id_msb << 8) | id_lsb;
    ESP_LOGI(TAG_CX, "Probing CX24116 at 0x%02X: Device ID = 0x%04X", i2c_addr, device_id);
    if (device_id == 0x0501) { /* Expected ID for CX24116 */
        ESP_LOGI(TAG_CX, "CX24116/CX24118 found.");
        return 0;
    }
    ESP_LOGE(TAG_CX, "CX24116/CX24118 not found or unexpected ID.");
    return -1;
}

int cx24116_init_and_load_firmware(uint8_t i2c_addr, const uint8_t *fw_data, size_t fw_size, uint8_t mpeg_clk_pos_pol) {
    ESP_LOGI(TAG_CX, "Initializing CX24116 and loading firmware (%d bytes)...", fw_size);

    /* Based on cx24116_load_firmware from reference */
    /* Reset procedure might be external (GPIO toggle), not handled here */

    /* Init PLL */
    cx24116_write_reg(i2c_addr, 0xE5, 0x00);
    cx24116_write_reg(i2c_addr, 0xF1, 0x08);
    cx24116_write_reg(i2c_addr, 0xF2, 0x13);

    /* Start PLL */
    cx24116_write_reg(i2c_addr, 0xe0, 0x03);
    cx24116_write_reg(i2c_addr, 0xe0, 0x00);

    cx24116_write_reg(i2c_addr, CX24116_REG_CLKDIV, 0x46);
    cx24116_write_reg(i2c_addr, CX24116_REG_RATEDIV, 0x00);

    cx24116_write_reg(i2c_addr, 0xF0, 0x03);
    cx24116_write_reg(i2c_addr, 0xF4, 0x81);
    cx24116_write_reg(i2c_addr, 0xF5, 0x00);
    cx24116_write_reg(i2c_addr, 0xF6, 0x00);

    /* Firmware load - reference uses i2c_wr_max, ESP-IDF I2C driver handles chunking if needed by controller */
    /* We send it in manageable chunks if fw_size is large, though i2c_write might handle this. */
    /* The reference driver writes to 0xF7. */
    const uint16_t max_chunk_size = 250; /* Max data bytes per write, plus 1 for register */
    for (size_t offset = 0; offset < fw_size; offset += max_chunk_size) {
        uint16_t current_chunk_size = (fw_size - offset > max_chunk_size) ? max_chunk_size : (fw_size - offset);
        if (cx24116_write_reg_n(i2c_addr, 0xF7, &fw_data[offset], current_chunk_size) != 0) {
            ESP_LOGE(TAG_CX, "Firmware load failed at offset %d", offset);
            return -1;
        }
        ESP_LOGD(TAG_CX, "Wrote %d bytes of firmware at offset %d", current_chunk_size, offset);
        vTaskDelay(pdMS_TO_TICKS(5)); /* Small delay between chunks */
    }

    cx24116_write_reg(i2c_addr, 0xF4, 0x10);
    cx24116_write_reg(i2c_addr, 0xF0, 0x00);
    cx24116_write_reg(i2c_addr, 0xF8, 0x06);

    uint8_t cmd_buf[CX24116_ARGLEN];

    /* CMD_SET_VCO */
    cmd_buf[0x00] = CMD_SET_VCO;
    cmd_buf[0x01] = 0x05; cmd_buf[0x02] = 0xdc; cmd_buf[0x03] = 0xda; cmd_buf[0x04] = 0xae;
    cmd_buf[0x05] = 0xaa; cmd_buf[0x06] = 0x04; cmd_buf[0x07] = 0x9d; cmd_buf[0x08] = 0xfc;
    cmd_buf[0x09] = 0x06;
    if (cx24116_cmd_execute_internal(i2c_addr, cmd_buf, 0x0a) != 0) return -1;

    cx24116_write_reg(i2c_addr, CX24116_REG_SSTATUS, 0x00);

    /* CMD_TUNERINIT */
    cmd_buf[0x00] = CMD_TUNERINIT;
    cmd_buf[0x01] = 0x00;
    cmd_buf[0x02] = 0x00;
    if (cx24116_cmd_execute_internal(i2c_addr, cmd_buf, 0x03) != 0) return -1;

    cx24116_write_reg(i2c_addr, 0xe5, 0x00);

    /* CMD_MPEGCONFIG */
    cmd_buf[0x00] = CMD_MPEGCONFIG;
    cmd_buf[0x01] = 0x01; /* Enable MPEG output */
    cmd_buf[0x02] = 0x75; /* Default from ref */
    cmd_buf[0x03] = 0x00; /* Default from ref */
    cmd_buf[0x04] = mpeg_clk_pos_pol; /* Configurable, e.g., 0x02 */
    cmd_buf[0x05] = 0x00;
    if (cx24116_cmd_execute_internal(i2c_addr, cmd_buf, 0x06) != 0) return -1;

    /* CMD_UPDFWVERS - Get firmware version */
    uint8_t vers[4];
    cmd_buf[0x00] = CMD_UPDFWVERS;
    for (int i = 0; i < 4; i++) {
        cmd_buf[0x01] = i;
        if (cx24116_cmd_execute_internal(i2c_addr, cmd_buf, 0x02) != 0) return -1;
        if (cx24116_read_reg(i2c_addr, CX24116_REG_MAILBOX, &vers[i]) != 0) return -1;
    }
    ESP_LOGI(TAG_CX, "Firmware version: %d.%d.%d.%d", vers[0], vers[1], vers[2], vers[3]);

    /* Read SYSCHIPID and SYSCHIPVER after successful FW load and version check */
    uint8_t syschipid, syschipver;
    if (cx24116_read_reg(i2c_addr, 0x94, &syschipid) == 0) {
        ESP_LOGI(TAG_CX, "SYSCHIPID (0x94): 0x%02X", syschipid);
    }
    if (cx24116_read_reg(i2c_addr, 0x95, &syschipver) == 0) {
        ESP_LOGI(TAG_CX, "SYSCHIPVER (0x95): 0x%02X", syschipver);
    }

    ESP_LOGI(TAG_CX, "Firmware load complete.");
    return 0;
}

/* Helper to map API FEC to register values */
static void map_fec_params(const cx24116_tune_params_t *params, uint8_t *reg_fec_val, uint8_t *reg_fec_mask) {
    *reg_fec_mask = 0xFE; /* Default for DVB-S Auto */
    *reg_fec_val  = 0x30; /* Default for DVB-S Auto */

    if (params->delivery_system == CX24116_SYS_DVBS) {
        switch (params->fec) {
            case CX24116_FEC_1_2:  *reg_fec_mask = 0x02; *reg_fec_val = 0x2E; break;
            case CX24116_FEC_2_3:  *reg_fec_mask = 0x04; *reg_fec_val = 0x2F; break;
            case CX24116_FEC_3_4:  *reg_fec_mask = 0x08; *reg_fec_val = 0x30; break;
            case CX24116_FEC_5_6:  *reg_fec_mask = 0x20; *reg_fec_val = 0x31; break;
            case CX24116_FEC_7_8:  *reg_fec_mask = 0x80; *reg_fec_val = 0x32; break;
            case CX24116_FEC_AUTO:
            default: *reg_fec_mask = 0xFE; *reg_fec_val = 0x30; break;
        }
    } else { /* CX24116_SYS_DVBS2 */
        *reg_fec_mask = 0x00; /* Mask is 0 for DVB-S2 */
        if (params->modulation == CX24116_MOD_QPSK) {
            switch (params->fec) {
                case CX24116_FEC_1_2:  *reg_fec_val = 0x04; break;
                case CX24116_FEC_3_5:  *reg_fec_val = 0x05; break;
                case CX24116_FEC_2_3:  *reg_fec_val = 0x06; break;
                case CX24116_FEC_3_4:  *reg_fec_val = 0x07; break;
                case CX24116_FEC_4_5:  *reg_fec_val = 0x08; break;
                case CX24116_FEC_5_6:  *reg_fec_val = 0x09; break;
                case CX24116_FEC_8_9:  *reg_fec_val = 0x0A; break;
                case CX24116_FEC_9_10: *reg_fec_val = 0x0B; break;
                default: ESP_LOGW(TAG_CX, "Unsupported S2 QPSK FEC: %d", params->fec); *reg_fec_val = 0x04; break;
            }
        } else if (params->modulation == CX24116_MOD_PSK_8) {
            switch (params->fec) {
                case CX24116_FEC_3_5:  *reg_fec_val = 0x0C; break;
                case CX24116_FEC_2_3:  *reg_fec_val = 0x0D; break;
                case CX24116_FEC_3_4:  *reg_fec_val = 0x0E; break;
                case CX24116_FEC_5_6:  *reg_fec_val = 0x0F; break;
                case CX24116_FEC_8_9:  *reg_fec_val = 0x10; break;
                case CX24116_FEC_9_10: *reg_fec_val = 0x11; break;
                default: ESP_LOGW(TAG_CX, "Unsupported S2 8PSK FEC: %d", params->fec); *reg_fec_val = 0x0C; break;
            }
        } else {
            ESP_LOGE(TAG_CX, "Unsupported S2 modulation for FEC mapping: %d", params->modulation);
            *reg_fec_val = 0x04; 
        }
    }
}

int cx24116_tune(uint8_t i2c_addr, const cx24116_tune_params_t *params) {
    ESP_LOGI(TAG_CX, "Tuning to Freq: %"PRIu32" kHz, SR: %"PRIu32" Sps, DelSys: %s", 
        params->frequency_khz, params->symbol_rate_sps, 
        params->delivery_system == CX24116_SYS_DVBS ? "DVBS" : "DVBS2");

    uint8_t cmd_buf[CX24116_ARGLEN];
    int ret;
    int retune_attempts = (params->delivery_system == CX24116_SYS_DVBS2 && params->pilot == CX24116_PILOT_AUTO && params->modulation == CX24116_MOD_PSK_8) ? 2 : 1;

    /* Set/Reset B/W (as per reference cx24116_set_frontend) */
    cmd_buf[0x00] = CMD_BANDWIDTH;
    cmd_buf[0x01] = 0x01; /* Set B/W before tune */
    if(cx24116_cmd_execute_internal(i2c_addr, cmd_buf, 0x02) != 0) return -1;

    /* Prepare CMD_TUNEREQUEST */
    cmd_buf[0x00] = CMD_TUNEREQUEST;
    cmd_buf[0x01] = (params->frequency_khz & 0xff0000) >> 16;
    cmd_buf[0x02] = (params->frequency_khz & 0x00ff00) >> 8;
    cmd_buf[0x03] = (params->frequency_khz & 0x0000ff);
    uint32_t symbol_rate_ksps = params->symbol_rate_sps / 1000;
    cmd_buf[0x04] = (symbol_rate_ksps & 0xff00) >> 8;
    cmd_buf[0x05] = (symbol_rate_ksps & 0x00ff);
    uint8_t inversion_val = 0x0C; /* Default AUTO */
    if (params->inversion == CX24116_INVERSION_OFF) inversion_val = 0x00;
    else if (params->inversion == CX24116_INVERSION_ON) inversion_val = 0x04;
    cmd_buf[0x06] = inversion_val;
    uint8_t fec_val, fec_mask;
    map_fec_params(params, &fec_val, &fec_mask);
    uint8_t pilot_val_initial = 0x00;
    if (params->delivery_system == CX24116_SYS_DVBS2) {
        if (params->pilot == CX24116_PILOT_ON) pilot_val_initial = 0x40;
        else if (params->pilot == CX24116_PILOT_AUTO) {
            pilot_val_initial = (params->modulation == CX24116_MOD_PSK_8) ? 0x40 : 0x00; /* Start with ON for 8PSK, OFF for QPSK */
        }
    }
    cmd_buf[0x07] = fec_val | pilot_val_initial;
#define CX24116_SEARCH_RANGE_KHZ 5000
    cmd_buf[0x08] = (CX24116_SEARCH_RANGE_KHZ >> 8) & 0xff;
    cmd_buf[0x09] = CX24116_SEARCH_RANGE_KHZ & 0xff;
    cmd_buf[0x0a] = 0x00; 
    cmd_buf[0x0b] = 0x00; 
    uint8_t rolloff_val = 0x02; /* Default 0.35 */
    if (params->delivery_system == CX24116_SYS_DVBS2) {
        if (params->rolloff == CX24116_ROLLOFF_20) rolloff_val = 0x00;
        else if (params->rolloff == CX24116_ROLLOFF_25) rolloff_val = 0x01;
    }
    cmd_buf[0x0c] = rolloff_val;
    cmd_buf[0x0d] = fec_mask;
    if (params->symbol_rate_sps > 30000000) {
        cmd_buf[0x0e] = 0x04; cmd_buf[0x0f] = 0x00; cmd_buf[0x10] = 0x01;
        cmd_buf[0x11] = 0x77; cmd_buf[0x12] = 0x36;
        cx24116_write_reg(i2c_addr, CX24116_REG_CLKDIV, 0x44);
        cx24116_write_reg(i2c_addr, CX24116_REG_RATEDIV, 0x01);
    } else {
        cmd_buf[0x0e] = 0x06; cmd_buf[0x0f] = 0x00; cmd_buf[0x10] = 0x00;
        cmd_buf[0x11] = 0xFA; cmd_buf[0x12] = 0x24;
        cx24116_write_reg(i2c_addr, CX24116_REG_CLKDIV, 0x46);
        cx24116_write_reg(i2c_addr, CX24116_REG_RATEDIV, 0x00);
    }
    uint8_t tune_cmd_len = 0x13;

    for (int attempt = 0; attempt < retune_attempts; attempt++) {
        uint8_t sstatus_val;
        if (cx24116_read_reg(i2c_addr, CX24116_REG_SSTATUS, &sstatus_val) == 0) {
            cx24116_write_reg(i2c_addr, CX24116_REG_SSTATUS, (sstatus_val & 0xC0));
        }
        ESP_LOGD(TAG_CX, "Tune CMD (attempt %d, len %d, pilot_byte 0x%02X):", attempt + 1, tune_cmd_len, cmd_buf[0x07]);
        for(int k=0; k<tune_cmd_len; k++) ESP_LOGD(TAG_CX, "  0x%02X: 0x%02x", k, cmd_buf[k]);
        ret = cx24116_cmd_execute_internal(i2c_addr, cmd_buf, tune_cmd_len);
        if (ret != 0) {
            ESP_LOGE(TAG_CX, "Tune command execution failed on attempt %d.", attempt + 1);
            continue; /* Try next attempt if any */
        }
        uint8_t current_status = 0;
        for (int i = 0; i < 50; i++) { 
            vTaskDelay(pdMS_TO_TICKS(20));
            if (cx24116_read_status(i2c_addr, &current_status) == 0) {
                if ((current_status & CX24116_STATUS_LOCKED_ALL) == CX24116_STATUS_LOCKED_ALL) {
                    ESP_LOGI(TAG_CX, "Tune successful on attempt %d, locked!", attempt + 1);
                    goto tuned_finish_success;
                }
            }
        }
        ESP_LOGW(TAG_CX, "Tune attempt %d timeout or no lock (status: 0x%02X).", attempt + 1, current_status);
        if (attempt < retune_attempts -1) { /* If there's another attempt for auto pilot */
             cmd_buf[0x07] = fec_val | (pilot_val_initial ^ 0x40); /* Toggle pilot bit */
             ESP_LOGI(TAG_CX, "Retrying tune with pilot toggled to 0x%02X", cmd_buf[0x07] & 0x40);
        }
    }
    /* If loop finishes without lock */
    ESP_LOGE(TAG_CX, "All tune attempts failed.");
    cmd_buf[0x00] = CMD_BANDWIDTH;
    cmd_buf[0x01] = 0x00; /* Reset B/W after tune */
    cx24116_cmd_execute_internal(i2c_addr, cmd_buf, 0x02);
    return -1; /* Indicate tune failure */

tuned_finish_success:
    cmd_buf[0x00] = CMD_BANDWIDTH;
    cmd_buf[0x01] = 0x00; /* Reset B/W after tune */
    return cx24116_cmd_execute_internal(i2c_addr, cmd_buf, 0x02);
}

/**
 * @brief Wait for LNB to be ready.
 * Checks QSTATUS register (0xBC), bit 0x20.
 * @param i2c_addr I2C address of the CX24116.
 * @return 0 if LNB is ready, -1 on timeout or error.
 */
static int cx24116_wait_for_lnb(uint8_t i2c_addr) {
    uint8_t qstatus;
    int ret;
    ESP_LOGD(TAG_CX, "Waiting for LNB ready...");
    for (int i = 0; i < 30; i++) { /* Timeout after ~300ms */
        ret = cx24116_read_reg(i2c_addr, CX24116_REG_QSTATUS, &qstatus);
        if (ret != 0) {
            ESP_LOGE(TAG_CX, "Failed to read QSTATUS for LNB ready check.");
            return -1;
        }
        ESP_LOGD(TAG_CX, "QSTATUS (0xBC) = 0x%02X", qstatus);
        if (qstatus & 0x20) {
            ESP_LOGD(TAG_CX, "LNB ready.");
            return 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGW(TAG_CX, "Timeout waiting for LNB ready (QSTATUS=0x%02X).", qstatus);
    return -1; /* Timeout */
}

/* --- Public API Implementation --- */

int cx24116_read_status(uint8_t i2c_addr, uint8_t *status) {
    return cx24116_read_reg(i2c_addr, CX24116_REG_RESET, status);
}

int cx24116_read_snr(uint8_t i2c_addr, uint16_t *snr) {
    uint8_t snr_msb, snr_lsb;
    int ret = cx24116_read_reg(i2c_addr, CX24116_REG_QUALITY8, &snr_msb);
    if (ret != 0) return ret;
    ret = cx24116_read_reg(i2c_addr, CX24116_REG_QUALITY0, &snr_lsb);
    if (ret != 0) return ret;
    *snr = (snr_msb << 8) | snr_lsb;
    return 0;
}

int cx24116_read_signal_strength(uint8_t i2c_addr, uint16_t *strength) {
    uint8_t sstatus_val, signal_low;
    int ret = cx24116_read_reg(i2c_addr, CX24116_REG_SSTATUS, &sstatus_val);
    if (ret != 0) return ret;
    ret = cx24116_read_reg(i2c_addr, CX24116_REG_SIGNAL, &signal_low);
    if (ret != 0) return ret;

    /* Simple AGC strength calculation (based on existing formula) */
    int32_t sig_reading = (int32_t)((sstatus_val & 0x0F) * 685);
    sig_reading += (int32_t)((signal_low & 0xFF) * 16);
    sig_reading = (sig_reading < 0) ? 0 : ((sig_reading > 65535) ? 65535 : sig_reading);
    *strength = (uint16_t)sig_reading;

    ESP_LOGD(TAG_CX, "Read Strength: SSTATUS=0x%02x, SIGNAL=0x%02x, raw_agc=0x%04"PRId32", cooked=0x%04x", 
        sstatus_val, signal_low, sig_reading, *strength);
    return 0;
}

int cx24116_set_voltage(uint8_t i2c_addr, uint8_t voltage_is_18v) {
    uint8_t cmd_buf[2];
    int ret;

    ESP_LOGI(TAG_CX, "Setting LNB Voltage to %sV", voltage_is_18v ? "18" : "13");

    ret = cx24116_wait_for_lnb(i2c_addr);
    if (ret != 0) {
        ESP_LOGE(TAG_CX, "LNB not ready before setting voltage.");
        return ret;
    }

    /* Min delay time before DiSEqC send / voltage change (from ref) */
    vTaskDelay(pdMS_TO_TICKS(100)); /* Reference uses msleep(100) */

    cmd_buf[0x00] = CMD_LNBDCLEVEL;
    cmd_buf[0x01] = voltage_is_18v ? 0x01 : 0x00;
    
    ret = cx24116_cmd_execute_internal(i2c_addr, cmd_buf, 2);
    if (ret == 0) {
         /* Min delay time after DiSEqC send (from ref, msleep(15)) - good practice after voltage change too */
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    return ret;
}

int cx24116_set_tone(uint8_t i2c_addr, uint8_t tone_on) {
    uint8_t cmd_buf[4];
    int ret;

    ESP_LOGI(TAG_CX, "Setting LNB Tone %s", tone_on ? "ON" : "OFF");

    ret = cx24116_wait_for_lnb(i2c_addr);
    if (ret != 0) {
        ESP_LOGE(TAG_CX, "LNB not ready before setting tone.");
        return ret;
    }

    /* Min delay time after DiSEqC send (from ref, msleep(15)) - applies before tone change too */
    vTaskDelay(pdMS_TO_TICKS(15));

    cmd_buf[0x00] = CMD_SET_TONE;
    cmd_buf[0x01] = 0x00;
    cmd_buf[0x02] = 0x00;
    cmd_buf[0x03] = tone_on ? 0x01 : 0x00;
    
    ret = cx24116_cmd_execute_internal(i2c_addr, cmd_buf, 4);
    if (ret == 0) {
        /* Min delay time before DiSEqC send (from ref, msleep(15)) - good practice after tone change */
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    return ret;
}

int cx24116_sleep(uint8_t i2c_addr) {
    uint8_t cmd_buf[2];
    cmd_buf[0x00] = CMD_TUNERSLEEP;
    cmd_buf[0x01] = 0x01; /* Sleep */
    return cx24116_cmd_execute_internal(i2c_addr, cmd_buf, 2);
}

int cx24116_wakeup_and_init(uint8_t i2c_addr) {
    ESP_LOGI(TAG_CX, "Waking up CX24116.");
    /* Power on clocks */
    cx24116_write_reg(i2c_addr, 0xe0, 0);
    cx24116_write_reg(i2c_addr, 0xe1, 0);
    cx24116_write_reg(i2c_addr, 0xea, 0);

    uint8_t cmd_buf[2];
    cmd_buf[0x00] = CMD_TUNERSLEEP;
    cmd_buf[0x01] = 0x00; /* Wake up */
    int ret = cx24116_cmd_execute_internal(i2c_addr, cmd_buf, 2);
    if (ret != 0) {
        ESP_LOGE(TAG_CX, "Wakeup command (CMD_TUNERSLEEP) failed.");
        return ret;
    }

    /* Basic DiSEqC init from reference (cx24116_diseqc_init) */
    uint8_t diseqc_cmd[8];
    diseqc_cmd[0x00] = CMD_LNBCONFIG;
    diseqc_cmd[0x01] = 0x00; diseqc_cmd[0x02] = 0x10; diseqc_cmd[0x03] = 0x00;
    diseqc_cmd[0x04] = 0x8f; diseqc_cmd[0x05] = 0x28;
    diseqc_cmd[0x06] = 0x01; /* toneburst enabled by default in ref */
    diseqc_cmd[0x07] = 0x01;
    ret = cx24116_cmd_execute_internal(i2c_addr, diseqc_cmd, 8);
    if (ret != 0) {
        ESP_LOGW(TAG_CX, "DiSEqC init (CMD_LNBCONFIG) failed during wakeup.");
        /* Continue, as wakeup might still be partially successful */
    }

    /* Set a default voltage, e.g., 13V, as done in reference cx24116_initfe */
    ESP_LOGI(TAG_CX, "Setting default LNB voltage to 13V after wakeup.");
    int voltage_ret = cx24116_set_voltage(i2c_addr, 0 /* 13V */);
    if (voltage_ret != 0) {
        ESP_LOGW(TAG_CX, "Failed to set default LNB voltage during wakeup.");
    }

    return ret; /* Return status of wakeup/diseqc_init, voltage set is best effort */ 
}

/* --- CLI Command Structures and Handlers --- */

static struct {
    struct arg_int *address;
    struct arg_end *end;
} cx_probe_args;

static struct {
    struct arg_int *address;
    struct arg_int *mpeg_clk_pol;
    struct arg_str *fw_path; /* Optional: path to firmware file */
    struct arg_end *end;
} cx_loadfw_args;

static struct {
    struct arg_int *address;
    struct arg_int *freq_khz;
    struct arg_int *sr_sps;
    struct arg_int *fec;
    struct arg_int *inv;
    struct arg_int *delsys;
    struct arg_int *mod;
    struct arg_int *pilot;
    struct arg_int *rolloff;
    struct arg_end *end;
} cx_tune_args;

static struct {
    struct arg_int *address;
    struct arg_end *end;
} cx_generic_addr_args; /* For status, snr, strength, sleep, wakeup */

static struct {
    struct arg_int *address;
    struct arg_int *voltage; /* 0 for 13V, 1 for 18V */
    struct arg_end *end;
} cx_voltage_args;

static struct {
    struct arg_int *address;
    struct arg_int *tone;    /* 0 for OFF, 1 for ON */
    struct arg_end *end;
} cx_tone_args;

/* --- Variables for Linux-style driver integration --- */
static struct dvb_frontend *g_linux_dvb_frontend = NULL;
static struct cx24116_config g_linux_cx24116_config;

/* Dummy I2C adapter for cx24116_attach.
 * The actual I2C operations within cx24116_linux.c must be adapted
 * to use ESP-IDF's I2C API, likely using demod_address from cx24116_config.
 */
static struct i2c_adapter g_linux_i2c_adapter_dummy; 

/* Arguments for tuner_init command */
static struct {
    struct arg_int *mpeg_clk_pol;
    struct arg_str *fw_path;
    struct arg_end *end;
} tuner_init_args;

/* Default firmware path for ESP-IDF filesystem */
#define CX24116_ESP_DEFAULT_FW_PATH "/storage/cx24116.fw"
/* Default MPEG clock polarity from Linux driver */
#define CX24116_DEFAULT_MPEG_CLK_POL 0x02


static int do_tuner_init_cmd(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&tuner_init_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, tuner_init_args.end, argv[0]);
        return 1;
    }

    uint8_t mpeg_clk_pol = tuner_init_args.mpeg_clk_pol->count > 0 ?
                           tuner_init_args.mpeg_clk_pol->ival[0] :
                           CX24116_DEFAULT_MPEG_CLK_POL;

    ESP_LOGI(TAG_CX, "Initializing tuner with Linux driver logic...");
    ESP_LOGI(TAG_CX, "I2C Addr: 0x%02X, MPEG Clk Pol: 0x%02X",
             CX24116_DEFAULT_ADDRESS, mpeg_clk_pol);

    /* Populate the global config for the Linux driver */
    g_linux_cx24116_config.demod_address = CX24116_DEFAULT_ADDRESS;
    g_linux_cx24116_config.mpg_clk_pos_pol = mpeg_clk_pol;
    g_linux_cx24116_config.i2c_wr_max = 0; /* ESP-IDF I2C driver handles chunking */
    g_linux_cx24116_config.reset_device = NULL; /* No specific reset GPIO for now */
    g_linux_cx24116_config.set_ts_params = NULL;
    g_linux_cx24116_config.set_lock_led = NULL;


    if (g_linux_dvb_frontend != NULL) {
        ESP_LOGW(TAG_CX, "Tuner already initialized. Re-initializing.");
        if (g_linux_dvb_frontend->ops.release) {
            g_linux_dvb_frontend->ops.release(g_linux_dvb_frontend);
        }
        g_linux_dvb_frontend = NULL;
    }

    g_linux_dvb_frontend = cx24116_attach(&g_linux_cx24116_config, &g_linux_i2c_adapter_dummy);

    if (g_linux_dvb_frontend != NULL) {
        ESP_LOGI(TAG_CX, "cx24116_attach successful.");
        if (g_linux_dvb_frontend->ops.init) {
            ESP_LOGI(TAG_CX, "Calling frontend init (firmware load)...");
            int ret = g_linux_dvb_frontend->ops.init(g_linux_dvb_frontend);
            if (ret == 0) {
                ESP_LOGI(TAG_CX, "Tuner firmware initialization successful.");
            } else {
                ESP_LOGE(TAG_CX, "Tuner firmware initialization failed (ret: %d).", ret);
                /* Clean up frontend if init failed */
                if (g_linux_dvb_frontend->ops.release) {
                    g_linux_dvb_frontend->ops.release(g_linux_dvb_frontend);
                }
                g_linux_dvb_frontend = NULL;
                return 1;
            }
        } else {
            ESP_LOGE(TAG_CX, "Frontend init function not available.");
            return 1;
        }
    } else {
        ESP_LOGE(TAG_CX, "cx24116_attach failed.");
        return 1;
    }
    return 0;
}


void register_cx24116_commands(void) {
    cx_probe_args.address = arg_int0("a", "addr", "<i2c_addr>", "I2C address of CX24116 (default: 0x55)");
    cx_probe_args.end = arg_end(1);
    const esp_console_cmd_t cx_probe_cmd = {
        .command = "cx_probe",
        .help = "Probe for CX24116 device.",
        .hint = NULL,
        .func = &do_cx_probe_cmd,
        .argtable = &cx_probe_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cx_probe_cmd));

    cx_loadfw_args.address = arg_int0("a", "addr", "<i2c_addr>", "I2C address (default: 0x55)");
    cx_loadfw_args.mpeg_clk_pol = arg_int1("p", "clkpol", "<pol>", "MPEG Clock Polarity (e.g., 0x02)");
    cx_loadfw_args.fw_path = arg_str0(NULL, "path", "<path>", "Firmware file path (default: /storage/cx24116.fw)");
    cx_loadfw_args.end = arg_end(3);
    const esp_console_cmd_t cx_loadfw_cmd = {
        .command = "cx_loadfw",
        .help = "Load firmware to CX24116.",
        .hint = NULL,
        .func = &do_cx_loadfw_cmd,
        .argtable = &cx_loadfw_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cx_loadfw_cmd));

    cx_tune_args.address = arg_int0("a", "addr", "<i2c_addr>", "I2C address (default: 0x55)");
    cx_tune_args.freq_khz = arg_int1("f", "freq", "<kHz>", "Frequency in kHz");
    cx_tune_args.sr_sps = arg_int1("s", "sr", "<Sps>", "Symbol Rate in Sps");
    cx_tune_args.fec = arg_int1(NULL, "fec", "<val>", "FEC (0:NONE/AUTO, 1:1/2, ... see cx24116_fec_t)");
    cx_tune_args.inv = arg_int1(NULL, "inv", "<val>", "Inversion (0:OFF, 1:ON, 2:AUTO)");
    cx_tune_args.delsys = arg_int1(NULL, "delsys", "<val>", "Delivery System (0:DVBS, 1:DVBS2)");
    cx_tune_args.mod = arg_int1(NULL, "mod", "<val>", "Modulation (0:QPSK, 1:8PSK)");
    cx_tune_args.pilot = arg_int1(NULL, "pilot", "<val>", "Pilot (0:OFF, 1:ON, 2:AUTO)");
    cx_tune_args.rolloff = arg_int1(NULL, "rolloff", "<val>", "Rolloff (0:0.20, 1:0.25, 2:0.35)");
    cx_tune_args.end = arg_end(9);
    const esp_console_cmd_t cx_tune_cmd = {
        .command = "cx_tune",
        .help = "Tune CX24116. Enum values in cx24116.h",
        .hint = NULL,
        .func = &do_cx_tune_cmd,
        .argtable = &cx_tune_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cx_tune_cmd));

    cx_generic_addr_args.address = arg_int0("a", "addr", "<i2c_addr>", "I2C address (default: 0x55)");
    cx_generic_addr_args.end = arg_end(1);
    
    const esp_console_cmd_t cx_status_cmd = {
        .command = "cx_status", .help = "Read CX24116 status.", .hint = NULL,
        .func = &do_cx_status_cmd, .argtable = &cx_generic_addr_args };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cx_status_cmd));
    
    const esp_console_cmd_t cx_snr_cmd = {
        .command = "cx_snr", .help = "Read CX24116 SNR.", .hint = NULL,
        .func = &do_cx_snr_cmd, .argtable = &cx_generic_addr_args };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cx_snr_cmd));
    
    const esp_console_cmd_t cx_strength_cmd = {
        .command = "cx_strength", .help = "Read CX24116 signal strength.", .hint = NULL,
        .func = &do_cx_strength_cmd, .argtable = &cx_generic_addr_args };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cx_strength_cmd));
    
    const esp_console_cmd_t cx_sleep_cmd = {
        .command = "cx_sleep", .help = "Put CX24116 to sleep.", .hint = NULL,
        .func = &do_cx_sleep_cmd, .argtable = &cx_generic_addr_args };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cx_sleep_cmd));
    
    const esp_console_cmd_t cx_wakeup_cmd = {
        .command = "cx_wakeup", .help = "Wake up CX24116.", .hint = NULL,
        .func = &do_cx_wakeup_cmd, .argtable = &cx_generic_addr_args };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cx_wakeup_cmd));

    cx_voltage_args.address = arg_int0("a", "addr", "<i2c_addr>", "I2C address (default: 0x55)");
    cx_voltage_args.voltage = arg_int1("v", "volt", "<0|1>", "Voltage (0 for 13V, 1 for 18V)");
    cx_voltage_args.end = arg_end(2);
    const esp_console_cmd_t cx_voltage_cmd = {
        .command = "cx_voltage", .help = "Set LNB voltage.", .hint = NULL,
        .func = &do_cx_voltage_cmd, .argtable = &cx_voltage_args };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cx_voltage_cmd));

    cx_tone_args.address = arg_int0("a", "addr", "<i2c_addr>", "I2C address (default: 0x55)");
    cx_tone_args.tone = arg_int1("t", "tone", "<0|1>", "Tone (0 for OFF, 1 for ON)");
    cx_tone_args.end = arg_end(2);
    const esp_console_cmd_t cx_tone_cmd = {
        .command = "cx_tone", .help = "Set LNB tone.", .hint = NULL,
        .func = &do_cx_tone_cmd, .argtable = &cx_tone_args };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cx_tone_cmd));

    /* Register tuner_init command */
    tuner_init_args.mpeg_clk_pol = arg_int0("p", "clkpol", "<pol>", "MPEG Clock Polarity (default: 0x02)");
    tuner_init_args.end = arg_end(2);
    const esp_console_cmd_t tuner_init_cmd = {
        .command = "tuner_init",
        .help = "Initialize CX24116 tuner using Linux driver logic and load firmware.",
        .hint = NULL,
        .func = &do_tuner_init_cmd,
        .argtable = &tuner_init_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&tuner_init_cmd));
}