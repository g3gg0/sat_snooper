/*
 * CX24116/CX24118 DVB-S/S2 Frontend Driver Implementation
 *
 * lnb: https://docs.televes.com/DS11180009_004_54728.pdf
 */
#include "tuner.h"
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
#include "voltage.h"
#include "led.h"

#include "gpio_config.h"
#include "driver/gpio.h"

static const char *TAG_CX = "cx24116";
static bool tuner_initialized = false;
static bool tuner_reset_active = true;

// Define common LO frequencies and the band switch frequency.
#define LO_FREQ_LOW_BAND 9750
#define LO_FREQ_HIGH_BAND 10600
#define KU_BAND_SWITCH_FREQ 11700
#define KU_BAND_MIN_FREQ 10700
#define KU_BAND_MAX_FREQ 12750

#define CX24116_DEFAULT_ADDRESS 0x55
#define CX24116_DEFAULT_MPEG_CLK_POL 0x00

/* --- Variables for Linux-style driver integration --- */
static struct dvb_frontend *g_linux_dvb_frontend = NULL;
static struct cx24116_config g_linux_cx24116_config;

/* Dummy I2C adapter for cx24116_attach.
 * The actual I2C operations within cx24116_linux.c must be adapted
 * to use ESP-IDF's I2C API, likely using demod_address from cx24116_config.
 */
static struct i2c_adapter g_linux_i2c_adapter_dummy;

/* Arguments for tuner_tune command */
static struct
{
    struct arg_int *freq_mhz;
    struct arg_int *sr_ksps;
    struct arg_int *fec;
    struct arg_int *inv;
    struct arg_int *delsys;
    struct arg_int *mod;
    struct arg_int *pilot;
    struct arg_int *rolloff;
    struct arg_end *end;
} tuner_tune_cmd_args;

/* Arguments for tuner_status command (none needed beyond default) */
static struct
{
    struct arg_end *end;
} tuner_status_cmd_args;

/* Arguments for tuner_reset command */
static struct
{
    struct arg_int *level;
    struct arg_end *end;
} tuner_reset_cmd_args;

/* Arguments for tuner_tone command */
static struct
{
    struct arg_int *tone; /* 0 for OFF, 1 for ON */
    struct arg_end *end;
} tuner_tone_cmd_args;

/* Arguments for tuner_tx command */
static struct
{
    struct arg_str *data; /* Hex string of 3-6 bytes */
    struct arg_end *end;
} tuner_tx_cmd_args;

/* Arguments for tuner_burst command */
static struct
{
    struct arg_str *burst; /* "a" or "b" */
    struct arg_end *end;
} tuner_burst_cmd_args;

/* Arguments for tuner_voltage command */
static struct
{
    struct arg_int *voltage; /* 0 for 13V, 1 for 18V */
    struct arg_end *end;
} tuner_voltage_cmd_args;

/**
 * @brief Converts a Ku-band frequency to an L-band frequency, auto-selecting the LO.
 *
 * This function simulates a Universal LNB by automatically selecting the correct
 * Local Oscillator (LO) frequency based on the Ku-band frequency.
 * - For Ku-band frequencies < 11700 MHz, it uses the 9750 MHz LO.
 * - For Ku-band frequencies >= 11700 MHz, it uses the 10600 MHz LO.
 * The calculated L-band frequency is returned via a pointer.
 *
 * @param ku_band_freq_mhz The incoming Ku-band frequency in Megahertz (MHz).
 * The valid range is typically 10700 MHz to 12750 MHz.
 * @param l_band_freq_mhz_out Pointer to a uint32_t variable where the calculated
 * L-band frequency in MHz will be stored.
 *
 * @return Returns 'true' if the high-band LO (10600 MHz) was used.
 * Returns 'false' if the low-band LO was used or if an error occurred.
 */
static bool convert_ku_to_l_band(uint32_t ku_band_freq_mhz, uint32_t *l_band_freq_mhz_out)
{
    // --- Input Validation ---
    if (l_band_freq_mhz_out == NULL)
    {
        return false;
    }

    // Initialize the output to a safe value.
    *l_band_freq_mhz_out = 0;

    // Check if the Ku frequency is within the expected range.
    if (ku_band_freq_mhz < KU_BAND_MIN_FREQ || ku_band_freq_mhz > KU_BAND_MAX_FREQ)
    {
        return false;
    }

    // --- LO Selection and Conversion Logic ---
    uint32_t lo_freq_mhz;
    bool is_high_band;

    if (ku_band_freq_mhz >= KU_BAND_SWITCH_FREQ)
    {
        // High-band frequency
        lo_freq_mhz = LO_FREQ_HIGH_BAND;
        is_high_band = true;
    }
    else
    {
        // Low-band frequency
        lo_freq_mhz = LO_FREQ_LOW_BAND;
        is_high_band = false;
    }

    // The L-band frequency is the absolute difference.
    // In a universal LNB, the Ku frequency is always higher than the selected LO.
    *l_band_freq_mhz_out = ku_band_freq_mhz - lo_freq_mhz;

    // Return whether the high-band LO was used.
    return is_high_band;
}

static int tuner_reset(struct dvb_frontend *fe)
{
    (void)fe;
    tuner_set_reset(true);
    vTaskDelay(pdMS_TO_TICKS(20));
    tuner_set_reset(false);

    return 0;
}

static int do_tuner_status_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&tuner_status_cmd_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, tuner_status_cmd_args.end, argv[0]);
        return 1;
    }

    if (!g_linux_dvb_frontend)
    {
        ESP_LOGE(TAG_CX, "Tuner not initialized. Call tuner_init first.");
        return 1;
    }

    enum fe_status status = 0;
    u16 snr = 0;
    u16 strength = 0;
    u32 ber = 0;
    u32 ucblocks = 0;
    int ret;

    ESP_LOGI(TAG_CX, "--- Tuner Status (Linux Driver Model) ---");

    if (g_linux_dvb_frontend->ops.read_status)
    {
        ret = g_linux_dvb_frontend->ops.read_status(g_linux_dvb_frontend, &status);
        if (ret == 0)
        {
            ESP_LOGI(TAG_CX, "Status Flags: 0x%02X", status);
            ESP_LOGI(TAG_CX, "  FE_HAS_SIGNAL:  %s", (status & FE_HAS_SIGNAL) ? "Yes" : "No");
            ESP_LOGI(TAG_CX, "  FE_HAS_CARRIER: %s", (status & FE_HAS_CARRIER) ? "Yes" : "No");
            ESP_LOGI(TAG_CX, "  FE_HAS_VITERBI: %s", (status & FE_HAS_VITERBI) ? "Yes" : "No");
            ESP_LOGI(TAG_CX, "  FE_HAS_SYNC:    %s", (status & FE_HAS_SYNC) ? "Yes" : "No");
            ESP_LOGI(TAG_CX, "  FE_HAS_LOCK:    %s", (status & FE_HAS_LOCK) ? "Yes" : "No");
        }
        else
        {
            ESP_LOGE(TAG_CX, "Failed to read status (ret: %d)", ret);
        }
    }
    else
    {
        ESP_LOGW(TAG_CX, "read_status operation not available.");
    }

    if (g_linux_dvb_frontend->ops.read_snr)
    {
        ret = g_linux_dvb_frontend->ops.read_snr(g_linux_dvb_frontend, &snr);
        if (ret == 0)
        {
            ESP_LOGI(TAG_CX, "SNR: 0x%04X (%u)", snr, snr);
        }
        else
        {
            ESP_LOGE(TAG_CX, "Failed to read SNR (ret: %d)", ret);
        }
    }
    else
    {
        ESP_LOGW(TAG_CX, "read_snr operation not available.");
    }

    if (g_linux_dvb_frontend->ops.read_signal_strength)
    {
        ret = g_linux_dvb_frontend->ops.read_signal_strength(g_linux_dvb_frontend, &strength);
        if (ret == 0)
        {
            ESP_LOGI(TAG_CX, "Signal Strength: 0x%04X (%u)", strength, strength);
        }
        else
        {
            ESP_LOGE(TAG_CX, "Failed to read signal strength (ret: %d)", ret);
        }
    }
    else
    {
        ESP_LOGW(TAG_CX, "read_signal_strength operation not available.");
    }

    if (g_linux_dvb_frontend->ops.read_ber)
    {
        ret = g_linux_dvb_frontend->ops.read_ber(g_linux_dvb_frontend, &ber);
        if (ret == 0)
        {
            ESP_LOGI(TAG_CX, "BER: %lu", ber);
        }
        else
        {
            ESP_LOGE(TAG_CX, "Failed to read BER (ret: %d)", ret);
        }
    }
    else
    {
        ESP_LOGW(TAG_CX, "read_ber operation not available.");
    }

    if (g_linux_dvb_frontend->ops.read_ucblocks)
    {
        ret = g_linux_dvb_frontend->ops.read_ucblocks(g_linux_dvb_frontend, &ucblocks);
        if (ret == 0)
        {
            ESP_LOGI(TAG_CX, "Uncorrected Blocks: %lu", ucblocks);
        }
        else
        {
            ESP_LOGE(TAG_CX, "Failed to read uncorrected blocks (ret: %d)", ret);
        }
    }
    else
    {
        ESP_LOGW(TAG_CX, "read_ucblocks operation not available.");
    }

    ESP_LOGI(TAG_CX, "-----------------------------------------");

    return 0;
}

esp_err_t tuner_initialize(void)
{
    tuner_set_reset(false);
    tuner_initialized = false;
    led_set_tuner_status(false, false, false);

    ESP_LOGI(TAG_CX, "Initializing tuner with Linux driver logic...");
    ESP_LOGI(TAG_CX, "I2C Addr: 0x%02X", CX24116_DEFAULT_ADDRESS);

    /* Populate the global config for the Linux driver */
    g_linux_cx24116_config.demod_address = CX24116_DEFAULT_ADDRESS;
    g_linux_cx24116_config.mpg_clk_pos_pol = CX24116_DEFAULT_MPEG_CLK_POL;
    g_linux_cx24116_config.i2c_wr_max = 128;
    g_linux_cx24116_config.reset_device = &tuner_reset;
    g_linux_cx24116_config.set_ts_params = NULL;
    g_linux_cx24116_config.set_lock_led = NULL;
    /* Ensure firmware_path is correctly set if cx24116_linux.c's request_firmware needs it. */
    /* g_linux_cx24116_config.firmware_path = CX24116_ESP_DEFAULT_FW_PATH; // Example if fw_path arg was restored */

    if (g_linux_dvb_frontend != NULL)
    {
        ESP_LOGW(TAG_CX, "Tuner already initialized. Re-initializing.");
        if (g_linux_dvb_frontend->ops.release)
        {
            g_linux_dvb_frontend->ops.release(g_linux_dvb_frontend);
        }
        g_linux_dvb_frontend = NULL;
    }

    g_linux_dvb_frontend = cx24116_attach(&g_linux_cx24116_config, &g_linux_i2c_adapter_dummy);

    if (g_linux_dvb_frontend != NULL)
    {
        ESP_LOGI(TAG_CX, "cx24116_attach successful.");
        if (g_linux_dvb_frontend->ops.init)
        {
            ESP_LOGI(TAG_CX, "Calling frontend init (firmware load)...");
            int ret = g_linux_dvb_frontend->ops.init(g_linux_dvb_frontend);
            if (ret == 0)
            {
                ESP_LOGI(TAG_CX, "Tuner firmware initialization successful.");
            }
            else
            {
                ESP_LOGE(TAG_CX, "Tuner firmware initialization failed (ret: %d).", ret);
                /* Clean up frontend if init failed */
                if (g_linux_dvb_frontend->ops.release)
                {
                    g_linux_dvb_frontend->ops.release(g_linux_dvb_frontend);
                }
                g_linux_dvb_frontend = NULL;
                return ESP_FAIL;
            }
        }
        else
        {
            ESP_LOGE(TAG_CX, "Frontend init function not available.");
            return ESP_ERR_NOT_SUPPORTED;
        }
    }
    else
    {
        ESP_LOGE(TAG_CX, "cx24116_attach failed.");
        return ESP_FAIL;
    }
    tuner_initialized = true;
    led_set_tuner_status(true, false, false);
    return ESP_OK;
}

static int do_tuner_init_cmd(int argc, char **argv)
{
    return tuner_initialize() == ESP_OK ? 0 : 1;
}

static int do_tuner_tune_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&tuner_tune_cmd_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, tuner_tune_cmd_args.end, argv[0]);
        return 1;
    }

    const tuner_tune_request_t request = {
        .frequency_mhz = tuner_tune_cmd_args.freq_mhz->ival[0],
        .symbol_rate_ksps = tuner_tune_cmd_args.sr_ksps->ival[0],
        .polarization = voltage_get_current() == VOLTAGE_18V
                            ? TUNER_POLARIZATION_HORIZONTAL
                            : TUNER_POLARIZATION_VERTICAL,
        .fec = (tuner_fec_t)tuner_tune_cmd_args.fec->ival[0],
        .inversion = (tuner_inversion_t)tuner_tune_cmd_args.inv->ival[0],
        .delivery_system = (tuner_delivery_system_t)tuner_tune_cmd_args.delsys->ival[0],
        .modulation = (tuner_modulation_t)tuner_tune_cmd_args.mod->ival[0],
        .pilot = (tuner_pilot_t)tuner_tune_cmd_args.pilot->ival[0],
        .rolloff = (tuner_rolloff_t)tuner_tune_cmd_args.rolloff->ival[0],
    };

    return tuner_tune(&request) == ESP_OK ? 0 : 1;
}

void tuner_set_reset(bool active)
{
    tuner_reset_active = active;
    gpio_set_level(GPIO_TUNER_RESET, tuner_reset_active ? 0 : 1);
    if (active)
    {
        led_set_tuner_status(tuner_initialized, false, false);
    }
}

bool tuner_get_reset(void)
{
    return tuner_reset_active;
}

static int do_tuner_reset_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&tuner_reset_cmd_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, tuner_reset_cmd_args.end, argv[0]);
        return 1;
    }

    if (tuner_reset_cmd_args.level->count == 0)
    {
        ESP_LOGE(TAG_CX, "Missing level argument (0 or 1) for tuner_reset.");
        return 1;
    }

    int level = tuner_reset_cmd_args.level->ival[0];

    if (level != 0 && level != 1)
    {
        ESP_LOGE(TAG_CX, "Invalid level for tuner_reset. Must be 0 or 1.");
        return 1;
    }

    ESP_LOGI(TAG_CX, "Setting GPIO_TUNER_RESET to %d", level);
    tuner_set_reset(level);
    ESP_LOGI(TAG_CX, "GPIO_TUNER_RESET set to %d", level);

    return 0;
}

static int do_tuner_tone_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&tuner_tone_cmd_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, tuner_tone_cmd_args.end, argv[0]);
        return 1;
    }

    if (!g_linux_dvb_frontend)
    {
        ESP_LOGE(TAG_CX, "Tuner not initialized. Call tuner_init first.");
        return 1;
    }

    if (tuner_tone_cmd_args.tone->count == 0)
    {
        ESP_LOGE(TAG_CX, "Missing tone argument (0 or 1) for tuner_tone.");
        return 1;
    }

    int tone_val = tuner_tone_cmd_args.tone->ival[0];
    enum fe_sec_tone_mode tone_mode;

    if (tone_val == 0)
    {
        tone_mode = SEC_TONE_OFF;
        ESP_LOGI(TAG_CX, "Setting tuner tone OFF");
    }
    else if (tone_val == 1)
    {
        tone_mode = SEC_TONE_ON;
        ESP_LOGI(TAG_CX, "Setting tuner tone ON");
    }
    else
    {
        ESP_LOGE(TAG_CX, "Invalid tone value. Must be 0 (OFF) or 1 (ON).");
        return 1;
    }

    if (g_linux_dvb_frontend->ops.set_tone)
    {
        int ret = g_linux_dvb_frontend->ops.set_tone(g_linux_dvb_frontend, tone_mode);
        if (ret == 0)
        {
            ESP_LOGI(TAG_CX, "Tuner tone set successfully.");
        }
        else
        {
            ESP_LOGE(TAG_CX, "Failed to set tuner tone (ret: %d).", ret);
            return 1;
        }
    }
    else
    {
        ESP_LOGE(TAG_CX, "set_tone operation not available.");
        return 1;
    }
    return 0;
}

static int do_tuner_tx_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&tuner_tx_cmd_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, tuner_tx_cmd_args.end, argv[0]);
        return 1;
    }

    if (!g_linux_dvb_frontend)
    {
        ESP_LOGE(TAG_CX, "Tuner not initialized. Call tuner_init first.");
        return 1;
    }

    if (tuner_tx_cmd_args.data->count == 0)
    {
        ESP_LOGE(TAG_CX, "Missing data argument for tuner_tx.");
        return 1;
    }

    const char *hex_data_str = tuner_tx_cmd_args.data->sval[0];
    size_t len = strlen(hex_data_str);

    if (len < 6 || len > 12 || len % 2 != 0)
    {
        ESP_LOGE(TAG_CX, "Invalid hex data string length. Must be 6 to 12 hex characters (3-6 bytes).");
        return 1;
    }

    struct dvb_diseqc_master_cmd cmd;
    cmd.msg_len = len / 2;

    for (size_t i = 0; i < cmd.msg_len; ++i)
    {
        char byte_str[3] = {hex_data_str[i * 2], hex_data_str[i * 2 + 1], '\0'};
        if (sscanf(byte_str, "%2hhx", &cmd.msg[i]) != 1)
        {
            ESP_LOGE(TAG_CX, "Invalid hex character in data string: %s", byte_str);
            return 1;
        }
    }

    ESP_LOGI(TAG_CX, "Sending DiSEqC master command (len: %d):", cmd.msg_len);
    for (int i = 0; i < cmd.msg_len; i++)
        ESP_LOGI(TAG_CX, "  0x%02X", cmd.msg[i]);

    if (g_linux_dvb_frontend->ops.diseqc_send_master_cmd)
    {
        int ret = g_linux_dvb_frontend->ops.diseqc_send_master_cmd(g_linux_dvb_frontend, &cmd);
        if (ret == 0)
        {
            ESP_LOGI(TAG_CX, "DiSEqC command sent successfully.");
        }
        else
        {
            ESP_LOGE(TAG_CX, "Failed to send DiSEqC command (ret: %d).", ret);
            return 1;
        }
    }
    else
    {
        ESP_LOGE(TAG_CX, "diseqc_send_master_cmd operation not available.");
        return 1;
    }
    return 0;
}

static int do_tuner_burst_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&tuner_burst_cmd_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, tuner_burst_cmd_args.end, argv[0]);
        return 1;
    }

    if (!g_linux_dvb_frontend)
    {
        ESP_LOGE(TAG_CX, "Tuner not initialized. Call tuner_init first.");
        return 1;
    }

    if (tuner_burst_cmd_args.burst->count == 0)
    {
        ESP_LOGE(TAG_CX, "Missing burst argument ('a' or 'b') for tuner_burst.");
        return 1;
    }

    const char *burst_str = tuner_burst_cmd_args.burst->sval[0];
    enum fe_sec_mini_cmd burst_cmd;

    if (strcmp(burst_str, "a") == 0 || strcmp(burst_str, "A") == 0)
    {
        burst_cmd = SEC_MINI_A;
        ESP_LOGI(TAG_CX, "Sending ToneBurst A");
    }
    else if (strcmp(burst_str, "b") == 0 || strcmp(burst_str, "B") == 0)
    {
        burst_cmd = SEC_MINI_B;
        ESP_LOGI(TAG_CX, "Sending ToneBurst B");
    }
    else
    {
        ESP_LOGE(TAG_CX, "Invalid burst argument. Must be 'a' or 'b'.");
        return 1;
    }

    if (g_linux_dvb_frontend->ops.diseqc_send_burst)
    {
        int ret = g_linux_dvb_frontend->ops.diseqc_send_burst(g_linux_dvb_frontend, burst_cmd);
        if (ret == 0)
        {
            ESP_LOGI(TAG_CX, "ToneBurst sent successfully.");
        }
        else
        {
            ESP_LOGE(TAG_CX, "Failed to send ToneBurst (ret: %d).", ret);
            return 1;
        }
    }
    else
    {
        ESP_LOGE(TAG_CX, "diseqc_send_burst operation not available.");
        return 1;
    }
    return 0;
}

static int do_tuner_voltage_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&tuner_voltage_cmd_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, tuner_voltage_cmd_args.end, argv[0]);
        return 1;
    }

    if (!g_linux_dvb_frontend)
    {
        ESP_LOGE(TAG_CX, "Tuner not initialized. Call tuner_init first.");
        return 1;
    }

    if (tuner_voltage_cmd_args.voltage->count == 0)
    {
        ESP_LOGE(TAG_CX, "Missing voltage argument (0 or 1) for tuner_voltage.");
        return 1;
    }

    int voltage_val = tuner_voltage_cmd_args.voltage->ival[0];
    enum fe_sec_voltage voltage_level;

    if (voltage_val == 0)
    {
        voltage_level = SEC_VOLTAGE_13;
        ESP_LOGI(TAG_CX, "Setting tuner LNB voltage to 13V");
    }
    else if (voltage_val == 1)
    {
        voltage_level = SEC_VOLTAGE_18;
        ESP_LOGI(TAG_CX, "Setting tuner LNB voltage to 18V");
    }
    else
    {
        ESP_LOGE(TAG_CX, "Invalid voltage value. Must be 0 (13V) or 1 (18V).");
        return 1;
    }

    if (g_linux_dvb_frontend->ops.set_voltage)
    {
        int ret = g_linux_dvb_frontend->ops.set_voltage(g_linux_dvb_frontend, voltage_level);
        if (ret == 0)
        {
            ESP_LOGI(TAG_CX, "Tuner LNB voltage set successfully.");
        }
        else
        {
            ESP_LOGE(TAG_CX, "Failed to set tuner LNB voltage (ret: %d).", ret);
            return 1;
        }
    }
    else
    {
        ESP_LOGE(TAG_CX, "set_voltage operation not available.");
        return 1;
    }
    return 0;
}

static bool decode_acquired_mode(uint8_t fec_status, tuner_status_t *status)
{
    uint8_t mode = fec_status & 0x1f;
    status->acquired_pilot_on = (fec_status & 0x80) != 0;

    if ((fec_status & 0x20) != 0)
    {
        if (mode < TUNER_FEC_1_2 || mode > TUNER_FEC_8_9)
        {
            return false;
        }
        status->acquired_delivery_system = TUNER_DELIVERY_DVB_S;
        status->acquired_modulation = TUNER_MODULATION_QPSK;
        status->acquired_fec = (tuner_fec_t)mode;
        return true;
    }

    status->acquired_delivery_system = TUNER_DELIVERY_DVB_S2;
    switch (mode)
    {
    case 0x04:
        status->acquired_modulation = TUNER_MODULATION_QPSK;
        status->acquired_fec = TUNER_FEC_1_2;
        return true;
    case 0x05:
    case 0x0c:
        status->acquired_modulation = mode == 0x05 ? TUNER_MODULATION_QPSK : TUNER_MODULATION_8PSK;
        status->acquired_fec = TUNER_FEC_3_5;
        return true;
    case 0x06:
    case 0x0d:
        status->acquired_modulation = mode == 0x06 ? TUNER_MODULATION_QPSK : TUNER_MODULATION_8PSK;
        status->acquired_fec = TUNER_FEC_2_3;
        return true;
    case 0x07:
    case 0x0e:
        status->acquired_modulation = mode == 0x07 ? TUNER_MODULATION_QPSK : TUNER_MODULATION_8PSK;
        status->acquired_fec = TUNER_FEC_3_4;
        return true;
    case 0x08:
        status->acquired_modulation = TUNER_MODULATION_QPSK;
        status->acquired_fec = TUNER_FEC_4_5;
        return true;
    case 0x09:
    case 0x0f:
        status->acquired_modulation = mode == 0x09 ? TUNER_MODULATION_QPSK : TUNER_MODULATION_8PSK;
        status->acquired_fec = TUNER_FEC_5_6;
        return true;
    case 0x0a:
    case 0x10:
        status->acquired_modulation = mode == 0x0a ? TUNER_MODULATION_QPSK : TUNER_MODULATION_8PSK;
        status->acquired_fec = TUNER_FEC_8_9;
        return true;
    case 0x0b:
    case 0x11:
        status->acquired_modulation = mode == 0x0b ? TUNER_MODULATION_QPSK : TUNER_MODULATION_8PSK;
        status->acquired_fec = TUNER_FEC_9_10;
        return true;
    default:
        return false;
    }
}

static void read_tuner_diagnostics(tuner_status_t *status)
{
    struct cx24116_diagnostics diagnostics = {0};
    if (cx24116_get_diagnostics(g_linux_dvb_frontend, &diagnostics) != 0)
    {
        return;
    }

    status->chip_id_available = true;
    status->chip_id = diagnostics.chip_id;
    status->firmware_version_available = diagnostics.firmware_version_valid;
    memcpy(status->firmware_version, diagnostics.firmware_version,
           sizeof(status->firmware_version));

    if (diagnostics.firmware_status >= 0)
    {
        status->firmware_running_available = true;
        status->firmware_running = diagnostics.firmware_status == 0;
    }
    if (diagnostics.qstatus >= 0)
    {
        status->lnb_ready_available = true;
        status->lnb_ready = (diagnostics.qstatus & 0x20) != 0;
    }
    if (diagnostics.quality8 >= 0 && diagnostics.quality0 >= 0)
    {
        status->esno_available = true;
        status->esno_tenth_db = ((uint16_t)diagnostics.quality8 << 8) |
                               (uint16_t)diagnostics.quality0;
    }
    if ((status->flags & TUNER_STATUS_LOCK) != 0 && diagnostics.fec_status >= 0)
    {
        status->acquired_mode_available = decode_acquired_mode(
            (uint8_t)diagnostics.fec_status, status);
    }
}

esp_err_t tuner_get_status(tuner_status_t *status)
{
    if (!status)
    {
        return ESP_ERR_INVALID_ARG;
    }

    memset(status, 0, sizeof(*status));

    if (!tuner_initialized || !g_linux_dvb_frontend)
    {
        ESP_LOGE(TAG_CX, "Tuner not initialized. Cannot get status.");
        return ESP_ERR_INVALID_STATE;
    }

    status->initialized = true;
    if (tuner_reset_active)
    {
        led_set_tuner_status(true, false, false);
        return ESP_OK;
    }

    int ret;
    enum fe_status fe_stat_val = 0;

    if (g_linux_dvb_frontend->ops.read_status)
    {
        ret = g_linux_dvb_frontend->ops.read_status(g_linux_dvb_frontend, &fe_stat_val);
        if (ret == 0)
        {
            status->flags = (uint32_t)fe_stat_val;
        }
        else
        {
            ESP_LOGE(TAG_CX, "Failed to read status (ret: %d)", ret);
            /* Continue to try and read other values if possible */
        }
    }
    else
    {
        ESP_LOGW(TAG_CX, "read_status operation not available.");
    }

    if (g_linux_dvb_frontend->ops.read_snr)
    {
        ret = g_linux_dvb_frontend->ops.read_snr(g_linux_dvb_frontend, &status->snr);
        if (ret == 0)
        {
            status->snr_available = true;
        }
        else
        {
            ESP_LOGE(TAG_CX, "Failed to read SNR (ret: %d)", ret);
        }
    }
    else
    {
        ESP_LOGW(TAG_CX, "read_snr operation not available.");
    }

    if (g_linux_dvb_frontend->ops.read_signal_strength)
    {
        ret = g_linux_dvb_frontend->ops.read_signal_strength(g_linux_dvb_frontend, &status->strength);
        if (ret == 0)
        {
            status->strength_available = true;
        }
        else
        {
            ESP_LOGE(TAG_CX, "Failed to read signal strength (ret: %d)", ret);
        }
    }
    else
    {
        ESP_LOGW(TAG_CX, "read_signal_strength operation not available.");
    }

    if (g_linux_dvb_frontend->ops.read_ber)
    {
        ret = g_linux_dvb_frontend->ops.read_ber(g_linux_dvb_frontend, &status->ber);
        if (ret == 0)
        {
            status->ber_available = true;
        }
        else
        {
            ESP_LOGE(TAG_CX, "Failed to read BER (ret: %d)", ret);
        }
    }
    else
    {
        ESP_LOGW(TAG_CX, "read_ber operation not available.");
    }

    if (g_linux_dvb_frontend->ops.read_ucblocks)
    {
        ret = g_linux_dvb_frontend->ops.read_ucblocks(g_linux_dvb_frontend, &status->uncorrected_blocks);
        if (ret == 0)
        {
            status->uncorrected_blocks_available = true;
        }
        else
        {
            ESP_LOGE(TAG_CX, "Failed to read uncorrected blocks (ret: %d)", ret);
        }
    }
    else
    {
        ESP_LOGW(TAG_CX, "read_ucblocks operation not available.");
    }

    led_set_tuner_status(true,
                         (status->flags & TUNER_STATUS_CARRIER) != 0,
                         (status->flags & TUNER_STATUS_VITERBI) != 0);
    read_tuner_diagnostics(status);

    return ESP_OK;
}

bool tuner_is_initialized(void)
{
    return tuner_initialized;
}

static bool tuner_request_is_valid(const tuner_tune_request_t *request)
{
    if (request->frequency_mhz < KU_BAND_MIN_FREQ || request->frequency_mhz > KU_BAND_MAX_FREQ ||
        request->symbol_rate_ksps < 1000 || request->symbol_rate_ksps > 45000)
    {
        return false;
    }

    if (request->polarization != TUNER_POLARIZATION_VERTICAL &&
        request->polarization != TUNER_POLARIZATION_HORIZONTAL)
    {
        return false;
    }

    if (request->fec < TUNER_FEC_NONE || request->fec > TUNER_FEC_2_5 ||
        request->inversion < TUNER_INVERSION_OFF || request->inversion > TUNER_INVERSION_AUTO ||
        request->pilot < TUNER_PILOT_ON || request->pilot > TUNER_PILOT_AUTO ||
        request->rolloff < TUNER_ROLLOFF_35 || request->rolloff > TUNER_ROLLOFF_AUTO)
    {
        return false;
    }

    if (request->delivery_system == TUNER_DELIVERY_DVB_S)
    {
        return request->modulation == TUNER_MODULATION_QPSK &&
               request->rolloff == TUNER_ROLLOFF_35 &&
               request->fec <= TUNER_FEC_AUTO;
    }

    if (request->delivery_system != TUNER_DELIVERY_DVB_S2 ||
        request->rolloff == TUNER_ROLLOFF_AUTO)
    {
        return false;
    }

    switch (request->fec)
    {
    case TUNER_FEC_3_5:
    case TUNER_FEC_2_3:
    case TUNER_FEC_3_4:
    case TUNER_FEC_5_6:
    case TUNER_FEC_8_9:
    case TUNER_FEC_9_10:
        return request->modulation == TUNER_MODULATION_QPSK ||
               request->modulation == TUNER_MODULATION_8PSK;
    case TUNER_FEC_1_2:
    case TUNER_FEC_4_5:
        return request->modulation == TUNER_MODULATION_QPSK;
    default:
        return false;
    }
}

esp_err_t tuner_tune(const tuner_tune_request_t *request)
{
    if (!request || !tuner_request_is_valid(request))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!tuner_initialized || !g_linux_dvb_frontend)
    {
        ESP_LOGE(TAG_CX, "Tuner not initialized. Call tuner_init first.");
        return ESP_ERR_INVALID_STATE;
    }

    led_set_tuner_status(true, false, false);

    voltage_select(request->polarization == TUNER_POLARIZATION_HORIZONTAL ? VOLTAGE_18V : VOLTAGE_13V);

    struct dtv_frontend_properties *c = &g_linux_dvb_frontend->dtv_property_cache;

    uint32_t freq = 0;
    bool high_band = convert_ku_to_l_band(request->frequency_mhz, &freq);
    if (freq == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Convert MHz to kHz and kSps to Sps for the dtv_property_cache */
    c->frequency = freq * 1000;
    c->sectone = high_band ? SEC_TONE_ON : SEC_TONE_OFF;
    c->symbol_rate = request->symbol_rate_ksps * 1000;

    c->fec_inner = (enum fe_code_rate)request->fec;
    c->inversion = (enum fe_spectral_inversion)request->inversion;
    c->delivery_system = (enum fe_delivery_system)request->delivery_system;
    c->modulation = (enum fe_modulation)request->modulation;
    c->pilot = (enum fe_pilot)request->pilot;
    c->rolloff = (enum fe_rolloff)request->rolloff;

    ESP_LOGI(TAG_CX, "Tune: Input_Ku=%lu MHz, Freq_L=%lu kHz, %s, SR=%lu Sps, DelSys=%d, Mod=%d, FEC=%d, Inv=%d, Pilot=%d, Rolloff=%d, sectone=%d",
             request->frequency_mhz, c->frequency, high_band ? "HI" : "LO", c->symbol_rate, c->delivery_system, c->modulation, c->fec_inner, c->inversion, c->pilot, c->rolloff, c->sectone);

    if (g_linux_dvb_frontend->ops.set_tone)
    {
        int ret = g_linux_dvb_frontend->ops.set_tone(g_linux_dvb_frontend, c->sectone);
        if (ret != 0)
        {
            ESP_LOGE(TAG_CX, "Failed to set tuner tone (ret: %d).", ret);
            return ESP_FAIL;
        }
    }

    if (g_linux_dvb_frontend->ops.tune)
    {
        unsigned int delay_ticks = 0;
        enum fe_status status_val = 0;

        ESP_LOGI(TAG_CX, "Calling frontend ops.tune...");
        int ret = g_linux_dvb_frontend->ops.tune(g_linux_dvb_frontend, true, 0, &delay_ticks, &status_val);

        if (ret == 0)
        {
            ESP_LOGI(TAG_CX, "Tune successful. FE Status: 0x%02x. Delay: %u ticks", status_val, delay_ticks);
            led_set_tuner_status(true,
                                 (status_val & FE_HAS_CARRIER) != 0,
                                 (status_val & FE_HAS_VITERBI) != 0);
            return ESP_OK;
        }
        else
        {
            ESP_LOGE(TAG_CX, "Tune failed (ret: %d)", ret);
            return ESP_FAIL;
        }
    }
    else
    {
        ESP_LOGE(TAG_CX, "Frontend tune function not available in ops table.");
        return ESP_ERR_NOT_SUPPORTED;
    }
    return ESP_FAIL;
}

void tuner_register_console(void)
{
    tuner_set_reset(true);

    /* Register tuner_init command */
    const esp_console_cmd_t tuner_init_cmd = {
        .command = "tuner_init",
        .help = "Initialize CX24116 tuner using Linux driver logic and load firmware.",
        .hint = NULL,
        .func = &do_tuner_init_cmd};
    ESP_ERROR_CHECK(esp_console_cmd_register(&tuner_init_cmd));

    /* Register tuner_tune command */
    tuner_tune_cmd_args.freq_mhz = arg_int1("f", "freq", "<MHz>", "Frequency in MHz");
    tuner_tune_cmd_args.sr_ksps = arg_int1("s", "sr", "<kSps>", "Symbol rate in kSym/s");
    tuner_tune_cmd_args.fec = arg_int1(NULL, "fec", "<val>", "FEC (see DVB API fe_code_rate)");
    tuner_tune_cmd_args.inv = arg_int1(NULL, "inv", "<val>", "Inversion (see DVB API fe_spectral_inversion)");
    tuner_tune_cmd_args.delsys = arg_int1(NULL, "delsys", "<val>", "Delivery System (see DVB API fe_delivery_system)");
    tuner_tune_cmd_args.mod = arg_int1(NULL, "mod", "<val>", "Modulation (see DVB API fe_modulation)");
    tuner_tune_cmd_args.pilot = arg_int1(NULL, "pilot", "<val>", "Pilot (see DVB API fe_pilot)");
    tuner_tune_cmd_args.rolloff = arg_int1(NULL, "rolloff", "<val>", "Rolloff (see DVB API fe_rolloff)");
    tuner_tune_cmd_args.end = arg_end(8);
    const esp_console_cmd_t tuner_tune_console_cmd = {/* Renamed to avoid conflict with cx_tune_cmd if it existed */
                                                      .command = "tuner_tune",
                                                      .help = "Tune CX24116 using Linux driver logic. Use DVB API enum values.",
                                                      .hint = NULL,
                                                      .func = &do_tuner_tune_cmd,
                                                      .argtable = &tuner_tune_cmd_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&tuner_tune_console_cmd));

    /* Register tuner_status command */
    tuner_status_cmd_args.end = arg_end(0);
    const esp_console_cmd_t tuner_status_console_cmd = {
        .command = "tuner_status",
        .help = "Read and display various status metrics from the CX24116 tuner (Linux model).",
        .hint = NULL,
        .func = &do_tuner_status_cmd,
        .argtable = &tuner_status_cmd_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&tuner_status_console_cmd));

    /* Register tuner_reset command */
    tuner_reset_cmd_args.level = arg_int1(NULL, NULL, "<0|1>", "Reset level (0 or 1)");
    tuner_reset_cmd_args.end = arg_end(1);
    const esp_console_cmd_t tuner_reset_console_cmd = {
        .command = "tuner_reset",
        .help = "Set the tuner reset GPIO pin to a specific level (0 or 1).",
        .hint = NULL,
        .func = &do_tuner_reset_cmd,
        .argtable = &tuner_reset_cmd_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&tuner_reset_console_cmd));

    /* Register tuner_tone command */
    tuner_tone_cmd_args.tone = arg_int1(NULL, "tone", "<0|1>", "Tone (0 for OFF, 1 for ON)");
    tuner_tone_cmd_args.end = arg_end(1);
    const esp_console_cmd_t tuner_tone_console_cmd = {
        .command = "tuner_tone",
        .help = "Set LNB tone using Linux driver ops (0=OFF, 1=ON).",
        .hint = NULL,
        .func = &do_tuner_tone_cmd,
        .argtable = &tuner_tone_cmd_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&tuner_tone_console_cmd));

    /* Register tuner_tx command */
    tuner_tx_cmd_args.data = arg_str1(NULL, "data", "<hexbytes>", "DiSEqC command bytes as a hex string (3-6 bytes, e.g., E01038F0)");
    tuner_tx_cmd_args.end = arg_end(1);
    const esp_console_cmd_t tuner_tx_console_cmd = {
        .command = "tuner_tx",
        .help = "Send DiSEqC master command using Linux driver ops.",
        .hint = NULL,
        .func = &do_tuner_tx_cmd,
        .argtable = &tuner_tx_cmd_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&tuner_tx_console_cmd));

    /* Register tuner_burst command */
    tuner_burst_cmd_args.burst = arg_str1(NULL, "burst", "<a|b>", "ToneBurst (A or B)");
    tuner_burst_cmd_args.end = arg_end(1);
    const esp_console_cmd_t tuner_burst_console_cmd = {
        .command = "tuner_burst",
        .help = "Send ToneBurst A or B using Linux driver ops.",
        .hint = NULL,
        .func = &do_tuner_burst_cmd,
        .argtable = &tuner_burst_cmd_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&tuner_burst_console_cmd));

    /* Register tuner_voltage command */
    tuner_voltage_cmd_args.voltage = arg_int1(NULL, "voltage", "<0|1>", "LNB Voltage (0 for 13V, 1 for 18V)");
    tuner_voltage_cmd_args.end = arg_end(1);
    const esp_console_cmd_t tuner_voltage_console_cmd = {
        .command = "tuner_voltage",
        .help = "Set LNB voltage using Linux driver ops (0=13V, 1=18V).",
        .hint = NULL,
        .func = &do_tuner_voltage_cmd,
        .argtable = &tuner_voltage_cmd_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&tuner_voltage_console_cmd));

}