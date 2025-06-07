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

#include "gpio_config.h"
#include "driver/gpio.h"

static const char *TAG_CX = "cx24116";
static bool tuner_initialized = false;
static bool tuner_reset_active = true;

#define CX24116_DEFAULT_ADDRESS 0x55
/* Default MPEG clock polarity from Linux driver */
#define CX24116_DEFAULT_MPEG_CLK_POL 0x02

/* --- Register Definitions (from cx24116-elixir.c) --- */
#define CX24116_REG_COMMAND (0x00)  /* command args 0x00..0x1e */
#define CX24116_REG_EXECUTE (0x1f)  /* execute command */
#define CX24116_REG_RESET (0x20)    /* reset status > 0     */
#define CX24116_REG_MAILBOX (0x96)  /* FW or multipurpose mailbox? */
#define CX24116_REG_SIGNAL (0x9e)   /* signal low           */
#define CX24116_REG_SSTATUS (0x9d)  /* signal high / status */
#define CX24116_REG_QUALITY8 (0xa3) /* SNR MSB (for EsNo mode) / Quality for S1 */
#define CX24116_REG_QSTATUS (0xbc)  /* LNB Ready, etc. */
#define CX24116_REG_QUALITY0 (0xd5) /* SNR LSB (for EsNo mode) / Quality for S1 */
#define CX24116_REG_BER0 (0xc9)
#define CX24116_REG_BER8 (0xc8)
#define CX24116_REG_BER16 (0xc7)
#define CX24116_REG_BER24 (0xc6)
#define CX24116_REG_UCB0 (0xcb)
#define CX24116_REG_UCB8 (0xca)
#define CX24116_REG_CLKDIV (0xf3)
#define CX24116_REG_RATEDIV (0xf9)
#define CX24116_REG_ID_MSB (0xFF) /* Device ID MSB */
#define CX24116_REG_ID_LSB (0xFE) /* Device ID LSB */

/* --- Command Definitions (from cx24116-elixir.c) --- */
enum cx24116_cmds
{
    CMD_SET_VCO = 0x10,
    CMD_TUNEREQUEST = 0x11,
    CMD_MPEGCONFIG = 0x13,
    CMD_TUNERINIT = 0x14,
    CMD_BANDWIDTH = 0x15,
    CMD_GETAGC = 0x19,
    CMD_LNBCONFIG = 0x20,
    CMD_LNBSEND = 0x21, /* Formerly CMD_SEND_DISEQC */
    CMD_LNBDCLEVEL = 0x22,
    CMD_SET_TONE = 0x23,
    CMD_UPDFWVERS = 0x35,
    CMD_TUNERSLEEP = 0x36,
    CMD_AGCCONTROL = 0x3b, /* Unknown */
};

#define CX24116_ARGLEN (0x1e)

/* --- Variables for Linux-style driver integration --- */
static struct dvb_frontend *g_linux_dvb_frontend = NULL;
static struct cx24116_config g_linux_cx24116_config;

/* Dummy I2C adapter for cx24116_attach.
 * The actual I2C operations within cx24116_linux.c must be adapted
 * to use ESP-IDF's I2C API, likely using demod_address from cx24116_config.
 */
static struct i2c_adapter g_linux_i2c_adapter_dummy;

/* Arguments for tuner_init command */
static struct
{
    struct arg_int *mpeg_clk_pol;
    struct arg_end *end;
} tuner_init_args;

/* Arguments for tuner_tune command */
static struct
{
    struct arg_int *freq_khz;
    struct arg_int *sr_sps;
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

int tuner_reset(struct dvb_frontend *fe)
{
    gpio_set_level(GPIO_TUNER_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(GPIO_TUNER_RESET, 1);

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

int tuner_init(uint8_t mpeg_clk_pol)
{
    if(tuner_reset_get())
    {
        return -1;
    }
    ESP_LOGI(TAG_CX, "Initializing tuner with Linux driver logic...");
    ESP_LOGI(TAG_CX, "I2C Addr: 0x%02X, MPEG Clk Pol: 0x%02X",
             CX24116_DEFAULT_ADDRESS, mpeg_clk_pol);

    /* Populate the global config for the Linux driver */
    g_linux_cx24116_config.demod_address = CX24116_DEFAULT_ADDRESS;
    g_linux_cx24116_config.mpg_clk_pos_pol = mpeg_clk_pol;
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
                return 1;
            }
        }
        else
        {
            ESP_LOGE(TAG_CX, "Frontend init function not available.");
            return 1;
        }
    }
    else
    {
        ESP_LOGE(TAG_CX, "cx24116_attach failed.");
        return 1;
    }
    return 0;
}

static int do_tuner_init_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&tuner_init_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, tuner_init_args.end, argv[0]);
        return 1;
    }

    uint8_t mpeg_clk_pol = tuner_init_args.mpeg_clk_pol->count > 0 ? tuner_init_args.mpeg_clk_pol->ival[0] : CX24116_DEFAULT_MPEG_CLK_POL;
    return tuner_init(mpeg_clk_pol);
}

static int do_tuner_tune_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&tuner_tune_cmd_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, tuner_tune_cmd_args.end, argv[0]);
        return 1;
    }

    if (!g_linux_dvb_frontend)
    {
        ESP_LOGE(TAG_CX, "Tuner not initialized. Call tuner_init first.");
        return 1;
    }

    struct dtv_frontend_properties *c = &g_linux_dvb_frontend->dtv_property_cache;

    c->frequency = tuner_tune_cmd_args.freq_khz->ival[0] * 1000;
    c->symbol_rate = tuner_tune_cmd_args.sr_sps->ival[0] * 1000;
    c->fec_inner = (enum fe_code_rate)tuner_tune_cmd_args.fec->ival[0];
    c->inversion = (enum fe_spectral_inversion)tuner_tune_cmd_args.inv->ival[0];
    c->delivery_system = (enum fe_delivery_system)tuner_tune_cmd_args.delsys->ival[0];
    c->modulation = (enum fe_modulation)tuner_tune_cmd_args.mod->ival[0];
    c->pilot = (enum fe_pilot)tuner_tune_cmd_args.pilot->ival[0];
    c->rolloff = (enum fe_rolloff)tuner_tune_cmd_args.rolloff->ival[0];
    /* Other dtv_frontend_properties like bandwidth_hz might be set by the driver or need defaults */

    ESP_LOGI(TAG_CX, "Populating DTV properties: Freq=%lu Hz, SR=%lu Sps, DelSys=%d, Mod=%d, FEC=%d, Inv=%d, Pilot=%d, Rolloff=%d",
             c->frequency, c->symbol_rate, c->delivery_system, c->modulation, c->fec_inner, c->inversion, c->pilot, c->rolloff);

    if (g_linux_dvb_frontend->ops.tune)
    {
        unsigned int delay_ticks = 0;
        enum fe_status status_val = 0;

        ESP_LOGI(TAG_CX, "Calling frontend ops.tune...");
        /* Call with re_tune = true to make set_frontend use the new DTV properties */
        int ret = g_linux_dvb_frontend->ops.tune(g_linux_dvb_frontend, true, 0, &delay_ticks, &status_val);

        if (ret == 0)
        {
            ESP_LOGI(TAG_CX, "tuner_tune command successful. FE Status: 0x%02x. Suggested delay (ticks): %u", status_val, delay_ticks);
            ESP_LOGI(TAG_CX, "FE_STATUS: Sig:%d Carr:%d Vit:%d Sync:%d Lock:%d",
                     (status_val & FE_HAS_SIGNAL) ? 1 : 0,
                     (status_val & FE_HAS_CARRIER) ? 1 : 0,
                     (status_val & FE_HAS_VITERBI) ? 1 : 0,
                     (status_val & FE_HAS_SYNC) ? 1 : 0,
                     (status_val & FE_HAS_LOCK) ? 1 : 0);
        }
        else
        {
            ESP_LOGE(TAG_CX, "tuner_tune command failed (ret: %d)", ret);
            return 1;
        }
    }
    else
    {
        ESP_LOGE(TAG_CX, "Frontend tune function not available in ops table.");
        return 1;
    }
    return 0;
}

void tuner_reset_set(bool state)
{
    tuner_reset_active = state;
    gpio_set_level(GPIO_TUNER_RESET, tuner_reset_active ? 0 : 1);
}

bool tuner_reset_get()
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
    tuner_reset_set(level);
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

/* Function to get current tuner status for web API */
esp_err_t tuner_get_status_data(tuner_status_t *status_data)
{
    if (!status_data || !tuner_initialized)
    {
        return ESP_ERR_INVALID_ARG;
    }

    memset(status_data, 0, sizeof(tuner_status_t));

    if (!g_linux_dvb_frontend)
    {
        ESP_LOGE(TAG_CX, "Tuner not initialized. Cannot get status.");
        status_data->initialized = false;
        return ESP_ERR_INVALID_STATE;
    }

    status_data->initialized = true;
    int ret;
    enum fe_status fe_stat_val = 0; /* Temporary variable for fe_status */

    if (g_linux_dvb_frontend->ops.read_status)
    {
        /* Cast the address of status_data->status_flags to enum fe_status* */
        /* This assumes tuner_status_t.status_flags is compatible in size and meaning with fe_status */
        ret = g_linux_dvb_frontend->ops.read_status(g_linux_dvb_frontend, &fe_stat_val);
        if (ret == 0)
        {
            status_data->status_flags = (uint32_t)fe_stat_val;
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
        ret = g_linux_dvb_frontend->ops.read_snr(g_linux_dvb_frontend, &status_data->snr);
        if (ret != 0)
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
        ret = g_linux_dvb_frontend->ops.read_signal_strength(g_linux_dvb_frontend, &status_data->strength);
        if (ret != 0)
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
        ret = g_linux_dvb_frontend->ops.read_ber(g_linux_dvb_frontend, &status_data->ber);
        if (ret != 0)
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
        ret = g_linux_dvb_frontend->ops.read_ucblocks(g_linux_dvb_frontend, &status_data->uc_blocks);
        if (ret != 0)
        {
            ESP_LOGE(TAG_CX, "Failed to read uncorrected blocks (ret: %d)", ret);
        }
    }
    else
    {
        ESP_LOGW(TAG_CX, "read_ucblocks operation not available.");
    }

    return ESP_OK;
}

/* Function to perform tuning based on parameters from web API */
esp_err_t tuner_web_tune(const tuner_tune_params_t *params)
{
    if (!params || !tuner_initialized)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_linux_dvb_frontend)
    {
        ESP_LOGE(TAG_CX, "Tuner not initialized. Call tuner_init first.");
        return ESP_ERR_INVALID_STATE;
    }

    struct dtv_frontend_properties *c = &g_linux_dvb_frontend->dtv_property_cache;

    /* Convert kHz to Hz and kSps to Sps for the dtv_property_cache */
    c->frequency = params->freq_khz * 1000;
    c->symbol_rate = params->sr_ksps * 1000;

    /* Directly use integer values from params for enums, assuming they match DVB API definitions */
    /* It's crucial that the values sent from the web UI correspond to valid enum values */
    c->fec_inner = (enum fe_code_rate)params->fec;
    c->inversion = (enum fe_spectral_inversion)params->inv;
    c->delivery_system = (enum fe_delivery_system)params->delsys;
    c->modulation = (enum fe_modulation)params->mod;
    c->pilot = (enum fe_pilot)params->pilot;
    c->rolloff = (enum fe_rolloff)params->rolloff;

    ESP_LOGI(TAG_CX, "Web Tune: Freq=%lu Hz, SR=%lu Sps, DelSys=%d, Mod=%d, FEC=%d, Inv=%d, Pilot=%d, Rolloff=%d",
             c->frequency, c->symbol_rate, c->delivery_system, c->modulation, c->fec_inner, c->inversion, c->pilot, c->rolloff);

    if (g_linux_dvb_frontend->ops.tune)
    {
        unsigned int delay_ticks = 0;
        enum fe_status status_val = 0;

        ESP_LOGI(TAG_CX, "Calling frontend ops.tune via web request...");
        int ret = g_linux_dvb_frontend->ops.tune(g_linux_dvb_frontend, true, 0, &delay_ticks, &status_val);

        if (ret == 0)
        {
            ESP_LOGI(TAG_CX, "Web tuner_tune command successful. FE Status: 0x%02x. Delay: %u ticks", status_val, delay_ticks);
            return ESP_OK;
        }
        else
        {
            ESP_LOGE(TAG_CX, "Web tuner_tune command failed (ret: %d)", ret);
            return ESP_FAIL; /* Or a more specific error code if available */
        }
    }
    else
    {
        ESP_LOGE(TAG_CX, "Frontend tune function not available in ops table.");
        return ESP_ERR_NOT_SUPPORTED;
    }
    return ESP_FAIL; /* Should not be reached if ops.tune exists */
}

void tuner_init_base(void)
{
    tuner_reset_set(true);

    /* Register tuner_init command */
    tuner_init_args.mpeg_clk_pol = arg_int0("p", "clkpol", "<pol>", "MPEG Clock Polarity (default: 0x02)");
    tuner_init_args.end = arg_end(1); /* Corrected from 2 to 1 as only mpeg_clk_pol is an arg */
    const esp_console_cmd_t tuner_init_cmd = {
        .command = "tuner_init",
        .help = "Initialize CX24116 tuner using Linux driver logic and load firmware.",
        .hint = NULL,
        .func = &do_tuner_init_cmd,
        .argtable = &tuner_init_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&tuner_init_cmd));

    /* Register tuner_tune command */
    tuner_tune_cmd_args.freq_khz = arg_int1("f", "freq", "<kHz>", "Frequency in kHz");
    tuner_tune_cmd_args.sr_sps = arg_int1("s", "sr", "<Sps>", "Symbol Rate in Sps");
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

    tuner_initialized = true;
}