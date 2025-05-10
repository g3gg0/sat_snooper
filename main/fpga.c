#include <string.h>
#include <stdlib.h> /* For atoi */

#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_console.h"

#include "fpga.h"
#include "i2c.h"
#include "gpio_config.h"

/* Define the expected FPGA ID as a uint32_t */
#define LCMXO2_256HC_QFN32_ID 0x012B8043
/* https://bsdl.info/details.htm?sid=1667b5d760b4fa6288ca59b002289d77 */

#define BITSTREAM_BLOCK_SIZE 128

static const char *TAG = "FPGA";

int fpga_bitstream_dl(const char *file)
{
    int ret = 0;

    FILE *fp = fopen(file, "rb");
    if (!fp)
    {
        ESP_LOGE(TAG, "Failed to open bitstream file: %s", file);
        return 1;
    }

    /* Enter programming mode */
    uint8_t cmd_refresh[] = {FPGA_CMD_REFRESH, 0x00, 0x00};
    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd_refresh, sizeof(cmd_refresh)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_REFRESH failed");
        fclose(fp);
        return 1;
    }

    uint8_t cmd_enable_offline[] = {FPGA_CMD_ENABLE_CFG_IF_OFFLINE, 0x00, 0x00};
    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd_enable_offline, sizeof(cmd_enable_offline)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_ENABLE_CFG_IF_OFFLINE failed");
        fclose(fp);
        return 1;
    }

    uint8_t cmd_erase[] = {FPGA_CMD_ERASE, 0x07, 0x00}; /* 0x0100: SRAM */
    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd_erase, sizeof(cmd_erase)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_ERASE failed");
        fclose(fp);
        return 1;
    }

    /* Open bitstream file */

    uint8_t burst_command[] = {FPGA_CMD_LSC_BITSTREAM_BURST, 0x00, 0x00, 0x00, 0x00};
    if (i2c_write_partial(FPGA_ADDR_CONF_PRIMARY, burst_command, sizeof(burst_command), false) != 0)
    {
        ESP_LOGE(TAG, "Failed to write bitstream burst command");
        fclose(fp);
        return 1;
    }

    /* Write bitstream in blocks */
    uint8_t buf[BITSTREAM_BLOCK_SIZE];
    size_t read_bytes;
    while ((read_bytes = fread(buf, 1, BITSTREAM_BLOCK_SIZE, fp)) > 0)
    {
        bool end = feof(fp);

        if (i2c_write_partial(FPGA_ADDR_CONF_PRIMARY, buf, read_bytes, end) != 0)
        {
            ESP_LOGE(TAG, "Failed to write bitstream block");
            ret = 1;
            break;
        }
    }

    fclose(fp);

    if(ret)
    {
        return ret;
    }

    /* Exit programming mode */
    uint8_t cmd_disable[] = {FPGA_CMD_DISABLE_CFG_IF, 0x00, 0x00};
    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd_disable, sizeof(cmd_disable)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_DISABLE_CFG_IF failed");
        ret = 1;
    }

    uint8_t cmd_noop[] = {FPGA_CMD_BYPASS, 0xFF, 0xFF, 0xFF};
    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd_noop, sizeof(cmd_noop)) != 0)
    {
        ESP_LOGE(TAG, "NO_OP failed");
        ret = 1;
    }

    return ret;
}

static int do_fpga_erase(int argc, char **argv)
{
    uint8_t erase_target = 0;

    if (argc < 2)
    {
        ESP_LOGE(TAG, "Usage: fpga_erase <sram|flash|feature|ufm|all>");
        return 1;
    }

    const char *target_str = argv[1];

    if (strcmp(target_str, "sram") == 0) {
        erase_target = ERASE_SRAM;
    } else if (strcmp(target_str, "flash") == 0) {
        erase_target = ERASE_FLASH;
    } else if (strcmp(target_str, "feature") == 0) {
        erase_target = ERASE_FEATURE;
    } else if (strcmp(target_str, "ufm") == 0) {
        erase_target = ERASE_UFM;
    } else if (strcmp(target_str, "all") == 0) {
        erase_target = ERASE_ALL;
    } else {
        ESP_LOGE(TAG, "Invalid erase target: %s. Use <sram|flash|feature|ufm|all>", target_str);
        return 1;
    }

    ESP_LOGI(TAG, "Erasing target: 0x%02X", erase_target);

    /* Enter programming mode (offline) */
    uint8_t cmd_enable_offline[] = {FPGA_CMD_ENABLE_CFG_IF_OFFLINE, 0x00, 0x00};
    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd_enable_offline, sizeof(cmd_enable_offline)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_ENABLE_CFG_IF_OFFLINE failed");
        return 1;
    }

    /* Send erase command */
    uint8_t cmd_erase[] = {FPGA_CMD_ERASE, erase_target, 0x00};
    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd_erase, sizeof(cmd_erase)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_ERASE failed for target 0x%02X", erase_target);
        /* Attempt to disable config interface even on failure */
        uint8_t cmd_disable[] = {FPGA_CMD_DISABLE_CFG_IF, 0x00, 0x00};
        i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd_disable, sizeof(cmd_disable));
        return 1;
    }

    ESP_LOGI(TAG, "Erase command sent successfully for target 0x%02X. Check status register for completion.", erase_target);

    /* Exit programming mode */
    uint8_t cmd_disable[] = {FPGA_CMD_DISABLE_CFG_IF, 0x00, 0x00};
    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd_disable, sizeof(cmd_disable)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_DISABLE_CFG_IF failed after erase");
        return 1; /* Return error even if erase might have worked */
    }

    return 0;
}

static int do_fpga_prog(int argc, char **argv)
{
    if (argc < 2)
    {
        ESP_LOGE(TAG, "Usage: fpga_prog <0|1>");
        return 1;
    }

    int level = atoi(argv[1]);

    if (level != 0 && level != 1)
    {
        ESP_LOGE(TAG, "Invalid level: %s. Use 0 or 1.", argv[1]);
        return 1;
    }

    ESP_LOGI(TAG, "Setting GPIO_FPGA_PROGRAMN to %d", level);
    gpio_set_level(GPIO_FPGA_PROGRAMN, level);

    return 0;
}

static int do_fpga_id(int argc, char **argv)
{
    uint8_t id[4];

    uint8_t dummy[] = {0x00};
    i2c_write(FPGA_ADDR_CONF_RESET, dummy, sizeof(dummy));

    /* Write first command */
    uint8_t cmd1[] = {FPGA_CMD_ENABLE_CFG_IF_TRANSPARENT, 0x00, 0x00};
    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd1, sizeof(cmd1)) != 0)
    {
        ESP_LOGE(TAG, "Failed to write first command to FPGA");
        return 1;
    }

    /* Write second command */
    uint8_t cmd2[] = {FPGA_CMD_READ_DEVICE_ID, 0x00, 0x00, 0x00};
    if (i2c_write_read(FPGA_ADDR_CONF_PRIMARY, cmd2, sizeof(cmd2), id, sizeof(id)) != 0)
    {
        ESP_LOGE(TAG, "Failed to write second command to FPGA");
        return 1;
    }

    ESP_LOGI(TAG, "FPGA ID: %02X %02X %02X %02X", id[0], id[1], id[2], id[3]);

    /* Combine the ID bytes into a uint32_t (big-endian) */
    uint32_t fpga_id = (id[0] << 24) | (id[1] << 16) | (id[2] << 8) | id[3];

    /* Check for specific FPGA ID and print chip type if matched */
    if (fpga_id == LCMXO2_256HC_QFN32_ID)
    {
        ESP_LOGI(TAG, "Detected chip type: LCMXO2 256HC QFN32");
    }
    else
    {
        ESP_LOGW(TAG, "Detected unknown chip type: 0x%08lX", fpga_id);
    }
    return 0;
}

static int do_fpga_status(int argc, char **argv)
{
    uint8_t status_bytes[4];
    uint32_t status_raw;
    fpga_statusreg_t *status_reg;

    /* Write read status command */
    uint8_t cmd_read_status[] = {FPGA_CMD_READ_STATUS_REGISTER, 0x00, 0x00, 0x00};
    if (i2c_write_read(FPGA_ADDR_CONF_PRIMARY, cmd_read_status, sizeof(cmd_read_status), status_bytes, sizeof(status_bytes)) != 0)
    {
        ESP_LOGE(TAG, "Failed to write second command to FPGA");
        return 1;
    }

    /* Combine bytes into uint32_t (assuming little-endian) */
    status_raw = status_bytes[0] | (status_bytes[1] << 8) | (status_bytes[2] << 16) | (status_bytes[3] << 24);
    status_reg = (fpga_statusreg_t *)&status_raw;

    ESP_LOGI(TAG, "FPGA Status Register (0x%08lX):", status_raw);
    ESP_LOGI(TAG, "  JTAG Transparent Mode  : %u", status_reg->jtag_transparent_mode);
    ESP_LOGI(TAG, "  Config Target Selection: %u", status_reg->config_target_selection);
    ESP_LOGI(TAG, "  JTAG Active            : %u", status_reg->jtag_active);
    ESP_LOGI(TAG, "  PWD Protect            : %u", status_reg->pwd_protect);
    ESP_LOGI(TAG, "  OTP                    : %u", status_reg->otp);
    ESP_LOGI(TAG, "  Decrypt Enable         : %u", status_reg->decrypt_enable);
    ESP_LOGI(TAG, "  DONE                   : %u", status_reg->done);
    ESP_LOGI(TAG, "  ISC Enable             : %u", status_reg->isc_enable);
    ESP_LOGI(TAG, "  Write Enable           : %u", status_reg->write_enable);
    ESP_LOGI(TAG, "  Read Enable            : %u", status_reg->read_enable);
    ESP_LOGI(TAG, "  Busy Flag              : %u", status_reg->busy_flag);
    ESP_LOGI(TAG, "  Fail Flag              : %u", status_reg->fail_flag);
    ESP_LOGI(TAG, "  FEA OTP                : %u", status_reg->fea_otp);
    ESP_LOGI(TAG, "  Decrypt Only           : %u", status_reg->decrypt_only);
    ESP_LOGI(TAG, "  PWD Enable             : %u", status_reg->pwd_enable);
    ESP_LOGI(TAG, "  UFM OTP                : %u", status_reg->ufm_otp);
    ESP_LOGI(TAG, "  ASSP                   : %u", status_reg->assp);
    ESP_LOGI(TAG, "  SDM En                 : %u", status_reg->sdm_en);
    ESP_LOGI(TAG, "  Encrypt Preamble       : %u", status_reg->encrypt_preamble);
    ESP_LOGI(TAG, "  Std Preamble           : %u", status_reg->std_preamble);
    ESP_LOGI(TAG, "  SPIm Fail 1            : %u", status_reg->spim_fail_1);
    ESP_LOGI(TAG, "  BSE Error Code         : %u", status_reg->bse_error_code);
    ESP_LOGI(TAG, "  Execution Error        : %u", status_reg->execution_error);
    ESP_LOGI(TAG, "  ID Error               : %u", status_reg->id_error);
    ESP_LOGI(TAG, "  Invalid Command        : %u", status_reg->invalid_command);
    ESP_LOGI(TAG, "  SED Error              : %u", status_reg->sed_error);
    ESP_LOGI(TAG, "  Bypass Mode            : %u", status_reg->bypass_mode);
    ESP_LOGI(TAG, "  FlowThrough Mode       : %u", status_reg->flowthrough_mode);

    return 0;
}

static int do_fpga_dl(int argc, char **argv)
{
    if (argc < 2)
    {
        return fpga_bitstream_dl("/storage/fpga.bit");
    }
    return fpga_bitstream_dl(argv[1]);
}

void register_fpga_id_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "fpga_id",
        .help = "Read FPGA ID via I2C",
        .hint = NULL,
        .func = &do_fpga_id,
        .argtable = NULL};
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void register_fpga_status_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "fpga_status",
        .help = "Read FPGA Status Register via I2C",
        .hint = NULL,
        .func = &do_fpga_status,
        .argtable = NULL};
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void register_fpga_prog_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "fpga_prog",
        .help = "Set FPGA PROGRAMN pin level (0 or 1)",
        .hint = "<0|1>",
        .func = &do_fpga_prog,
        .argtable = NULL};
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void register_fpga_erase_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "fpga_erase",
        .help = "Erase FPGA memory sections (sram, flash, feature, ufm, all)",
        .hint = "<sram|flash|feature|ufm|all>",
        .func = &do_fpga_erase,
        .argtable = NULL};
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void register_fpga_dl_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "fpga_dl",
        .help = "Download bitstream file into FPGA via I2C",
        .hint = NULL,
        .func = &do_fpga_dl,
        .argtable = NULL};
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void fpga_init()
{
    gpio_set_direction(GPIO_FPGA_PROGRAMN, GPIO_MODE_OUTPUT);

    register_fpga_id_command();
    register_fpga_status_command();
    register_fpga_dl_command();
    register_fpga_erase_command();
    register_fpga_prog_command(); /* Register the new prog command */

    gpio_set_level(GPIO_FPGA_PROGRAMN, 0);
    //fpga_bitstream_dl("/storage/fpga.bit");
    gpio_set_level(GPIO_FPGA_PROGRAMN, 0);
}
