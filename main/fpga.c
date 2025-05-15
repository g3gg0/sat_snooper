#include <string.h>
#include <stdlib.h> /* For atoi */

#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_console.h"

#include "fpga.h"
#include "i2c.h"
#include "gpio_config.h"

/* Includes for socket listener */
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Define the expected FPGA ID as a uint32_t */
#define LCMXO2_256HC_QFN32_ID 0x012B8043
/* https://bsdl.info/details.htm?sid=1667b5d760b4fa6288ca59b002289d77 */

#define BITSTREAM_BLOCK_SIZE 128
#define SOCKET_LISTENER_PORT 255
#define SOCKET_RX_BUFFER_INITIAL_SIZE 1024
#define SOCKET_LISTENER_TASK_STACK_SIZE 4096
#define SOCKET_LISTENER_TASK_PRIORITY 5

static const char *TAG = "FPGA";

int fpga_bitstream_dl_data(const uint8_t *data, size_t length)
{
    ESP_LOGI(TAG, "Starting bitstream download from data buffer (%d bytes)...", length);
    int ret = 0;

    gpio_set_level(GPIO_FPGA_PROGRAMN, 0);

    /* Enter programming mode */
    uint8_t cmd_refresh[] = {FPGA_CMD_REFRESH, 0x00, 0x00};
    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd_refresh, sizeof(cmd_refresh)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_REFRESH failed");
        gpio_set_level(GPIO_FPGA_PROGRAMN, 1);
        return 1;
    }

    uint8_t cmd_enable_offline[] = {FPGA_CMD_ENABLE_CFG_IF_OFFLINE, 0x00, 0x00};
    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd_enable_offline, sizeof(cmd_enable_offline)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_ENABLE_CFG_IF_OFFLINE failed");
        gpio_set_level(GPIO_FPGA_PROGRAMN, 1);
        return 1;
    }

    uint8_t cmd_erase[] = {FPGA_CMD_ERASE, 0x07, 0x00}; /* 0x0100: SRAM, 0x0200: UFM, 0x0400: Feature Row, 0x0000: CFG Flash */
                                                        /* Using 0x07 to match previous erase logic if it implied SRAM + UFM + Feature */
                                                        /* For SRAM only programming, typically 0x01 is used for ERASE_SRAM. */
                                                        /* The original fpga_bitstream_dl used 0x07. Let's assume it means erase relevant config flash sectors. */
                                                        /* For programming SRAM directly, ERASE_SRAM (0x01) is more typical. */
    cmd_erase[1] = ERASE_SRAM;                          /* Corrected to ERASE_SRAM for typical bitstream loading to SRAM */

    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd_erase, sizeof(cmd_erase)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_ERASE (SRAM) failed");
        gpio_set_level(GPIO_FPGA_PROGRAMN, 1);
        return 1;
    }
    ESP_LOGI(TAG, "FPGA SRAM erase command sent.");
    // Add delay or status check for erase completion if necessary, though often not for SRAM.
    vTaskDelay(pdMS_TO_TICKS(100)); // Small delay after erase

    uint8_t burst_command[] = {FPGA_CMD_LSC_BITSTREAM_BURST, 0x00, 0x00, 0x00, 0x00};
    if (i2c_write_partial(FPGA_ADDR_CONF_PRIMARY, burst_command, sizeof(burst_command), false) != 0)
    {
        ESP_LOGE(TAG, "Failed to write bitstream burst command");
        gpio_set_level(GPIO_FPGA_PROGRAMN, 1);
        return 1;
    }
    ESP_LOGI(TAG, "FPGA bitstream burst command sent.");

    /* Write bitstream in blocks */
    size_t bytes_written = 0;
    while (bytes_written < length)
    {
        size_t current_chunk_size = (length - bytes_written > BITSTREAM_BLOCK_SIZE) ? BITSTREAM_BLOCK_SIZE : (length - bytes_written);
        bool end_of_stream = (bytes_written + current_chunk_size >= length);

        if (i2c_write_partial(FPGA_ADDR_CONF_PRIMARY, data + bytes_written, current_chunk_size, end_of_stream) != 0)
        {
            ESP_LOGE(TAG, "Failed to write bitstream block (offset %d, size %d)", bytes_written, current_chunk_size);
            ret = 1;
            break;
        }
        bytes_written += current_chunk_size;
        // ESP_LOGD(TAG, "Wrote %d bytes, total %d/%d", current_chunk_size, bytes_written, length);
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay to allow I2C bus to settle if needed
    }

    if (ret)
    {
        ESP_LOGE(TAG, "Bitstream data download failed at %d bytes.", bytes_written);
        // Attempt to disable config interface even on failure
        uint8_t cmd_disable_fail[] = {FPGA_CMD_DISABLE_CFG_IF, 0x00, 0x00};
        i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd_disable_fail, sizeof(cmd_disable_fail));
        return ret;
    }
    ESP_LOGI(TAG, "Bitstream data download completed (%d bytes).", length);

    /* Exit programming mode */
    uint8_t cmd_disable[] = {FPGA_CMD_DISABLE_CFG_IF, 0x00, 0x00};
    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd_disable, sizeof(cmd_disable)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_DISABLE_CFG_IF failed");
        ret = 1;
    }

    /* According to Lattice documentation, after LSC_BITSTREAM_BURST and DISABLE_CFG_IF,
     * a LSC_REFRESH command might be needed to make the FPGA load the configuration from SRAM.
     * Or, PROGRAMDONE followed by REFRESH. The original code had a NOOP.
     * Let's try with a REFRESH command as it's often part of reconfiguring from external source.
     * However, if programming SRAM directly, the device usually configures itself upon PROGRAMDONE being asserted by the master.
     * The DISABLE_CFG_IF might be enough. Let's stick to the original NOOP for now unless issues arise.
     */
    uint8_t cmd_noop[] = {FPGA_CMD_BYPASS, 0xFF, 0xFF, 0xFF}; // Or FPGA_CMD_PROGRAMDONE followed by FPGA_CMD_REFRESH
    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd_noop, sizeof(cmd_noop)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_BYPASS (NO_OP) failed after bitstream load");
        // ret = 1; // This might be non-critical if programming already succeeded.
    }

    gpio_set_level(GPIO_FPGA_PROGRAMN, 1);
    if (ret == 0)
    {
        ESP_LOGI(TAG, "FPGA programming from data buffer successful.");
    }
    else
    {
        ESP_LOGE(TAG, "FPGA programming from data buffer failed.");
    }
    return ret;
}

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

    if (ret)
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

static void fpga_socket_listener_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = 0;
    int keepAlive = 1;
    int listen_sock = -1;

    struct sockaddr_storage dest_addr;

    if (addr_family == AF_INET)
    {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(SOCKET_LISTENER_PORT);
        ip_protocol = IPPROTO_IP;
    }

    listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket created");

    // Set SO_REUSEADDR to allow binding to the same address shortly after closing
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0)
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", SOCKET_LISTENER_PORT);

    err = listen(listen_sock, 1);
    if (err != 0)
    {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket listening");

    while (1)
    {
        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break; // Exit loop on accept error
        }

        // Set connection keep-alive
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));

        if (source_addr.ss_family == PF_INET)
        {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        uint8_t *rx_buffer = (uint8_t *)malloc(SOCKET_RX_BUFFER_INITIAL_SIZE);
        size_t buffer_capacity = SOCKET_RX_BUFFER_INITIAL_SIZE;
        size_t total_received_bytes = 0;
        int len;

        if (!rx_buffer)
        {
            ESP_LOGE(TAG, "Failed to allocate initial receive buffer");
            close(sock);
            continue;
        }

        do
        {
            len = recv(sock, rx_buffer + total_received_bytes, buffer_capacity - total_received_bytes, 0);
            if (len < 0)
            {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                // Connection error
                break;
            }
            else if (len == 0)
            {
                ESP_LOGI(TAG, "Connection closed by client");
                // Connection closed
            }
            else
            {
                total_received_bytes += len;
                ESP_LOGD(TAG, "Received %d bytes, total %d", len, total_received_bytes);
                if (total_received_bytes == buffer_capacity)
                {
                    // Buffer is full, try to resize
                    size_t new_capacity = buffer_capacity * 2;
                    uint8_t *new_buffer = (uint8_t *)realloc(rx_buffer, new_capacity);
                    if (!new_buffer)
                    {
                        ESP_LOGE(TAG, "Failed to reallocate receive buffer to %d bytes", new_capacity);
                        // Keep existing data, but can't receive more
                        len = -1; // Simulate error to break loop
                        break;
                    }
                    rx_buffer = new_buffer;
                    buffer_capacity = new_capacity;
                    ESP_LOGI(TAG, "Resized receive buffer to %d bytes", buffer_capacity);
                }
            }
        } while (len > 0);

        if (total_received_bytes > 0)
        {
            ESP_LOGI(TAG, "Total bytes received: %d. Programming FPGA...", total_received_bytes);
            fpga_bitstream_dl_data(rx_buffer, total_received_bytes);
        }
        else if (len < 0)
        {
            ESP_LOGE(TAG, "Socket error, no data to program.");
        }

        free(rx_buffer);
        shutdown(sock, 0);
        close(sock);
        ESP_LOGI(TAG, "Socket closed, waiting for new connection...");
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

void fpga_start_socket_listener(void)
{
    xTaskCreate(fpga_socket_listener_task, "fpga_sock_listen", SOCKET_LISTENER_TASK_STACK_SIZE, NULL, SOCKET_LISTENER_TASK_PRIORITY, NULL);
    ESP_LOGI(TAG, "FPGA bitstream socket listener task started on port %d", SOCKET_LISTENER_PORT);
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

    if (strcmp(target_str, "sram") == 0)
    {
        erase_target = ERASE_SRAM;
    }
    else if (strcmp(target_str, "flash") == 0)
    {
        erase_target = ERASE_FLASH;
    }
    else if (strcmp(target_str, "feature") == 0)
    {
        erase_target = ERASE_FEATURE;
    }
    else if (strcmp(target_str, "ufm") == 0)
    {
        erase_target = ERASE_UFM;
    }
    else if (strcmp(target_str, "all") == 0)
    {
        erase_target = ERASE_ALL;
    }
    else
    {
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
    fpga_bitstream_dl("/storage/fpga.bit");
    gpio_set_level(GPIO_FPGA_PROGRAMN, 1);

    fpga_start_socket_listener(); // Start the socket listener
}
