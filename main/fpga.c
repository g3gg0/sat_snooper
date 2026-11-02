#include <string.h>
#include <stdlib.h> /* For atoi */

#include "driver/gpio.h"
#include "driver/ledc.h" /* Added for LEDC clock output functionality */

#include "esp_clock_output.h"
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

#define BITSTREAM_BLOCK_SIZE 128
#define SOCKET_LISTENER_PORT 255
#define SOCKET_RX_BUFFER_INITIAL_SIZE 1024
#define SOCKET_LISTENER_TASK_STACK_SIZE 4096
#define SOCKET_LISTENER_TASK_PRIORITY 5
#define FPGA_PID_FILTER_COMMAND 0xf0

static const char *TAG = "FPGA";

static bool fpga_prog_active = false;

esp_err_t fpga_set_pid_filters(const fpga_pid_filter_t filters[FPGA_PID_FILTER_COUNT])
{
    if (!filters)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t command[2 + FPGA_PID_FILTER_COUNT * 4] = {FPGA_PID_FILTER_COMMAND, 0};
    for (size_t index = 0; index < FPGA_PID_FILTER_COUNT; index++)
    {
        if (filters[index].pid > 0x1fff || filters[index].mask > 0x1fff)
        {
            return ESP_ERR_INVALID_ARG;
        }
        if (filters[index].enabled)
        {
            command[1] |= 1U << index;
        }
        size_t offset = 2 + index * 4;
        command[offset] = filters[index].pid >> 8;
        command[offset + 1] = filters[index].pid;
        command[offset + 2] = filters[index].mask >> 8;
        command[offset + 3] = filters[index].mask;
    }

    if (i2c_write(FPGA_ADDR_USER_PRIMARY, command, sizeof(command)) != 0)
    {
        ESP_LOGE(TAG, "Failed to write PID filters to FPGA");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void fpga_prog(bool active)
{
    gpio_set_level(GPIO_FPGA_PROGRAMN, active ? 0 : 1);
    fpga_prog_active = active;
}

bool fpga_prog_get()
{
    return fpga_prog_active;
}

int fpga_dl_data(const uint8_t *data, size_t length)
{
    ESP_LOGI(TAG, "Starting bitstream download from data buffer (%d bytes)...", length);
    int ret = 0;

    fpga_prog(true);

    /* Enter programming mode */
    uint8_t cmd_refresh[] = {FPGA_CMD_REFRESH, 0x00, 0x00};
    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd_refresh, sizeof(cmd_refresh)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_REFRESH failed");
        fpga_prog(false);
        return 1;
    }

    uint8_t cmd_enable_offline[] = {FPGA_CMD_ENABLE_CFG_IF_OFFLINE, 0x00, 0x00};
    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd_enable_offline, sizeof(cmd_enable_offline)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_ENABLE_CFG_IF_OFFLINE failed");
        fpga_prog(false);
        return 1;
    }

    uint8_t cmd_erase[] = {FPGA_CMD_ERASE, 0x00, 0x00, 0x00}; /* 0x0100: SRAM, 0x0200: UFM, 0x0400: Feature Row, 0x0000: CFG Flash */
                                                              /* Using 0x07 to match previous erase logic if it implied SRAM + UFM + Feature */
                                                              /* For SRAM only programming, typically 0x01 is used for ERASE_SRAM. */
                                                              /* The original fpga_dl_file used 0x07. Let's assume it means erase relevant config flash sectors. */
                                                              /* For programming SRAM directly, ERASE_SRAM (0x01) is more typical. */
    cmd_erase[1] = ERASE_SRAM;                                /* Corrected to ERASE_SRAM for typical bitstream loading to SRAM */

    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd_erase, sizeof(cmd_erase)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_ERASE (SRAM) failed");
        fpga_prog(false);
        return 1;
    }
    ESP_LOGI(TAG, "FPGA SRAM erase command sent.");
    // Add delay or status check for erase completion if necessary, though often not for SRAM.
    vTaskDelay(pdMS_TO_TICKS(100)); // Small delay after erase

    uint8_t burst_command[] = {FPGA_CMD_LSC_BITSTREAM_BURST, 0x00, 0x00, 0x00};
    if (i2c_write_partial(FPGA_ADDR_CONF_PRIMARY, burst_command, sizeof(burst_command), false) != 0)
    {
        ESP_LOGE(TAG, "Failed to write bitstream burst command");
        fpga_prog(false);
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

    fpga_prog(false);
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

int fpga_dl_file(const char *file)
{
    ESP_LOGI(TAG, "Attempting to load bitstream file: %s into RAM", file);
    FILE *fp = fopen(file, "rb");
    if (!fp)
    {
        ESP_LOGE(TAG, "Failed to open bitstream file: %s", file);
        return 1;
    }

    fseek(fp, 0, SEEK_END);
    long length = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    if (length <= 0)
    {
        ESP_LOGE(TAG, "Bitstream file is empty or invalid size: %s", file);
        fclose(fp);
        return 1;
    }

    uint8_t *bitstream_data = (uint8_t *)malloc(length);
    if (!bitstream_data)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for bitstream data (%ld bytes)", length);
        fclose(fp);
        return 1;
    }

    size_t read_bytes = fread(bitstream_data, 1, length, fp);
    fclose(fp);

    if (read_bytes != length)
    {
        ESP_LOGE(TAG, "Failed to read full bitstream file into memory (read %d, expected %ld)", read_bytes, length);
        free(bitstream_data);
        return 1;
    }

    ESP_LOGI(TAG, "Successfully loaded bitstream file into RAM (%ld bytes). Calling fpga_dl_data.", length);
    int ret = fpga_dl_data(bitstream_data, length);

    free(bitstream_data);
    return ret;
}

void fpga_clk_init(void)
{
    ESP_LOGI(TAG, "Initializing FPGA clock output on GPIO %d for 40MHz", GPIO_FPGA_CLOCK);
    esp_clock_output_mapping_handle_t clkout_handle = NULL;
    esp_err_t ret;

    /* Attempt 1: Use CLKOUT_SIG_XTAL (Crystal oscillator) */
    /* Assuming XTAL is 40MHz. If it is not, this will output the XTAL's actual frequency. */
    ESP_LOGI(TAG, "Attempting to use CLKOUT_SIG_XTAL as clock source.");
    ret = esp_clock_output_start(CLKOUT_SIG_XTAL, GPIO_FPGA_CLOCK, &clkout_handle);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "FPGA clock output (from XTAL) started successfully on GPIO %d.", GPIO_FPGA_CLOCK);
        /* If CLKOUT_SIG_XTAL is precisely 40MHz, no divider is needed. */
        /* The handle clkout_handle could be stored if we need to stop/modify the clock later. */
        /* If XTAL is not 40MHz, and divider support is available and needed, one might call esp_clock_output_set_divider here. */
        /* However, esp_clock_output_set_divider might not be applicable to all clock sources or SoCs. */
        /* For simplicity, if XTAL is the target frequency, we assume it matches. */
        return;
    }
    ESP_LOGW(TAG, "Failed to start FPGA clock output with CLKOUT_SIG_XTAL (%s).", esp_err_to_name(ret));
    if (clkout_handle)
    { /* If a previous attempt partially initialized, try to clean up */
        esp_clock_output_stop(clkout_handle);
        clkout_handle = NULL;
    }

    /* Attempt 2: Use CLKOUT_SIG_PLL_F80M (80MHz PLL clock) and set a divider */
    ESP_LOGI(TAG, "Attempting to use CLKOUT_SIG_PLL_F80M as source with a divider for 40MHz.");
    ret = esp_clock_output_start(CLKOUT_SIG_PLL_F80M, GPIO_FPGA_CLOCK, &clkout_handle);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Clock output (from PLL_F80M at 80MHz) started on GPIO %d.", GPIO_FPGA_CLOCK);
#if SOC_CLOCKOUT_SUPPORT_CHANNEL_DIVIDER
        ESP_LOGI(TAG, "Setting divider to 2 to achieve 40MHz output.");
        ret = esp_clock_output_set_divider(clkout_handle, 2); /* 80MHz / 2 = 40MHz */
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "FPGA clock output (40MHz from PLL_F80M) configured successfully on GPIO %d.", GPIO_FPGA_CLOCK);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to set clock divider for PLL_F80M: %s. Stopping clock output.", esp_err_to_name(ret));
            esp_clock_output_stop(clkout_handle);
            clkout_handle = NULL; /* Mark handle as invalid */
        }
#else
        ESP_LOGW(TAG, "Clock divider setting is not supported for this configuration. Output will be 80MHz from PLL_F80M.");
        /* If 80MHz is not acceptable, this path is problematic. Consider stopping: esp_clock_output_stop(clkout_handle); clkout_handle = NULL; */
#endif
        if (clkout_handle)
            return; /* Return if successfully configured or if divider not supported but output started */
    }
    ESP_LOGE(TAG, "Failed to start FPGA clock output with SOC_CLOCK_PLL_F80M: %s.", esp_err_to_name(ret));
    if (clkout_handle)
    { /* Clean up if needed from a failed start */
        esp_clock_output_stop(clkout_handle);
        clkout_handle = NULL;
    }

    /* Attempt 3: Use CLKOUT_SIG_PLL_F80M (160MHz PLL clock) and set a divider */
    ESP_LOGI(TAG, "Attempting to use CLKOUT_SIG_PLL_F80M as source with a divider for 40MHz.");
    ret = esp_clock_output_start(CLKOUT_SIG_PLL_F80M, GPIO_FPGA_CLOCK, &clkout_handle);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Clock output (from PLL_F160M at 160MHz) started on GPIO %d.", GPIO_FPGA_CLOCK);
#if SOC_CLOCKOUT_SUPPORT_CHANNEL_DIVIDER
        ESP_LOGI(TAG, "Setting divider to 4 to achieve 40MHz output.");
        ret = esp_clock_output_set_divider(clkout_handle, 4); /* 160MHz / 4 = 40MHz */
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "FPGA clock output (40MHz from PLL_F160M) configured successfully on GPIO %d.", GPIO_FPGA_CLOCK);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to set clock divider for PLL_F160M: %s. Stopping clock output.", esp_err_to_name(ret));
            esp_clock_output_stop(clkout_handle);
            clkout_handle = NULL;
        }
#else
        ESP_LOGW(TAG, "Clock divider setting is not supported. Output will be 160MHz from PLL_F160M.");
#endif
        if (clkout_handle)
            return;
    }
    ESP_LOGE(TAG, "Failed to start FPGA clock output with SOC_CLOCK_PLL_F160M: %s.", esp_err_to_name(ret));
    if (clkout_handle)
    {
        esp_clock_output_stop(clkout_handle);
        clkout_handle = NULL;
    }

    ESP_LOGE(TAG, "All attempts to configure FPGA clock output failed.");
}

#ifdef GPIO_FPGA_CLOCK_OLD
void fpga_clk_old_init(uint32_t freq)
{
    ESP_LOGI(TAG, "Initializing OLD FPGA clock output (10 MHz) on GPIO %d using LEDC", GPIO_FPGA_CLOCK_OLD);

    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     * For a 10MHz output with 50% duty, we can use 1-bit resolution.
     * Max duty for 1-bit resolution is (2^1 - 1) = 1.
     * Duty = 1 will give 50% duty cycle.
     */
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,   /* Timer mode */
        .duty_resolution = LEDC_TIMER_1_BIT, /* Duty resolution (1 bit for 50% duty) */
        .timer_num = LEDC_TIMER_1,           /* Timer index (using TIMER_1 to avoid conflict if TIMER_0 is used elsewhere) */
        .freq_hz = freq,                     /* Frequency in Hertz */
        .clk_cfg = LEDC_AUTO_CLK             /* Auto select the source clock */
    };
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "LEDC timer configuration failed: %s", esp_err_to_name(ret));
        return;
    }

    /*
     * Prepare and set configuration of LEDC channels
     * channels will be assigned timers.
     */
    ledc_channel_config_t ledc_channel = {
        .gpio_num = GPIO_FPGA_CLOCK_OLD,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1, /* LEDC channel (using CHANNEL_1) */
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1, /* Timer index to use */
        .duty = 1,                 /* Set duty to 1 for 50% with 1-bit resolution */
        .hpoint = 0};
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "LEDC channel configuration failed: %s", esp_err_to_name(ret));
        /* Optionally, try to deconfigure timer if channel config fails */
        /* ledc_timer_rst(ledc_timer.speed_mode, ledc_timer.timer_num); */
        return;
    }

    ESP_LOGI(TAG, "LEDC configured to output 10 MHz on GPIO %d", GPIO_FPGA_CLOCK_OLD);
}
#endif

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
            fpga_dl_data(rx_buffer, total_received_bytes);
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
        ESP_LOGE(TAG, "Usage: fpga_prog <0|1> (0=inactive, 1=active)");
        return 1;
    }

    int level = atoi(argv[1]);

    if (level != 0 && level != 1)
    {
        ESP_LOGE(TAG, "Invalid level: %s. Use 0 or 1.", argv[1]);
        return 1;
    }

    ESP_LOGI(TAG, "Setting GPIO_FPGA_PROGRAMN to %s", level ? "active" : "inactive");
    fpga_prog(level);

    return 0;
}

static const char *fpga_machxo2_device_name(uint32_t idcode)
{
    /* The upper nibble is the silicon revision, not the device type. */
    switch (idcode & UINT32_C(0x0fffffff))
    {
    case FPGA_MACHXO2_ID_256ZE:
        return "LCMXO2-256ZE";
    case FPGA_MACHXO2_ID_640ZE:
        return "LCMXO2-640ZE";
    case FPGA_MACHXO2_ID_1200ZE:
        return "LCMXO2-1200ZE";
    case FPGA_MACHXO2_ID_2000ZE:
        return "LCMXO2-2000ZE / LCMXO2-2000HE";
    case FPGA_MACHXO2_ID_4000ZE:
        return "LCMXO2-4000ZE / LCMXO2-4000HE / LCMXO2-2000UHE";
    case FPGA_MACHXO2_ID_7000ZE:
        return "LCMXO2-7000ZE / LCMXO2-7000HE";
    case FPGA_MACHXO2_ID_256HC:
        return "LCMXO2-256HC";
    case FPGA_MACHXO2_ID_640HC:
        return "LCMXO2-640HC";
    case FPGA_MACHXO2_ID_1200HC:
        return "LCMXO2-1200HC / LCMXO2-640UHC";
    case FPGA_MACHXO2_ID_2000HC:
        return "LCMXO2-2000HC / LCMXO2-1200UHC";
    case FPGA_MACHXO2_ID_4000HC:
        return "LCMXO2-4000HC / LCMXO2-2000UHC";
    case FPGA_MACHXO2_ID_7000HC:
        return "LCMXO2-7000HC";
    default:
        return NULL;
    }
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
    uint32_t fpga_id = ((uint32_t)id[0] << 24) | ((uint32_t)id[1] << 16) |
                       ((uint32_t)id[2] << 8) | (uint32_t)id[3];

    const char *device_name = fpga_machxo2_device_name(fpga_id);
    if (device_name)
    {
        ESP_LOGI(TAG, "Detected chip type: %s", device_name);
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

static uint8_t fpga_exec_read_busy()
{
    uint8_t cmd[] = {FPGA_CMD_READ_BUSY_FLAG, 0x00, 0x00, 0x00};
    uint8_t status_byte[1];

    if (i2c_write_read(FPGA_ADDR_CONF_PRIMARY, cmd, sizeof(cmd), status_byte, sizeof(status_byte)) != 0)
    {
        ESP_LOGE(TAG, "Failed to write second command to FPGA");
        return 0xFF;
    }

    return status_byte[0];
}

static uint8_t fpga_exec_wait_busy(uint32_t timeout_ms)
{
    TickType_t start_time = xTaskGetTickCount();

    while (fpga_exec_read_busy() & 0x80)
    {
        vTaskDelay(pdMS_TO_TICKS(10));

        if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(timeout_ms))
        {
            return 1; /* timeout */
        }
    }

    ESP_LOGI(TAG, "BUSY released after %" PRIu32 " ms", (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - start_time));

    return 0; /* success */
}

static int fpga_exec_enter_offline()
{
    uint8_t cmd[] = {FPGA_CMD_ENABLE_CFG_IF_OFFLINE, 0x08, 0x00};
    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd, sizeof(cmd)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_ENABLE_CFG_IF_OFFLINE failed");
        return 1;
    }

    return 0;
}

static int fpga_exec_exit_offline()
{
    uint8_t cmd[] = {FPGA_CMD_DISABLE_CFG_IF, 0x00, 0x00};
    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd, sizeof(cmd)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_DISABLE_CFG_IF failed");
        return 1;
    }

    return 0;
}

static int fpga_exec_reset_cfg_address()
{
    uint8_t cmd[] = {FPGA_CMD_RESET_CFG_FLASH_ADDRESS, 0x00, 0x00, 0x00};
    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd, sizeof(cmd)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_RESET_CFG_FLASH_ADDRESS failed");
        return 1;
    }

    return 0;
}

static int fpga_exec_erase(uint8_t type)
{
    uint8_t cmd[] = {FPGA_CMD_ERASE, type, 0x00, 0x00};

    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd, sizeof(cmd)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_ERASE failed");
        return 1;
    }
    fpga_exec_wait_busy(10000);

    return 0;
}

static int fpga_exec_program_feabits(uint16_t feabits_value)
{
    /* Prepare and send write FEABITS command */
    uint8_t cmd[] = {FPGA_CMD_WRITE_FEABITS, 0x00, 0x00, 0x00, (feabits_value >> 8) & 0xFF, feabits_value & 0xFF};

    ESP_LOGI(TAG, " - hex: %02X %02X %02X %02X %02X %02X", cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5]);

    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd, sizeof(cmd)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_WRITE_FEABITS failed for value 0x%04X", feabits_value);
        fpga_exec_exit_offline();
        return 1;
    }

    fpga_exec_wait_busy(5000);
    return 0;
}

static int fpga_exec_program_feature_row(uint64_t value)
{
    /* Prepare and send write 64 bits FEATURE_ROW command */
    uint8_t cmd[] = {
        FPGA_CMD_WRITE_FEATURE_ROW, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        (uint8_t)(value >> 56), (uint8_t)(value >> 48), (uint8_t)(value >> 40), (uint8_t)(value >> 32),
        (uint8_t)(value >> 24), (uint8_t)(value >> 16), (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};

    ESP_LOGI(TAG, "Programming FEATURE_ROW with value: 0x%016llX", value);
    ESP_LOGI(TAG, "Command buffer for WRITE_FEATURE_ROW (size %u):", sizeof(cmd));
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, cmd, sizeof(cmd), ESP_LOG_INFO);

    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd, sizeof(cmd)) != 0)
    {
        ESP_LOGE(TAG, "FPGA_CMD_WRITE_FEATURE_ROW failed for value 0x%016llX", value);
        fpga_exec_exit_offline(); /* Assuming fpga_cmd_exit_offline() is appropriate here */
        return 1;
    }

    vTaskDelay(pdMS_TO_TICKS(5000)); /* Delay after programming, consult datasheet for appropriate timing */
    return 0;
}

static void fpga_dump_feabits(uint16_t feabits_data)
{
    const struct
    {
        uint16_t mask;
        const char *name;
    } feabit_name_map[] = {
        {FPGA_MACHXO2_FEABITS_SECURE_PWD, "FPGA_MACHXO2_FEABITS_SECURE_PWD (Bit 0: Password protect configuration)"},
        {FPGA_MACHXO2_FEABITS_DEC_ONLY, "FPGA_MACHXO2_FEABITS_DEC_ONLY (Bit 1: Decryption only)"},
        {FPGA_MACHXO2_FEABITS_PWD_ENABLE, "FPGA_MACHXO2_FEABITS_PWD_ENABLE (Bit 2: Password enable for feature bits)"},
        {FPGA_MACHXO2_FEABITS_PWD_ENABLE_ALL, "FPGA_MACHXO2_FEABITS_PWD_ENABLE_ALL (Bit 3: Password enable for all non-volatile memory)"},
        {FPGA_MACHXO2_FEABITS_MY_ASSP_ENABLE, "FPGA_MACHXO2_FEABITS_MY_ASSP_ENABLE (Bit 4: Enable MyASSP)"},
        {FPGA_MACHXO2_FEABITS_PRG_PERSIST_DIS, "FPGA_MACHXO2_FEABITS_PRG_PERSIST_DIS (Bit 5: Disable PROGRAMN persistence)"},
        {FPGA_MACHXO2_FEABITS_INIT_PERSIST_EN, "FPGA_MACHXO2_FEABITS_INIT_PERSIST_EN (Bit 6: Enable INITN persistence)"},
        {FPGA_MACHXO2_FEABITS_DONE_PERSIST_EN, "FPGA_MACHXO2_FEABITS_DONE_PERSIST_EN (Bit 7: Enable DONE persistence)"},
        {FPGA_MACHXO2_FEABITS_JTAG_PERSIST_DIS, "FPGA_MACHXO2_FEABITS_JTAG_PERSIST_DIS (Bit 8: Disable JTAG port persistence)"},
        {FPGA_MACHXO2_FEABITS_SSP_PERSIST_DIS, "FPGA_MACHXO2_FEABITS_SSP_PERSIST_DIS (Bit 9: Disable SSPI port persistence)"},
        {FPGA_MACHXO2_FEABITS_I2C_PERSIST_DIS, "FPGA_MACHXO2_FEABITS_I2C_PERSIST_DIS (Bit 10: Disable I2C port persistence)"},
        {FPGA_MACHXO2_FEABITS_MSPI_PERSIST_EN, "FPGA_MACHXO2_FEABITS_MSPI_PERSIST_EN (Bit 11: Enable Master SPI port persistence)"},
        {FPGA_MACHXO2_FEABITS_BOOTSEL0, "FPGA_MACHXO2_FEABITS_BOOTSEL0 (Bit 12: Boot Select Bit 0)"},
        {FPGA_MACHXO2_FEABITS_BOOTSEL1, "FPGA_MACHXO2_FEABITS_BOOTSEL1 (Bit 13: Boot Select Bit 1)"},
        {FPGA_MACHXO2_FEABITS_RES2, "FPGA_MACHXO2_FEABITS_RES2 (Bit 14: Reserved)"},
        {FPGA_MACHXO2_FEABITS_RES1, "FPGA_MACHXO2_FEABITS_RES1 (Bit 15: Reserved)"}};

    ESP_LOGI(TAG, "  FEABITS: 0x%04X", feabits_data);

    bool any_set = false;
    for (int i = 0; i < sizeof(feabit_name_map) / sizeof(feabit_name_map[0]); i++)
    {
        if ((feabits_data & feabit_name_map[i].mask) == feabit_name_map[i].mask)
        {
            ESP_LOGI(TAG, "    - %s", feabit_name_map[i].name);
            any_set = true;
        }
    }
    if (!any_set)
    {
        ESP_LOGI(TAG, "  (No specific FEABITS recognized as set or all are 0)");
    }
}

static int fpga_exec_read_feabits(uint16_t *feabits)
{
    uint8_t data[2];
    uint8_t cmd_read_feabits[] = {FPGA_CMD_READ_FEABITS, 0x00, 0x00, 0x00};
    if (i2c_write_read(FPGA_ADDR_CONF_PRIMARY, cmd_read_feabits, sizeof(cmd_read_feabits), data, sizeof(data)) != 0)
    {
        ESP_LOGE(TAG, "Failed to read back FPGA FEABITS after writing.");
        return 1;
    }
    *feabits = (data[0] << 8) | data[1];
    return 0;
}

static int fpga_exec_dump_feabits()
{
    /* Read back and dump FEABITS for verification */
    uint16_t feabits;
    fpga_exec_read_feabits(&feabits);
    fpga_dump_feabits(feabits);
    return 0;
}

static int do_fpga_read_feature_row(int argc, char **argv)
{
    uint8_t feature_row_data[8]; /* Feature Row is 64 bits (8 bytes) */

    /* Command to read feature row */
    uint8_t cmd_read_feature_row[] = {FPGA_CMD_READ_FEATURE_ROW, 0x00, 0x00, 0x00};
    if (i2c_write_read(FPGA_ADDR_CONF_PRIMARY, cmd_read_feature_row, sizeof(cmd_read_feature_row), feature_row_data, sizeof(feature_row_data)) != 0)
    {
        ESP_LOGE(TAG, "Failed to read FPGA Feature Row");
        return 1;
    }

    ESP_LOGI(TAG, "FPGA Feature Row (8 bytes):");
    char hex_buffer[8 * 3 + 1]; /* Each byte takes 2 hex chars + 1 space, plus null terminator */
    for (int i = 0; i < sizeof(feature_row_data); i++)
    {
        sprintf(&hex_buffer[i * 3], "%02X ", feature_row_data[i]);
    }
    ESP_LOGI(TAG, "%s", hex_buffer);

    return 0;
}

static int do_fpga_read_feabits(int argc, char **argv)
{
    fpga_exec_enter_offline();
    fpga_exec_reset_cfg_address();

    uint16_t feabits;
    fpga_exec_read_feabits(&feabits);
    fpga_dump_feabits(feabits);

    fpga_exec_exit_offline();

    return 0;
}

static int do_fpga_write_feabits(int argc, char **argv)
{
    if (argc < 2)
    {
        ESP_LOGE(TAG, "Usage: fpga_write_feabits <%d_bit_binary_value>", FPGA_FEABITS_WIDTH);
        ESP_LOGE(TAG, "Example: fpga_write_feabits 1000000000000001");
        return 1;
    }

    const char *binary_value_str = argv[1];
    if (strlen(binary_value_str) != FPGA_FEABITS_WIDTH)
    {
        ESP_LOGE(TAG, "Invalid binary value length: %s. Must be %d bits.", binary_value_str, FPGA_FEABITS_WIDTH);
        return 1;
    }

    uint16_t feabits_value = 0;
    for (int i = 0; i < FPGA_FEABITS_WIDTH; i++)
    {
        if (binary_value_str[i] == '1')
        {
            feabits_value |= (1 << (FPGA_FEABITS_WIDTH - 1 - i)); /* MSB first */
        }
        else if (binary_value_str[i] != '0')
        {
            ESP_LOGE(TAG, "Invalid character in binary string: '%c' at position %d. Only '0' or '1' allowed.", binary_value_str[i], i);
            return 1;
        }
    }

    fpga_exec_enter_offline();
    ESP_LOGI(TAG, "Enter offline mode...");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "Reset config address...");
    fpga_exec_reset_cfg_address();

    ESP_LOGI(TAG, "FEABITS before erase...");
    fpga_exec_dump_feabits();
    ESP_LOGI(TAG, "Erase feature bits...");
    fpga_exec_erase(ERASE_FLASH);
    ESP_LOGI(TAG, "FEABITS after erase...");
    fpga_exec_dump_feabits();

    ESP_LOGI(TAG, "Reset config address...");
    fpga_exec_reset_cfg_address();
    ESP_LOGI(TAG, "Write feature bits...");
    fpga_exec_program_feabits(feabits_value);

    ESP_LOGI(TAG, "Finishing programming...");
    uint8_t cmd_program_done[] = {FPGA_CMD_PROGRAM_DONE, 0x00, 0x00, 0x00};
    if (i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd_program_done, sizeof(cmd_program_done)) != 0)
    {
        ESP_LOGE(TAG, "Failed to write program done");
        /* Attempt to disable config interface even on failure */
        uint8_t cmd_disable_fail[] = {FPGA_CMD_DISABLE_CFG_IF, 0x00, 0x00};
        i2c_write(FPGA_ADDR_CONF_PRIMARY, cmd_disable_fail, sizeof(cmd_disable_fail));
        return 1;
    }

    fpga_exec_wait_busy(1000);

    ESP_LOGI(TAG, "Reading back FEABITS");
    fpga_exec_dump_feabits();

    fpga_exec_exit_offline();

    ESP_LOGI(TAG, "FPGA configuration interface disabled.");

    ESP_LOGI(TAG, "FEABITS write process completed for 0x%04X. A REFRESH (0x79) or power cycle may be needed for changes to take effect.", feabits_value);

    return 0;
}

static int do_fpga_dl(int argc, char **argv)
{
    if (argc < 2)
    {
        return fpga_dl_file("/storage/fpga.bit");
    }
    return fpga_dl_file(argv[1]);
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

void register_fpga_read_feature_row_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "fpga_feature_row",
        .help = "Read FPGA Feature Row via I2C",
        .hint = NULL,
        .func = &do_fpga_read_feature_row,
        .argtable = NULL};
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void register_fpga_read_feabits_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "fpga_feabits",
        .help = "Read FPGA FEABITS via I2C",
        .hint = NULL,
        .func = &do_fpga_read_feabits,
        .argtable = NULL};
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void register_fpga_write_feabits_command(void)
{
    const esp_console_cmd_t cmd = {
        .command = "fpga_write_feabits",
        .help = "Write FPGA FEABITS via I2C. Arg: <16_bit_binary_value>",
        .hint = "1000000000000001",
        .func = &do_fpga_write_feabits,
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

void fpga_ensure_febits(uint16_t expected)
{
    uint16_t feabits;
    fpga_exec_enter_offline();
    fpga_exec_reset_cfg_address();
    fpga_exec_read_feabits(&feabits);
    fpga_dump_feabits(feabits);

    if (feabits != expected)
    {
        ESP_LOGW(TAG, " - do not match 0x%04" PRIx16 ", reprogramming", expected);
        ESP_LOGI(TAG, " - Enter offline mode...");
        fpga_exec_enter_offline();
        fpga_exec_reset_cfg_address();
        ESP_LOGI(TAG, " - Erase feature bits...");
        fpga_exec_erase(ERASE_FLASH);
        fpga_exec_reset_cfg_address();
        ESP_LOGI(TAG, " - Write feature bits...");
        fpga_exec_program_feabits(expected);
        fpga_exec_exit_offline();
        ESP_LOGI(TAG, " - Done...");
    }
}

void fpga_init()
{
    ESP_LOGI(TAG, "Initializing FPGA");
    gpio_set_direction(GPIO_FPGA_PROGRAMN, GPIO_MODE_OUTPUT);

    fpga_clk_init();
    // fpga_clk_old_init(2000000);
    register_fpga_id_command();
    register_fpga_status_command();
    register_fpga_dl_command();
    register_fpga_erase_command();
    register_fpga_prog_command();
    register_fpga_read_feature_row_command();
    register_fpga_read_feabits_command();
    register_fpga_write_feabits_command();

    fpga_start_socket_listener();

    fpga_prog(true);
    fpga_ensure_febits(FPGA_FEABITS_DEFAULT);
    fpga_dl_file("/storage/fpga.bit");
    fpga_prog(false);
}
