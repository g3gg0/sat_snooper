#include <stdarg.h>  /* For va_list, va_start, va_end */
#include "esp_log.h" /* For esp_log_writev, ESP_LOG_INFO */

#include "linux_adapter.h"
#include "driver/i2c.h"

/* Default tag for printk messages. You can change "printk" to any other tag you prefer. */
#define PRINTK_TAG "printk"
#define TAG_CX "cx"

void printk(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    /* ESP_LOGI is a macro; use esp_log_writev to correctly handle va_list */
    esp_log_writev(ESP_LOG_INFO, PRINTK_TAG, fmt, args);
    va_end(args);
}

int i2c_transfer(struct i2c_adapter *state, void *data, uint32_t length)
{
    struct i2c_msg *msg = (struct i2c_msg *)data;
    int ret = 0;

    for (int i = 0; i < length; i++)
    {
        if (msg[i].flags & I2C_M_RD)
        {
            /* Read operation */
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (msg[i].addr << 1) | I2C_MASTER_READ, true);
            i2c_master_read(cmd, msg[i].buf, msg[i].len, I2C_MASTER_LAST_NACK);
            i2c_master_stop(cmd);
            ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
            i2c_cmd_link_delete(cmd);
        }
        else
        {
            /* Write operation */
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (msg[i].addr << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write(cmd, msg[i].buf, msg[i].len, true);
            i2c_master_stop(cmd);
            ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
            i2c_cmd_link_delete(cmd);
        }

        if (ret != ESP_OK)
        {
            return -EREMOTEIO;
        }
    }

    return length;
}

int request_firmware(const struct firmware **fw_, const char *name, void *parent)
{
    const char *fw_path = "/storage/cx24116.fw";

    FILE *f = fopen(fw_path, "rb");
    if (!f)
    {
        ESP_LOGE(TAG_CX, "Failed to open firmware file: %s", fw_path);
        return 1;
    }

    fseek(f, 0, SEEK_END);
    long fw_size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (fw_size <= 0)
    {
        ESP_LOGE(TAG_CX, "Firmware file is empty or invalid size: %s", fw_path);
        fclose(f);
        return 1;
    }

    uint8_t *fw_data = malloc(fw_size);
    if (!fw_data)
    {
        ESP_LOGE(TAG_CX, "Failed to allocate memory for firmware (%ld bytes)", fw_size);
        fclose(f);
        return 1;
    }

    if (fread(fw_data, 1, fw_size, f) != fw_size)
    {
        ESP_LOGE(TAG_CX, "Failed to read firmware file: %s", fw_path);
        free(fw_data);
        fclose(f);
        return 1;
    }
    fclose(f);

    struct firmware **fw = (struct firmware **)fw_;
    *fw = malloc(sizeof(struct firmware));
    (*fw)->data = fw_data;
    (*fw)->size = fw_size;

    return 0;
}

int release_firmware(const struct firmware *fw_)
{
    struct firmware *fw = (struct firmware *)fw_;
    free(fw->data);
    free(fw);
    return 0;
}