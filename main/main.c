/* Basic console example (esp_console_repl API)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "tuner.h"
#include "voltage.h"
#include "i2c.h"
#include "i2s.h"
#include "fpga.h"
#include "wifi.h"
#include "telnet.h"
#include "web.h"
#include "cmd.h"
#include "led.h"


static const char *TAG = "satsnooper";



void app_main(void)
{
    led_init();
    led_set_color(32, 0, 0);

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_init());
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_loop_create_default());

    /* init NVS */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    /* Mount the FAT image in the 'storage' partition */
    esp_vfs_fat_mount_config_t mount_config = {
        .max_files = 4,
        .format_if_mount_failed = false};
    wl_handle_t wl_handle;
    err = esp_vfs_fat_spiflash_mount_rw_wl("/storage", "storage", &mount_config, &wl_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount FAT image in 'storage' partition (%s)", esp_err_to_name(err));
    }

    led_set_color(32, 32, 0);
    
    voltage_init();
    i2c_init();
    fpga_init();
    i2s_init();
    tuner_init_base();

    wifi_start();
    telnet_start();
    web_start();
    cmd_start();

    led_set_color(0, 32, 0);
}
