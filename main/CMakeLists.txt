idf_component_register(
    SRCS
    "led.c"
    "boot_button.c"
    "channel_config.c"
    "web.c"
    "main.c"
    "serial_protocol.c"
    "voltage.c"
    "i2s.c"
    "ts_stream.c"
    "i2c.c"
    "fpga.c"
    "wifi.c"
    "cmd.c"
    "gpio_cmd.c"
    "telnet.c"
    "tuner/tuner.c"
    "tuner/cx24116_linux.c"
    "tuner/linux_adapter.c"
    
    INCLUDE_DIRS 
    "."
    "tuner"

    EMBED_FILES "favicon.ico"
    PRIV_REQUIRES fatfs driver esp_driver_gpio esp_hw_support esp_psram esp_timer esp_wifi wpa_supplicant esp_event nvs_flash esp_http_server json mbedtls )
