set(COMPONENT_PRIV_REQUIRES esp_peripherals)

idf_component_register(
    SRCS
    "web.c"
    "main.c"
    "voltage.c"
    "i2c.c"
    "fpga.c"
    "wifi.c"
    "cmd.c"
    "telnet.c"
    "tuner/tuner.c"
    "tuner/cx24116_linux.c"
    "tuner/linux_adapter.c"
    
    INCLUDE_DIRS 
    "."
    "tuner"

    PRIV_REQUIRES fatfs esp_psram esp_wifi wpa_supplicant esp_event nvs_flash esp_http_server json)
