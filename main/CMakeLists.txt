set(COMPONENT_PRIV_REQUIRES esp_peripherals)

idf_component_register(
    SRCS
    "main.c"
    "voltage.c"
    "i2c.c"
    "fpga.c"
    "cx24116.c"
    "wifi.c"
    "telnet.c"
    "cx24116_linux.c"
    "linux_adapter.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES fatfs esp_psram esp_wifi wpa_supplicant esp_event nvs_flash)
