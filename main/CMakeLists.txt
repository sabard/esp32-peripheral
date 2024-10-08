idf_component_register(
    SRCS "main.c" "wifi.c"
    INCLUDE_DIRS "."
    REQUIRES esp_adc esp_driver_gpio esp_eth esp_netif esp_wifi nvs_flash
    PRIV_REQUIRES esp_msgpack ethernet_init
)
