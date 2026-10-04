idf_component_register(
    SRCS "test_oled.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_driver_gpio
        esp_driver_i2c
        esp_adc
        esp_driver_ledc
        esp_timer
)
