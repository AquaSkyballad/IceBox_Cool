idf_component_register(SRCS "icebox.c"
                             "NCP81239.c"
                             "tft_display.c"
                             "thumbbwheel.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_i2c
                             esp_driver_gpio
                             esp_driver_spi
                             driver
                             nvs_flash)
