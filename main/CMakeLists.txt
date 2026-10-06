idf_component_register(SRCS "main.c"
                            "inmp441_doa.c"
                            "max98357.c"
                            "ili9341.c"
                            "uart_stream.c"
                            "scope_display.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_i2s esp_driver_spi esp_driver_gpio esp_driver_uart esp_lcd)
