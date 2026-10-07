idf_component_register(SRCS "oled_display.c" "main.c"
                            "inmp441_doa.c"
                            "max98357.c"
                            "ili9341.c"
                            "uart_stream.c"
                            "scope_display.c"
                            "time_sync.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_i2s esp_driver_spi esp_driver_gpio esp_driver_uart esp_lcd espressif__ssd1306)
