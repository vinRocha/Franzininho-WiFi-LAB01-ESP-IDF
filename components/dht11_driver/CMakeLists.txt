idf_component_register(SRCS "dht11_driver.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_driver_gpio esp_timer)
