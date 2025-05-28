idf_component_register(SRCS "dht11_driver.c"
                    REQUIRES
                        esp_driver_gpio
                        esp_timer
                    INCLUDE_DIRS "include")
