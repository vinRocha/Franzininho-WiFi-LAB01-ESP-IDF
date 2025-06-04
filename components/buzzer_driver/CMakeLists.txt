idf_component_register(SRCS "buzzer_driver.c"
                    REQUIRES
                        esp_driver_dac
                    INCLUDE_DIRS "include")
