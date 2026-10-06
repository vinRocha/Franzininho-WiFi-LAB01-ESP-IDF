idf_component_register(SRCS "piano_botoes.c"
                    INCLUDE_DIRS "."
                    REQUIRES input_driver ledc_buzzer_driver)