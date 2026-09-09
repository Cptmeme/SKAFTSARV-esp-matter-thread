idf_component_register(SRCS "led_test_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES console esp_driver_gpio esp_timer)
