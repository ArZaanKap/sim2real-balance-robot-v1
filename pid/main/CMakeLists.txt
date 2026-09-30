idf_component_register(SRCS "pid_main.c" "motor.c" "encoder.c" "imu.c" "pid.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES sh2 esp_driver_gpio esp_driver_spi esp_timer
                                     esp_driver_pcnt esp_driver_ledc)
