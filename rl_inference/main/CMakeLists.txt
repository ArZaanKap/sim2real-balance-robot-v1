# Add "policy.c" and "obs.c" to SRCS as you create them.
idf_component_register(SRCS "policy_main.c" "motor.c" "encoder.c" "imu.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES sh2 esp_driver_gpio esp_driver_spi esp_timer
                                     esp_driver_pcnt esp_driver_ledc)
