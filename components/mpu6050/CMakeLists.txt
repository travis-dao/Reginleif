# "driver" works on v5.2 and later (on v5.3+ it pulls in esp_driver_i2c)
idf_component_register(
    SRCS "mpu6050.c"
    INCLUDE_DIRS "include"
    REQUIRES driver
    PRIV_REQUIRES esp_timer
)