# ESP-IDF 5.2.x: the I2C master driver is part of the "driver" component.
# (On ESP-IDF >= 5.3 you can use "esp_driver_i2c" instead.)
idf_component_register(
    SRCS "ism330.cpp" "mpu6050"
    INCLUDE_DIRS "include"
    REQUIRES driver
    PRIV_REQUIRES esp_timer
)