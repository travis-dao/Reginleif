# ESP-IDF 5.2.x: the I2C master driver is part of the "driver" component.
# (On ESP-IDF >= 5.3 you can use "esp_driver_i2c" instead.)
idf_component_register(
    SRCS "imu/mpu6050.cpp" "imu/ism330.cpp"
    INCLUDE_DIRS "imu/include" "i2c/include"
    REQUIRES driver core util
    PRIV_REQUIRES esp_timer
)