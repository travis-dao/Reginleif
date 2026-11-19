idf_component_register(
    SRCS "servo.cpp" "servo_driver.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
)