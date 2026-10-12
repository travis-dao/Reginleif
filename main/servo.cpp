#include <math.h>

#include "servo.h"

esp_err_t servo_set_angle(pca9685_handle_t pca, uint8_t ch, float deg)
{
    float counts = ServoConfig::SERVO_MIN_PULSE + (ServoConfig::SERVO_MAX_PULSE - ServoConfig::SERVO_MIN_PULSE) * deg / 180.0f;
    return pca9685_set_duty(pca, ch, static_cast<uint16_t>(std::lround(counts)));
}