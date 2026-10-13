#include <math.h>

#include "servo.h"

esp_err_t servo_set_angle(pca9685::Pca9685 &pca, uint8_t ch, float deg)
{
    const float counts = ServoConfig::SERVO_MIN_PULSE +
                         (ServoConfig::SERVO_MAX_PULSE - ServoConfig::SERVO_MIN_PULSE) * deg / 180.0f;
    return pca.setDuty(ch, static_cast<uint16_t>(std::lround(counts)));
}