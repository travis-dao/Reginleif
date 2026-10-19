#include <math.h>

#include "servo.h"
#include "pca9685.h"

esp_err_t servoSetAngle(Pca9685 &pca, uint8_t ch, float deg)
{
    const float counts = ServoConfig::SERVO_MIN_PULSE +
                         (ServoConfig::SERVO_MAX_PULSE - ServoConfig::SERVO_MIN_PULSE) * deg / 180.0f;
    return pca.setDuty(ch, static_cast<uint16_t>(std::lround(counts)));
}