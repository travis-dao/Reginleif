#include <math.h>

#include "servo.h"
#include "pca9685.h"

static uint16_t servoAngleToCounts(float deg)
{
    const float counts = ServoConfig::SERVO_MIN_PULSE + (ServoConfig::SERVO_MAX_PULSE - ServoConfig::SERVO_MIN_PULSE) * deg / 180.0f;
    return static_cast<uint16_t>(std::lround(counts));
}

esp_err_t servoSetAngle(Pca9685 &pca, uint8_t ch, float deg)
{
    return pca.setDuty(ch, servoAngleToCounts(deg));
}

esp_err_t servoSetAngles(Pca9685 &pca, const float *deg, size_t count)
{
    uint16_t duty[NUM_CHANNELS];

    if (count > NUM_CHANNELS) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < count; i++) {
        duty[i] = servoAngleToCounts(deg[i]);
    }
    
    return pca.setDuties(0, duty, count);
}