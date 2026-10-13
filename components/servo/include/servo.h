#ifndef SERVO_h
#define SERVO_h

#include "esp_err.h"
#include "pca9685.h"

struct ServoLimit {
	float min_deg;
	float max_deg;

	constexpr float clamp(float angle) const {
		return angle < min_deg ? min_deg : (angle > max_deg ? max_deg : angle);
	}
};

struct LegServoLimits {
	ServoLimit coxa;
	ServoLimit femur;
	ServoLimit tibia;
};

namespace ServoConfig {
    constexpr float SERVO_FREQ = 50.0f;
    constexpr float SERVO_MIN_PULSE = 160.0f;
    constexpr float SERVO_MAX_PULSE = 560.0f;

	constexpr LegServoLimits limits[4] = {
		{ // Front Left
			{  45.0f, 180.0f }, 
			{  40.0f, 135.0f }, 
			{  0.0f, 135.0f } 
		},
		{ // Front Right
			{  0.0f, 135.0f }, 
			{  45.0f, 140.0f }, 
			{  45.0f, 180.0f } 
		},
		{ // Back Right
			{  45.0f, 170.0f }, 
			{  45.0f, 140.0f }, 
			{  45.0f, 180.0f }
		 },
		{ // Back Left
			{  10.0f, 135.0f }, 
			{  40.0f, 135.0f }, 
			{  0.0f, 135.0f } 
		},
	};
}

esp_err_t servo_set_angle(pca9685::Pca9685 &pca, uint8_t ch, float deg);

#endif