#ifndef HELPERS_H
#define HELPERS_H

#include <stdint.h>
#include <thread>
#include <chrono>

#include "pca9685.h"
#include "types.h"

bool is_adjacent_leg(int ref_leg_idx, int leg_idx);
bool is_opposite_leg(int ref_leg_idx, int leg_idx);

// /**
//  * @brief Converts a servo angle in degrees to a PWM pulse value.
//  *
//  * @param angle Desired servo angle in degrees.
//  * @return uint16_t Corresponding PWM pulse value.
//  */
// uint16_t angle_to_pulse(float angle);

// interpolation
namespace Interpolation {
	Vec3 lerp(const Vec3& a, const Vec3& b, float t);
	float sin_interpolation(float start, float height, float t);
	Vec3 circular_interpolation(float radius, float start_angle, float angle_delta, float t);
}

namespace ThisThread {
    inline void sleep_for(std::chrono::milliseconds) { /* no-op on host */ }
}

constexpr std::chrono::milliseconds operator""ms(unsigned long long ms) {
    return std::chrono::milliseconds{
        static_cast<std::chrono::milliseconds::rep>(ms)
    };
}

esp_err_t servo_set_angle(pca9685_handle_t pca, uint8_t ch, float deg);

#endif