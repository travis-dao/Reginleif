#include <random>

#include "helpers.h"

#pragma region Leg

/**
 * @brief Linearly remaps a value from one range to another.
 *
 * @param value The input value to remap.
 * @param from_low Lower bound of the input range.
 * @param from_high Upper bound of the input range.
 * @param to_low Lower bound of the output range.
 * @param to_high Upper bound of the output range.
 * @return float The remapped value in the target range.
 */
float map_float(float value, float from_low, float from_high, float to_low, float to_high) {
	return (value - from_low) * (to_high - to_low)
		/ (from_high - from_low) + to_low;
}

bool is_adjacent_leg(int ref_leg_idx, int leg_idx) {
  	return ((ref_leg_idx + 1) % 2) == (leg_idx % 2);
}
bool is_opposite_leg(int ref_leg_idx, int leg_idx) {
  	return (ref_leg_idx != leg_idx) && (ref_leg_idx % 2 == leg_idx % 2);
}

#pragma endregion

#pragma region Interpolation

Vec3 lerp(const Vec3& a, const Vec3& b, float t) {
  	return a + (b - a) * t;
}
float sin_interpolation(float start, float height, float t) {
  	return start + height * sin(M_PI * t);
}

#pragma endregion