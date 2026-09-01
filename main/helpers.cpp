#include <math.h>
#include <algorithm>
#include <random>
#include <cstdint>

#include "helpers.h"
#include "leg.h"
#include "base.h"

#pragma region Vec3

Vec3 Vec3::operator+(const Vec3& other) const {
  	return { x + other.x, y + other.y, z + other.z };
}

Vec3& Vec3::operator+=(const Vec3& other) {
	x += other.x;
	y += other.y;
	z += other.z;
	return *this;
}

Vec3 Vec3::operator-(const Vec3& other) const {
  	return { x - other.x, y - other.y, z - other.z };
}

Vec3 Vec3::operator*(float scalar) const {
  	return { x * scalar, y * scalar, z * scalar };
}

Vec3& Vec3::operator*=(float scalar) {
	x *= scalar;
	y *= scalar;
	z *= scalar;
	return *this;
}

Vec3 Vec3::operator*(const Vec3& other) const {
  	return { x * other.x, y * other.y, z * other.z };
}

Vec3& Vec3::operator*=(const Vec3& other) {
	x *= other.x;
	y *= other.y;
	z *= other.z;
	return *this;
}

Vec3 Vec3::operator/(float scalar) const {
  	return { x / scalar, y / scalar, z / scalar };
}

/**
 * @brief Computes the cross product of this vector and another.
 *
 * @param other The vector to cross with.
 * @return Vec3 The resulting perpendicular vector.
 */
Vec3 Vec3::operator%(const Vec3& other) const {
	return {
		y * other.z - z * other.y,
		z * other.x - x * other.z,
		x * other.y - y * other.x
	};
}

bool Vec3::operator==(const Vec3& other) {
  	return (x == other.x) && (y == other.y) && (z == other.z);
}

void Vec3::print() const {
  	printf("(%f, %f, %f)", x, y, z);
}

void Vec3::println() const {
	print();
	printf("\n");
}

/**
 * @brief Returns a unit-length version of this vector.
 *
 * Divides each component by the vector's magnitude. Does not guard
 * against division by zero if the vector has zero magnitude.
 *
 * @return Vec3 The normalized (unit) vector.
 */
Vec3 Vec3::normalized() const {
	float x = this->x;
	float y = this->y;
	float z = this->z;
	float magnitude = sqrt(x * x + y * y + z * z);

	x /= magnitude;
	y /= magnitude;
	z /= magnitude;

	return Vec3 {x, y, z};
}

/**
 * @brief Computes the magnitude of this vector after masking/scaling by another vector.
 *
 * Multiplies each component of this vector by the corresponding
 * component of @p on, then returns the Euclidean length of the result.
 * Useful for measuring magnitude along only certain axes (e.g. passing
 * a vector of 1s/0s as a mask).
 *
 * @param on Vector used to scale/mask each axis before computing magnitude.
 * @return float The resulting magnitude.
 */
float Vec3::magnitude(Vec3 on) const {
	float x = this->x * on.x;
	float y = this->y * on.y;
	float z = this->z * on.z;
	return sqrt(x*x + y*y + z*z);
}

/**
 * @brief Clamps each component of a vector to the given range.
 *
 * @param v The vector to clamp.
 * @param min_val Minimum allowed value for each component.
 * @param max_val Maximum allowed value for each component.
 * @return Vec3 The component-wise clamped vector.
 */
Vec3 clamp_vec3(const Vec3& v, float min_val, float max_val) {
	return Vec3 {
		std::clamp(v.x, min_val, max_val),
		std::clamp(v.y, min_val, max_val),
		std::clamp(v.z, min_val, max_val)
	};
}

#pragma endregion

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

/**
 * @brief Converts a servo angle in degrees to a PWM pulse value.
 *
 * Clamps the input angle to the valid [0, 180] degree range, then maps
 * it linearly to the servo's configured min/max pulse values.
 *
 * @param angle Desired servo angle in degrees.
 * @return uint16_t Corresponding PWM pulse value.
 */
uint16_t angle_to_pulse(float angle) {
	angle = std::clamp(angle, 0.0f, 180.0f);
	return map_float(angle, 0.0f, 180.0f, ServoConfig::SERVO_MIN_PULSE, ServoConfig::SERVO_MAX_PULSE);
}

/**
 * @brief Determines whether two legs are adjacent (diagonally opposite pairing check).
 *
 * Checks the parity relationship between the reference leg index and
 * the candidate leg index to determine adjacency in the leg numbering
 * scheme.
 *
 * @param ref_leg_idx Index of the reference leg.
 * @param leg_idx Index of the leg being checked.
 * @return bool True if the legs are considered adjacent, false otherwise.
 */
bool is_adjacent_leg(int ref_leg_idx, int leg_idx) {
  	return ((ref_leg_idx + 1) % 2) == (leg_idx % 2);
}

/**
 * @brief Determines whether two legs are diagonally opposite each other.
 *
 * A leg is considered "opposite" the reference leg if it is a different
 * leg but shares the same index parity (same side of the diagonal
 * gait pairing).
 *
 * @param ref_leg_idx Index of the reference leg.
 * @param leg_idx Index of the leg being checked.
 * @return bool True if the legs are opposite each other, false otherwise.
 */
bool is_opposite_leg(int ref_leg_idx, int leg_idx) {
  	return (ref_leg_idx != leg_idx) && (ref_leg_idx % 2 == leg_idx % 2);
}

#pragma endregion

#pragma region Interpolation

/**
 * @brief Linearly interpolates between two vectors.
 *
 * @param a Start vector (returned when t = 0).
 * @param b End vector (returned when t = 1).
 * @param t Interpolation factor, typically in [0, 1].
 * @return Vec3 The interpolated vector.
 */
Vec3 lerp(const Vec3& a, const Vec3& b, float t) {
  	return a + (b - a) * t;
}

/**
 * @brief Interpolates a value along a sine arc, useful for smooth lift/step motions.
 *
 * Computes start + height * sin(pi * t), producing a curve that begins
 * and ends at @p start and peaks at start + height when t = 0.5.
 *
 * @param start Baseline value at t = 0 and t = 1.
 * @param height Peak height added at the midpoint of the arc.
 * @param t Interpolation factor, typically in [0, 1].
 * @return float The interpolated value along the sine arc.
 */
float sin_interpolation(float start, float height, float t) {
  	return start + height * sin(M_PI * t);
}

#pragma endregion