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

float Vec3::magnitude(Vec3 on) const {
	float x = this->x * on.x;
	float y = this->y * on.y;
	float z = this->z * on.z;
	return sqrt(x*x + y*y + z*z);
}

Vec3 clamp_vec3(const Vec3& v, float min_val, float max_val) {
	return Vec3 {
		std::clamp(v.x, min_val, max_val),
		std::clamp(v.y, min_val, max_val),
		std::clamp(v.z, min_val, max_val)
	};
}

#pragma endregion

#pragma region Leg
float map_float(float value, float from_low, float from_high, float to_low, float to_high) {
	return (value - from_low) * (to_high - to_low)
		/ (from_high - from_low) + to_low;
}

// cuz dumbass servos take pulses instead of angles
uint16_t angle_to_pulse(float angle) {
	angle = std::clamp(angle, 0.0f, 180.0f);
	return map_float(angle, 0.0f, 180.0f, ServoConfig::SERVO_MIN_PULSE, ServoConfig::SERVO_MAX_PULSE);
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