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


#pragma region Mat3

// Matrix * Vector
Vec3 Mat3::operator*(const Vec3& v) const {
	return Vec3{
		rows[0].x*v.x + rows[0].y*v.y + rows[0].z*v.z,
		rows[1].x*v.x + rows[1].y*v.y + rows[1].z*v.z,
		rows[2].x*v.x + rows[2].y*v.y + rows[2].z*v.z
	};
}

// Matrix * Matrix
Mat3 Mat3::operator*(const Mat3& o) const {
	// Need columns of `o` as Vec3s to dot against
	Vec3 col0{o.rows[0].x, o.rows[1].x, o.rows[2].x};
	Vec3 col1{o.rows[0].y, o.rows[1].y, o.rows[2].y};
	Vec3 col2{o.rows[0].z, o.rows[1].z, o.rows[2].z};

	Mat3 r;
	for (int i = 0; i < 3; ++i) {
		const Vec3& row = rows[i];
		r.rows[i] = Vec3{
		row.x*col0.x + row.y*col0.y + row.z*col0.z,
		row.x*col1.x + row.y*col1.y + row.z*col1.z,
		row.x*col2.x + row.y*col2.y + row.z*col2.z
		};
	}
	return r;
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

esp_err_t servo_set_angle(pca9685_handle_t pca, uint8_t ch, float deg)
{
    deg = std::clamp(deg, 0.0f, 180.0f);
    float counts = ServoConfig::SERVO_MIN_PULSE + (ServoConfig::SERVO_MAX_PULSE - ServoConfig::SERVO_MIN_PULSE) * deg / 180.0f;
    return pca9685_set_duty(pca, ch, static_cast<uint16_t>(std::lround(counts)));
}