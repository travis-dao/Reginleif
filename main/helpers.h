#ifndef HELPERS_H
#define HELPERS_H

#include <stdint.h>
#include <thread>
#include <chrono>

#pragma region structs and enums

struct Vec3 {
	float x, y, z;

	Vec3 operator+(const Vec3& other) const;
	Vec3& operator+=(const Vec3& other);
	Vec3 operator-(const Vec3& other) const;
	Vec3 operator*(float scalar) const;
	Vec3& operator*=(float scalar);
	Vec3 operator*(const Vec3& other) const;
	Vec3& operator*=(const Vec3& other);
	Vec3 operator/(float scalar) const;
	Vec3 operator%(const Vec3& other) const;      // Cross product
	bool operator==(const Vec3& other);
	void print() const;
	void println() const;
	Vec3 normalized() const;
	float magnitude(Vec3 on = Vec3 { 1, 1, 1}) const;
};

Vec3 clamp_vec3(const Vec3& v, float min_val, float max_val);

#pragma endregion

bool is_adjacent_leg(int ref_leg_idx, int leg_idx);
bool is_opposite_leg(int ref_leg_idx, int leg_idx);
uint16_t angle_to_pulse(float angle);

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

#endif