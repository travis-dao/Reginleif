#ifndef TYPES_h
#define TYPES_h

#include <cstdint>
#include <math.h>
#include <array>
#include <Eigen/Dense>

constexpr float k_rad_to_deg = 57.2957795131f;
constexpr float k_deg_to_rad = 0.01745329252f;
constexpr float k_pi = static_cast<float>(M_PI);
constexpr float k_epsilon = 1e-6f; // float error
constexpr float k_float_max = std::numeric_limits<float>::max();

struct Theta3 {
  	float coxa, femur, tibia;
};

struct Vec3 {
	float x, y, z = 0.0f;

	Vec3 operator+(const Vec3& other) const;
	Vec3& operator+=(const Vec3& other);
	Vec3 operator-(const Vec3& other) const;
	Vec3 operator*(float scalar) const;
	Vec3& operator*=(float scalar);
	Vec3 operator*(const Vec3& other) const;
	Vec3& operator*=(const Vec3& other);
	Vec3 operator/(float scalar) const;
	bool operator==(const Vec3& other);
	void print() const;
	void println() const;
	float dis(const Vec3& other) const;
	float dot(const Vec3& other) const;
	float dot2D(const Vec3& other) const;
	float cross2D(const Vec3& other) const;
};

float dis(const float a, const float b);

template <typename T, std::size_t N>
using Array = std::array<T, N>;

// using Vec3 = Eigen::Vector3f;
using Mat3 = Eigen::Matrix3f;


#endif