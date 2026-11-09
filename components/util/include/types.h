#ifndef TYPES_h
#define TYPES_h

#include <cstdint>
#include <math.h>
#include <array>
#include <optional>

constexpr float RAD_TO_DEG = 57.2957795131f;
constexpr float DEG_TO_RAD = 0.01745329252f;
constexpr float PI = static_cast<float>(M_PI);
constexpr float EPSILON = 1e-6f; // float error
constexpr float FLOAT_MAX = std::numeric_limits<float>::max();

struct Theta3 {
  	float coxa, femur, tibia;
};

template <typename T, std::size_t N>
using Array = std::array<T, N>;

struct Mat3 {
    Array<Array<float, 3>, 3> data{};

    constexpr auto& operator[](size_t i) {
        return data[i];
    }

    constexpr const auto& operator[](size_t i) const {
        return data[i];
    }

    static constexpr Mat3 identity() {
        return Mat3{{
            {
                {1.0f, 0.0f, 0.0f},
                {0.0f, 1.0f, 0.0f},
                {0.0f, 0.0f, 1.0f}
            }
        }};
    }
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
	Vec3 cross(const Vec3& other) const;
	Vec3 rotate(const Mat3& rot_mat) const;
};

float dis(const float a, const float b);

template <typename Tp>
using Optional = std::optional<Tp>;

#endif