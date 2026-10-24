#ifndef TYPES_h
#define TYPES_h

#include <cstdint>
#include <math.h>

constexpr float RAD_TO_DEG = static_cast<float>(180.0f / M_PI);
constexpr float DEG_TO_RAD = static_cast<float>(M_PI / 180.0f);
constexpr float PI = static_cast<float>(M_PI);

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
	Vec3 operator%(const Vec3& other) const;      // Cross product
	bool operator==(const Vec3& other);
	void print() const;
	void println() const;
	Vec3 normalized() const;

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
	float magnitude(Vec3 on = Vec3 { 1, 1, 1}) const;
};

struct Mat3 {
	Vec3 rows[3] = { {1,0,0}, {0,1,0}, {0,0,1} }; // identity matrix by default

	// Matrix * Vector
	Vec3 operator*(const Vec3& v) const;

	// Matrix * Matrix
	Mat3 operator*(const Mat3& o) const;
};

#endif