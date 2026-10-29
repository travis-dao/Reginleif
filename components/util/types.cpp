#include <math.h>
#include <algorithm>
#include <cstdio>

#include "types.h"

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

Vec3 clamp_vec3(const Vec3& v, float min_val, float max_val) {
	return Vec3 {
		std::clamp(v.x, min_val, max_val),
		std::clamp(v.y, min_val, max_val),
		std::clamp(v.z, min_val, max_val)
	};
}

#pragma endregion