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

float Vec3::dis(const Vec3& other) const {
	Vec3 diff = *this - other;
    return std::sqrt( diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
}

float Vec3::dot(const Vec3& other) const {
    return x * other.x + y * other.y + z * other.z;
}
float Vec3::dot2D(const Vec3& other) const {
    return x * other.x + y * other.y;
}

float Vec3::cross2D(const Vec3& other) const {
    return x * other.y - y * other.x;
}

Vec3 Vec3::rotate(const Mat3& rot_mat) {
    Eigen::Vector3f p(this->x, this->y, this->z);
    Eigen::Vector3f result = rot_mat * p;

    return Vec3 {result.x(), result.y(), result.z()};
}

Vec3 clamp_vec3(const Vec3& v, float min_val, float max_val) {
	return Vec3 {
		std::clamp(v.x, min_val, max_val),
		std::clamp(v.y, min_val, max_val),
		std::clamp(v.z, min_val, max_val)
	};
}

#pragma endregion

float dis(const float a, const float b) {
	return std::sqrt(a * a + b * b);
}