#ifndef TYPES_h
#define TYPES_h

struct Theta3 {
  	float coxa, femur, tibia;
};

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

/**
 * @brief Clamps each component of a vector to the given range.
 *
 * @param v The vector to clamp.
 * @param min_val Minimum allowed value for each component.
 * @param max_val Maximum allowed value for each component.
 * @return Vec3 The component-wise clamped vector.
 */
Vec3 clamp_vec3(const Vec3& v, float min_val, float max_val);


struct Mat3 {
	Vec3 rows[3];

	// Matrix * Vector
	Vec3 operator*(const Vec3& v) const;

	// Matrix * Matrix
	Mat3 operator*(const Mat3& o) const;
};

#endif