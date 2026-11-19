#ifndef TYPES_h
#define TYPES_h

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>  // for .exp()

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

template <typename T1, typename T2, typename T3>
using Triple = std::tuple<T1, T2, T3>;

using Mat3 = Eigen::Matrix3f;
using Vec3 = Eigen::Vector3f;

template <typename T, int Rows, int Cols, int Options = Eigen::ColMajor>
using Mat = Eigen::Matrix<T, Rows, Cols, Options>;

using Sparse = Eigen::Sparse;

template <typename T>
using Vector = std::vector<T, Eigen::aligned_allocator<T>>;

#endif