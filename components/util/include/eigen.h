#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>  // for .exp()

using MatXf = Eigen::MatrixXf;
using VecXf = Eigen::VectorXf;

using Mat3f = Eigen::Matrix3f;
using Vec3f = Eigen::Vector3f;

template <typename T, int Rows, int Cols, int Options = Eigen::ColMajor>
using Mat = Eigen::Matrix<T, Rows, Cols, Options>;

using Sparse = Eigen::Sparse;

template <typename T>
using Vector = std::vector<T, Eigen::aligned_allocator<T>>;