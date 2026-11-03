#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>  // for .exp()

using MatX = Eigen::MatrixXf;
using VecX = Eigen::VectorXf;
using Sparse = Eigen::Sparse;

template <typename T>
using Vector = std::vector<T, Eigen::aligned_allocator<T>>;