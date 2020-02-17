/**
** @file 
** Defines types based on Eigen, but with a maximum size known at compile time.
** Avoids calls to malloc.
*/

#pragma once

#include <Eigen/Dense>

namespace robo {
  // maximum size of dynamically sized vectors and matrices
  // set max size to avoid calls to malloc
  constexpr int MAX_SIZE = 10;
  using Vector3d  = Eigen::Vector3d;
  using Vector6d  = Eigen::Matrix<double, 6, 1>;
  using VectorXd  = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, MAX_SIZE, 1>;
  using Matrix3d  = Eigen::Matrix3d;
  using Matrix4d  = Eigen::Matrix4d;
  using Matrix6d  = Eigen::Matrix<double, 6, 6>;
  using Matrix6Xd = Eigen::Matrix<double, 6, Eigen::Dynamic, 0, 6, MAX_SIZE>;
  using MatrixXd  = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, MAX_SIZE, MAX_SIZE>;

} // namespace robo
