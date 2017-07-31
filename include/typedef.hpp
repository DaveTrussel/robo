#pragma once

#include <Eigen/Dense>

namespace robo {

    constexpr int max_size = 10; // maximum size of dynamically sized vectors and matrices
    using Vector3d  = Eigen::Vector3d;
    using Vector6d  = Eigen::Matrix<double, 6, 1>;
    using VectorXd  = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, max_size, 1>; // set max size to avoid calls to malloc
    using Matrix3d  = Eigen::Matrix3d;
    using Matrix4d  = Eigen::Matrix4d;
    using Matrix6d  = Eigen::Matrix<double, 6, 6>;
    using Matrix6Xd = Eigen::Matrix<double, 6, Eigen::Dynamic, 0, 6, max_size>;
    using MatrixXd  = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, max_size, max_size>;

}
