#pragma once

#include "twist.hpp"
#include "wrench.hpp"
#include <Eigen/Dense>

namespace robo {

    // TODO think about best place to define those aliases (own alias header file?)
    constexpr int max_size = 10;
    using Vector3d  = Eigen::Vector3d;
    using Vector6d  = Eigen::Matrix<double, 6, 1>;
    using VectorXd  = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, max_size, 1>; // set max size to avoid calls to malloc
    using Matrix3d  = Eigen::Matrix3d;
    using Matrix4d  = Eigen::Matrix4d;
    using Matrix6d  = Eigen::Matrix<double, 6, 6>;
    using Matrix6Xd = Eigen::Matrix<double, 6, Eigen::Dynamic, 0, 6, max_size>;
    using MatrixXd  = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, max_size, max_size>;

    class Frame{
    public:
        // Members
        Vector3d origin;
        Matrix3d orientation;

        // Constructors
        Frame(const Vector3d& vec, const Matrix3d& rot):
            origin(vec), orientation(rot){};
        
        Frame(const Vector3d& vec):
            origin(vec), orientation(Matrix3d::Identity()){};
        
        Frame():
            origin(Vector3d::Zero()), orientation(Matrix3d::Identity()){};

        // Member functions
        Matrix4d as_homogeneous_matrix()const{
            Matrix4d homo = Matrix4d::Constant(0.0);
            homo.block(0,0,3,3) = orientation;
            homo.block(0,3,3,1) = origin;
            homo.row(3) << 0.0, 0.0, 0.0, 1.0;
            return homo;
        };
        
        Eigen::Vector3d nautical_angles()const{
            return orientation.eulerAngles(0,1,2);
        };

        // Operators
        Frame& operator =(const Frame& other){
            origin = other.origin;
            orientation = other.orientation;
            return *this;
        };
        
        Eigen::Vector3d operator *(const Eigen::Vector3d & arg) const{
            return origin + orientation*arg;
        };
    };

    inline Frame operator *(const Frame& left, const Frame& right){
            return Frame(left.orientation*right.origin+left.origin,
                         left.orientation*right.orientation);
    };

    inline Wrench operator *(Frame frame, Wrench wrench){
        return Wrench(frame.orientation*wrench.force,
                      frame.orientation*wrench.torque + frame.origin.cross(wrench.force));
    };

    inline Vector6d operator -(const Frame& left, const Frame& right){
        Vector6d delta_frame;
        delta_frame.block<3,1>(0,0) << left.origin - right.origin;
        Matrix3d rotinvrot;
        rotinvrot << left.orientation.inverse() * right.orientation; 
        Eigen::AngleAxisd angle_axis(rotinvrot);
        delta_frame.block<3,1>(3,0) << left.orientation * angle_axis.axis() * angle_axis.angle(); 
        return delta_frame;
    };

}
