#pragma once

#include "twist.hpp"
#include "wrench.hpp"
#include <Eigen/Dense>

namespace robo {

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

        Frame inverse()const{
            Matrix3d reverse = orientation.transpose().eval();
            return Frame(reverse*origin, reverse);
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

    inline Twist operator *(Frame frame, Twist twist){
        return Twist(frame.orientation*twist.linear + frame.origin.cross(twist.rotation),
                     frame.orientation*twist.rotation);
    };

    inline Wrench operator *(Frame frame, Wrench wrench){
        return Wrench(frame.orientation*wrench.force,
                      frame.orientation*wrench.torque + frame.origin.cross(wrench.force));
    };

    inline Vector6d operator -(const Frame& left, const Frame& right){
        Vector6d delta_frame;
        delta_frame.block<3,1>(0,0) << left.origin - right.origin;
        Matrix3d rot_inv_rot;
        rot_inv_rot << left.orientation * right.orientation.transpose(); // transpose == inverse, for rotation matrices
        Eigen::AngleAxisd angle_axis(rot_inv_rot);
        delta_frame.block<3,1>(3,0) << angle_axis.axis() * angle_axis.angle();
        return delta_frame;
    };

}
