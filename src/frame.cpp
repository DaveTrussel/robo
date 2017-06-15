#include "../include/frame.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace robo {

	// Constructors
	Frame::Frame(const Eigen::Vector3d& vec, const Eigen::Matrix3d& rot):
		origin(vec), orientation(rot){}

	Frame::Frame(const Eigen::Vector3d& vec):
		origin(vec), orientation(Eigen::Matrix3d::Identity()){}

	Frame::Frame():
		origin(Eigen::Vector3d::Zero()), orientation(Eigen::Matrix3d::Identity()){}

	// Named constructors
	Frame Frame::DenavitHartenberg(double a, double alpha, double d, double theta){
		return Frame(); // TODO
	}

	Frame Frame::DenavitHartenberg_Craig1989(double a, double alpha, double d, double theta){
		return Frame(); // TODO
	}

	// Member functions
	Eigen::Matrix4d Frame::as_homogeneous_matrix()const{
		Eigen::Matrix4d homo = Eigen::Matrix4d::Constant(0.0);
		homo.block(0,0,3,3) = orientation;
		homo.block(0,3,3,1) = origin;
		homo.row(3) << 0.0, 0.0, 0.0, 1.0;
		return homo;
	}

	Eigen::Vector3d Frame::nautical_angles()const{
		return orientation.eulerAngles(0,1,2);
	}

	// Operators
    Frame& Frame::operator =(const Frame& other){
        if(this != &other){
            origin = other.origin;
            orientation = other.orientation;
        }
        return *this;
    }

	Eigen::Vector3d Frame::operator *(const Eigen::Vector3d & arg) const{
	    return origin + orientation*arg;
	}

	// Frame * Frame
	Frame operator *(const Frame& left, const Frame& right){
			return Frame(left.orientation*right.origin+left.origin,
						 left.orientation*right.orientation);
	}

	// Frame - Frame
	Vector6d operator -(const Frame& left, const Frame& right){
		Vector6d delta_twist;
		delta_twist.block<3,1>(0,0) << left.origin - right.origin;
		Eigen::Matrix3d rotinvrot;
		rotinvrot << left.orientation.inverse() * right.orientation; 
		Eigen::AngleAxisd angle_axis(rotinvrot);
		Eigen::Vector3d angular;
		delta_twist.block<3,1>(3,0) << left.orientation * angle_axis.axis() * angle_axis.angle(); // TODO check if angle_axis.axis() is normalized or not
		return delta_twist;
	}

	////////////////////////////////////////////////////////////////
	// TODO this got out of hand, create own Twist and Wrench class
	////////////////////////////////////////////////////////////////
	Vector6d rotate_twist(const Eigen::Matrix3d& rot, const Vector6d& twist){
		Vector6d twist_new;
		twist_new.block<3,1>(0,0) << rot * Eigen::Vector3d(twist.block<3,1>(0,0));
		twist_new.block<3,1>(3,0) << rot * Eigen::Vector3d(twist.block<3,1>(3,0));
		return twist_new;

	}

	Vector6d change_twist_reference(const Vector6d& twist, const Eigen::Vector3d& delta_ref){
		Vector6d twist_new;
		twist_new.block<3,1>(0,0) << twist.block<3,1>(0,0) + delta_ref.cross(twist.block<3,1>(3,0));
		twist_new.block<3,1>(3,0) << twist.block<3,1>(3,0);
		return twist_new;
	}

	Vector6d multiply_twists(const Vector6d& rhs, const Vector6d& lhs){
		Vector6d twist_new;
		twist_new.block<3,1>(0,0) << lhs.block<3,1>(3,0).cross(rhs.block<3,1>(0,0)) + lhs.block<3,1>(0,0).cross(rhs.block<3,1>(3,0));
		twist_new.block<3,1>(3,0) << lhs.block<3,1>(3,0).cross(rhs.block<3,1>(3,0));
		return twist_new;
	}
}
