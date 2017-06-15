#include "../include/twist.hpp"
#include <Eigen/Dense>

namespace robo {

	// Constructors
	Twist::Twist(const Eigen::Vector3d& lin, const Eigen::Vector3d& rot): linear(lin), rotation(rot){}
	
	Twist::Twist(const Eigen::Vector3d& lin): linear(lin), rotation(rot){}
	
	Twist::Twist(): linear(Vector3d()), rotation(Vector3d()){}

	// Operators
	Twist& Twist::operator =(const Twist& other){
        if(this != &other){
            linear = other.linear;
            rotation = other.rotation;
        }
        return *this;
	}


	Twist rotate_twist(const Eigen::Matrix3d& rot, const Vector6d& twist){
		Vector6d res;
		res.linear = rot * twist.linear;
		res.rotation = rot * twist.rotation;
		return res;
	}

	Twist change_twist_reference(const Twist& twist, const Eigen::Vector3d& delta_ref){
		Twist res;
		res.linear = twist.linear + delta_ref.cross(twist.rotation);
		res.rotation = twist.rotation;
		return res;
	}

	// TODO remove after complete refactor
	Twist multiply_twists(const Twist& lhs, const Twist& rhs){
		Twist res;
		res.linear = lhs.rotation.cross(rhs.linear) + lhs.linear.cross(rhs.rotation);
		res.rotation = lhs.rotation.cross(rhs.rotation);
		return res;
	}

	// Twist * Twist
	Twist operator *(const Twist& lhs, const Twist& rhs){
		Twist res;
		res.linear = lhs.rotation.cross(rhs.linear) + lhs.linear.cross(rhs.rotation);
		res.rotation = lhs.rotation.cross(rhs.rotation);
		return res;
	}

	// Rotation * Twist
	Twist operator *(const Eigen::Matrix3d& rot, const Vector6d& twist){
		Twist res;
		res.linear = rot * twist.linear;
		res.rotation = rot * twist.rotation;
		return res;
	}
}
