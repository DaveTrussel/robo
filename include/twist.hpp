#pragma once

#include <Eigen/Dense>

namespace robo {

	class Twist{
	public:

		// Members
		Eigen::Vector3d linear;
		Eigen::Vector3d rotation;

		// Constructors
		explicit Twist(const Eigen::Vector3d& lin, const Eigen::Vector3d& rot): linear(lin), rotation(rot){};
		
		explicit Twist(const Eigen::Vector3d& lin): linear(lin), rotation(Eigen::Vector3d()){};
		
		Twist(): linear(Eigen::Vector3d(0.0, 0.0, 0.0)), rotation(Eigen::Vector3d()){};

		// Operators
		Twist& operator =(const Twist& other){
			linear = other.linear;
        	rotation = other.rotation;
        	return *this;
		};

	};


	inline Twist rotate_twist(const Eigen::Matrix3d& rot, const Twist& twist){
		return Twist(rot * twist.linear, rot * twist.rotation);
	};

	inline Twist change_twist_reference(const Twist& twist, const Eigen::Vector3d& delta_ref){
		return Twist(twist.linear + delta_ref.cross(twist.rotation), twist.rotation);
	};

	inline Twist multiply_twists(const Twist& lhs, const Twist& rhs){
		return Twist(lhs.rotation.cross(rhs.linear) + lhs.linear.cross(rhs.rotation), lhs.rotation.cross(rhs.rotation));
	};

	inline Twist operator +(const Twist& lhs, const Twist& rhs){
		return Twist(lhs.linear+rhs.linear, lhs.rotation+rhs.rotation);
	};

	inline Twist operator *(const Twist& lhs, const Twist& rhs){
		return Twist(lhs.rotation.cross(rhs.linear) + lhs.linear.cross(rhs.rotation),
			         lhs.rotation.cross(rhs.rotation));
	};

	inline Twist operator *(const Eigen::Matrix3d& rot, const Twist& twist){
		return Twist(rot * twist.linear, rot * twist.rotation);
	};

	inline Twist operator *(const double& val, const Twist& twist){
		return Twist(twist.linear*val, twist.rotation*val);
	};

	inline Twist operator *(const Twist& twist, const double& val){
		return Twist(twist.linear*val, twist.rotation*val);
	};

}