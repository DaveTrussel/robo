#pragma once

#include <Eigen/Dense>

namespace robo {

	class Twist{
	public:

		// Members
		Eigen::Vector3d linear;
		Eigen::Vector3d rotation;

		// Constructors
		Twist(const Eigen::Vector3d& lin, const Eigen::Vector3d& rot);
		
		Twist(const Eigen::Vector3d& lin);
		
		Twist();

		// Operators
		Twist& operator =(const Twist& other);

	};


	Twist rotate_twist(const Eigen::Matrix3d& rot, const Twist& twist);

	Twist change_twist_reference(const Twist& twist, const Eigen::Vector3d& delta_ref);

	Twist multiply_twists(const Twist& lhs, const Twist& rhs);

	Twist operator +(const Twist& lhs, const Twist& rhs);

	Twist operator *(const Twist& lhs, const Twist& rhs);

	Twist operator *(const Eigen::Matrix3d& rot, const Twist& twist);

	Twist operator *(const double& val, const Twist& twist);

	Twist operator *(const Twist& twist, const double& val);

}