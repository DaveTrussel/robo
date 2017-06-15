#pragma once

#include "../include/twist.hpp"
#include <Eigen/Dense>

namespace robo {

	typedef Eigen::Matrix< double, 6, 1 > Vector6d;

	class Frame{
	public:

		// Members
		Eigen::Vector3d origin;
		Eigen::Matrix3d orientation;

		// Constructors
		Frame(const Eigen::Vector3d& vec, const Eigen::Matrix3d& rot);
		
		Frame(const Eigen::Vector3d& vec);
		
		Frame();

		// Named constructors
		static Frame DenavitHartenberg(double a, double alpha, double d, double theta);
		
		static Frame DenavitHartenberg_Craig1989(double a, double alpha, double d, double theta);

		// Member functions
		Eigen::Matrix4d as_homogeneous_matrix()const;
		
		Eigen::Vector3d nautical_angles()const;

		// Operators
		Frame& operator =(const Frame& other);
		
		Eigen::Vector3d operator *(const Eigen::Vector3d & arg) const;
	};

	Frame operator *(const Frame& left, const Frame& rigth);

	Twist operator -(const Frame& left, const Frame& right);

	Vector6d operator -(const Frame& left, const Frame& right);

	Vector6d rotate_twist(const Eigen::Matrix3d& rot, const Vector6d& twist);

	Vector6d change_twist_reference(const Vector6d& twist, const Eigen::Vector3d& delta_ref);

	Vector6d multiply_twists(const Vector6d& rhs, const Vector6d& lhs);
}
