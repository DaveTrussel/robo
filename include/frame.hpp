#pragma once

#include "twist.hpp"
#include "wrench.hpp"
#include <Eigen/Dense>

namespace robo {

	using Vector6d = Eigen::Matrix<double, 6, 1>;
	using Matrix6Xd = Eigen::Matrix<double, 6, Eigen::Dynamic>;
	using Matrix6d = Eigen::Matrix<double, 6, 6>;

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

	inline Frame operator *(const Frame& left, const Frame& right){
			return Frame(left.orientation*right.origin+left.origin,
						 left.orientation*right.orientation);
	};

	inline Wrench operator *(Frame frame, Wrench wrench){
		return Wrench(frame.orientation*wrench.force,
					  frame.orientation*wrench.torque + frame.origin.cross(wrench.force));
	};

	Vector6d operator -(const Frame& left, const Frame& right);
}
