#pragma once

#include <Eigen/Dense>

namespace robo {

	class Wrench{
	public:

		// Members
		Eigen::Vector3d force;
		Eigen::Vector3d torque;

		// Constructors
		explicit Wrench(const Eigen::Vector3d& force, const Eigen::Vector3d& torque);
		
		explicit Wrench(const Eigen::Vector3d& force);
		
		Wrench();

		// Operators
		Wrench& operator =(const Wrench& other);

	};

	Wrench operator -(const Wrench& lhs, const Wrench& rhs);

	Wrench operator *(const Eigen::Matrix3d& rot, const Wrench& wrench);

	Wrench operator *(const double& val, const Wrench& wrench);

	Wrench operator *(const Wrench& wrench, const double& val);

}