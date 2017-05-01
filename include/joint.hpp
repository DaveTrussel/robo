#pragma once

#include "frame.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace robo {

	typedef Matrix< double, 6, 1 > 	Eigen::Vector6d;

	enum class JointType { None, Rotational, Translational };

	class Joint{
	public:
		
		// Members
		int id;
		JointType type;
		Eigen::Vector3d axis;
		Frame frame;

		// Constructors
		Joint(const int id_in, const Frame frame_in, const Eigen::Vector3d axis_in, 
			const JointType type_in=JointType::Rotational);
		Joint(const int id_in, const Eigen::Vector3d origin_in, const Eigen::Vector3d axis_in, 
			const JointType type_in=JointType::Rotational);

		// Member functions
		Frame pose(const double& q)const;

		Eigen::Vector6d twist(const double& q, const double &dq)const;

		Eigen::Matrix3d rotation(const double& q)const;

	};

}
