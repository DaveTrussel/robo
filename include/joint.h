#pragma once

#include "core.hpp"

namespace robo {

	typedef enum { None,RotX,RotY,RotZ } JointType;

	class Joint{
	public:
		
		int id;
		JointType type;
		Eigen::Vector3d axis;
		Eigen::Vector3d origin;

		Joint(const int id, const JointType& type=None);

		Frame pose(const double& q)const;

		Eigen::Vector6d twist(const double& q, const double &dq)const;

		Eigen::Matrix3d rotation(const double& q)const;

	}

}
