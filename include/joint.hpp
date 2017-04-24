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

		static Joint createRotX(const int id, Eigen::Vector3d origin);
		static Joint createRotY(const int id, Eigen::Vector3d origin);
		static Joint createRotZ(const int id, Eigen::Vector3d origin);

		Frame pose(const double& q)const;

		Eigen::Vector6d twist(const double& q, const double &dq)const;

		Eigen::Matrix3d rotation(const double& q)const;

	}

}
