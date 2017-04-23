#pragma once

#include "core.hpp"

namespace robo {

	typedef enum { RotX,RotY,RotZ } JointType;

	class Joint{
	public:
		
		int id;
		Eigen::Vector3d axis;
		Eigen::Vector3d origin;

		Joint(const int id, const JointType& type=None);

		Frame pose(const double& q)const;
		Twist twist(const double& q, const double &dq)const;


	}

}
