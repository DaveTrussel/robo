#pragma once

#include "joint.hpp"
#include <Eigen/Dense>

namespace robo{
	class Link{
	public:

		int id;
		Joint joint;
		Frame tip;

		Link(int id, const Joint& joint, const Frame& tip);

		Frame pose(const double& q)const;
		Eigen::Vector6d twist(const double& q, const double &dq)const;
		bool has_joint();
	}

}	

		
