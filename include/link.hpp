#pragma once

#include "joint.hpp"
#include <Eigen/Dense>

namespace robo{

	class Link{
	public:
		// Members
		int id;
		Joint joint;
		Frame tip;

		// Constructors
		Link(int id, const Joint& joint, const Frame& tip);

		// Member functions
		Frame pose(const double& q)const;
		
		Twist twist(const double& q, const double &dq)const;
		
		bool has_joint()const;
	};

}	
