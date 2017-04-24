#pragma once

#include "link.h"

#include <vector>

namespace robo{

	class Chain{
	public:
		unsigned int nr_Joints;
		unsigned int nr_Links;
		
		std::vector<Link> links;

		Chain();
		
		void addLink(const Link& link);

		void setJointPositions(const Eigen::VectorXd& q)
	}

}	