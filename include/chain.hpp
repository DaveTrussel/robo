#pragma once

#include "link.hpp"
#include <Eigen/Dense>
#include <vector>

namespace robo{

	class Chain{
	public:
		int nr_joints;
		int nr_links;
		std::vector<Link> links;

		Chain();
		
		void addLink(const Link& link);
	};

}	