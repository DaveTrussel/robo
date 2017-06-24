#pragma once

#include "link.hpp"
#include <vector>

namespace robo{

	class Chain{
	public:
		int nr_joints;
		int nr_links;
		std::vector<Link> links;

		Chain():nr_joints(0), nr_links(0){};
		
		void addLink(const Link& link){
			links.push_back(link);
			++nr_links;
			if(link.has_joint()){
				++nr_joints;
			}
		};
	};

}	