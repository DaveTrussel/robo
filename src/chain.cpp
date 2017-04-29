#pragma once

#include "../include/chain.hpp"

namespace robo{

	class Chain{
	public:
		unsigned int nr_joints;
		unsigned int nr_links;
		
		std::vector<Link> links;

		Chain();
		
		void addLink(const Link& link){
			links.push_back(link);
			nr_links ++;
			if(link.has_joint()){
				nr_joints++;
			}
		}
	}

}	