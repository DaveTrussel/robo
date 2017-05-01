#include "../include/chain.hpp"

namespace robo{

	Chain::Chain();
	
	void Chain::addLink(const Link& link){
		links.push_back(link);
		nr_links ++;
		if(link.has_joint()){
			nr_joints++;
		}
	}
}	