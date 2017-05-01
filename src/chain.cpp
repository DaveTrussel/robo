#include "../include/chain.hpp"

namespace robo{

	Chain::Chain():nr_joints(0), nr_links(0){};
	
	void Chain::addLink(const Link& link){
		links.push_back(link);
		nr_links ++;
		if(link.has_joint()){
			nr_joints++;
		}
	}
}	