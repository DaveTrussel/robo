#include "../include/forward_kinematics.hpp"
#include <Eigen/Dense>

namespace robo{

	ForwardKinematics::ForwardKinematics(const Chain& chain){
		this->chain = chain;
	}
	
	void ForwardKinematics::joint2cartesian(const Eigen::VectorXd& q, std::vector<Frame>& f_out){
		// init
		int iter_joint = 0;
		if(chain.links[0].has_joint()){
			f_out[0] = chain.links[0].pose(q[iter_joint]); 
			iter_joint++;
		}
		else{
			f_out[0] = chain.links[0].pose(0.0);
		}
		// loop over all links
		for(int iter_link=1; iter_link<chain.nr_links; iter_link++){
			if(chain.links[iter_link].has_joint()){
				f_out[iter_link] = f_out[iter_link-1]*chain.links[iter_link].pose(q[iter_joint]);
				iter_joint++;
			}
			else{
				f_out[iter_link] = f_out[iter_link-1]*chain.links[iter_link].pose(0.0);
			}
		}
	}
}	
