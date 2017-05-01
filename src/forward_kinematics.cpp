#include "../include/forward_kinematics.hpp"
#include <Eigen/Dense>

namespace robo{

	ForwardKinematics::ForwardKinematics(const Chain& chain){
		this->chain = chain;
	}
	
	void ForwardKinematics::joint2cartesian(const Eigen::VectorXd& q, std::vector<Frame>& f_out){
		f_out[0] = chain.links[0].pose(q[0]); // init
		for(int i=1; i<chain.nr_links; i++){
			f_out[i] = f_out[i-1]*chain.links[i].pose(q[i]);
		}
	}
}	
