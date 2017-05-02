#include "../include/forward_kinematics.hpp"
#include <Eigen/Dense>

namespace robo{

	InverseKinematics::InverseKinematics(const Chain& chain_, const ForwardKinematics& fk_,
						                 unsigned int max_iter_=100, double& eps_=1e-6):
	chain(chain_), fk(fk_), max_iter(max_iter_), eps(eps_) {}

	
	void ForwardKinematics::cartesian2joint(const Frame f_in, const Eigen::VectorXd& q_start, 
		                                    Eigen::VectorXd& q_out){
		
	}
}	
