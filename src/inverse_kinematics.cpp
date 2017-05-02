#include "../include/forward_kinematics.hpp"
#include <Eigen/Dense>

namespace robo{

	InverseKinematics::InverseKinematics(const Chain& chain_, const ForwardKinematics& fk_,
						                 unsigned int max_iter_=100, double& eps_=1e-6):
	chain(chain_), nr_joints(chain_.nr_joints), fk(fk_), max_iter(max_iter_), eps(eps_) {}

	
	void ForwardKinematics::cartesian2joint(const Frame f_in, const Eigen::VectorXd& q_start, 
		                                    Eigen::VectorXd& q_out){
		q_out = q_start;
		for(int iter=0; iter<max_iter; iter++){
			fk.joint2cartesian(q_out, target);
			// TODO clculate delta_twist = f - p
			// if delta_twist <= eps --> break
			// inverse_velocity solver --> delta_q
			// q_out += delta_q
			// check for joint limits and if exceeded set to joint limits
		}
	}
}	
