#include "../include/forward_kinematics.hpp"
#include <Eigen/Dense>

namespace robo{

	ForwardKinematics::ForwardKinematics(const Chain& chain_):
	chain(chain_), f_end(), joint_roots(chain_.nr_joints), joint_tips(chain_.nr_joints),
	jacobian(6, chain_.nr_joints), svd(6, nj,Eigen::ComputeThinU | Eigen::ComputeThinV)
	{}
	
	void ForwardKinematics::joint_to_cartesian(const Eigen::VectorXd& q, std::vector<Frame>& f_out){
		// init
		int iter_joint = 0;
		f_end = Frame();
		for(int iter_link=0; iter_link<chain.nr_links; iter_link++){
			if(chain.links[iter_link].has_joint()){
				joint_roots[iter_joint] = f_end;
				f_end = f_end*chain.links[iter_link].pose(q[iter_joint]);
				joint_tips[iter_joint] = f_end;
				iter_joint++;
			}
			else{
				f_end = f_end*chain.links[iter_link].pose(0.0);
			}
			f_out[iter_link] = f_end;
		}
	}

	void ForwardKinematics::calculate_jacobian(const Eigen::VectorXd& q){
		int iter_joint = 0;
		for(int iter_link=0; iter_link<chain.nr_links; iter_link++){
			if (chain.links[iter_link].has_joint()) {
				// compute twist of the end effector motion caused by joint [jointndx]; expressed in base frame, with vel. ref. point equal to the end effector
				Vector6d unit_twist = joint_roots[iter_joint].orientation * link.twist(q(iter_joint), 1.0);
				Vector6d end_twist = change_twist_reference(unit_twist, f_end - joint_tips[iter_joint].origin);
				jacobian.block<6,1>(0, iter_joint) << end_twist;
				iter_joint++;
			}
		}
	}
}	
