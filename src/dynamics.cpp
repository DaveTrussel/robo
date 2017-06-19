#include "../include/dynamics.hpp"
#include <Eigen/Dense>

namespace robo{

	// Constructors
	Dynamics::Dynamics(const Chain& chain_):chain(chain_){}

	int Dynamics::calculate_torques(const Eigen::VectorXd& q_, const Eigen::VectorXd& dq_,
							        const Eigen::VectorXd& ddq_, const Eigen::Vector3d& gravity_,
							        const std::vector<Wrench>& wrenches_extern){
		double q, dq, ddq;
		Twist link_twist;
		Eigen::Matrix3d to_link_rot;
		Inertia inertia;

		Twist gravity{gravity_, Eigen::Vector3d(0.0, 0.0, 0.0)};

		// From root to tip
		int iter_joint = 0;
		for(int iter_link=0; iter_link<chain.nr_links; ++iter_link){
			if(chain.links[iter_link].has_joint()){
				q = q_[iter_joint];
				dq = dq_[iter_joint];
				ddq = ddq_[iter_joint];
				++iter_joint;
			}
			else{
				q = dq = ddq = 0.0;
			}

			// calculate frame of current link
			link_frames[iter_link] = chain.links[iter_link].pose(q);

			// calculate twist and unit twist in current link frame
			to_link_rot = link_frames[iter_link].orientation.inverse();
			link_twist = rotate_twist(to_link_rot, chain.links[iter_link].twist(q, dq));
			unit_twists[iter_link] = rotate_twist(to_link_rot, chain.links[iter_link].twist(q, 1.0));

			// calculate velocity anc acceleration in current link frame
			if(iter_link==0){
				velocities[iter_link] = link_twist;
				accelerations[iter_link] = rotate_twist(to_link_rot, gravity) +
										   unit_twists[iter_link] * ddq +
										   multiply_twists(velocities[iter_link], link_twist);
			}
			else{
				velocities[iter_link] = rotate_twist(to_link_rot, velocities[iter_link-1]) + link_twist;
				accelerations[iter_link] = rotate_twist(to_link_rot, accelerations[iter_link-1]) +
										   accelerations[iter_link-1] * ddq +
										   multiply_twists(velocities[iter_link], link_twist);
			}

			// calculate wrench acting on current link
			inertia = chain.links[iter_link].inertia_matrix;
			wrenches[iter_link] = inertia * accelerations[iter_link] +
								  velocities[iter_link] * (inertia * velocities[iter_link]) -
								  wrenches_extern[iter_link];
			// TODO define inertia_matrix * twist
		}

		// Back from tip to root
		iter_joint = chain.nr_joints;
		for(int iter_link=chain.nr_links-1; iter_link>=0; --iter_link){
			if(chain.links[iter_link].has_joint()){
				// calculate torque (unit twist * wrench)
				joint_torques[--iter_joint] = wrenches[iter_link].dot(unit_twists[iter_link]);
			}
			if(iter_link>0){
				// add wrench of this link to lower link's wrench (transform into lower frame)
				wrenches[iter_link-1] = wrenches[iter_link-1] + link_frames[iter_link]*wrenches[iter_link];
			}
		}
		return 1;
	}

}	
