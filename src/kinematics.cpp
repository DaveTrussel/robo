#include "../include/kinematics.hpp"
#include <Eigen/Dense>
#include <iostream>

namespace robo{

	// Constructors
	Kinematics::Kinematics(const Chain& chain_, int max_iter_, double eps_, double eps_joints_):
    chain(chain_), f_end(), joint_roots(chain_.nr_joints), joint_tips(chain_.nr_joints), link_tips(chain.nr_links),
	jacobian(6, chain_.nr_joints), svd(6, chain_.nr_joints,Eigen::ComputeThinU | Eigen::ComputeThinV),
    max_iter(max_iter_), eps(eps_), eps_joints(eps_joints_)
	{
		L << 1, 1, 1, 0.1, 0.1, 0.1;
	}

	Kinematics::Kinematics(const Chain& chain_, Vector6d L_, int max_iter_, double eps_, double eps_joints_):
    chain(chain_), f_end(), joint_roots(chain_.nr_joints), joint_tips(chain_.nr_joints), link_tips(chain.nr_links),
	jacobian(6, chain_.nr_joints), svd(6, chain_.nr_joints,Eigen::ComputeThinU | Eigen::ComputeThinV),
    max_iter(max_iter_), eps(eps_), eps_joints(eps_joints_), L(L_){}
	
	// Member functions
	void Kinematics::joint_to_cartesian(const Eigen::VectorXd& q){
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
			link_tips[iter_link] = f_end;
		}
	}

	int Kinematics::cartesian_to_joint(const Frame& f_in, const Eigen::VectorXd& q_init){
		Vector6d delta_frame;
		Vector6d delta_frame_new;
		q = q_init;
		joint_to_cartesian(q);
		delta_frame = f_end - f_in;
		delta_frame = L.asDiagonal() * delta_frame;
		double norm_delta_frame = delta_frame.norm();
		if(norm_delta_frame < eps){
			delta_frame = f_end - f_in;
			svd.compute(jacobian);
			q_out = q;
			return (error = E_NO_ERROR);
		}
		calculate_jacobian(q);
		jacobian = L.asDiagonal() * jacobian;

		double v = 2;
		double lambda = 10;
		double dnorm = 1;
		double norm_delta_frame_new;
		double rho;

		for(int i=0; i<max_iter; i++){
			svd.compute(jacobian);
			singular_vals = svd.singularValues();
			for(int j=0; j<singular_vals.rows(); ++j){
				singular_vals(j) = singular_vals(j) / (singular_vals(j) * singular_vals(j) + lambda);
			}
			tmp = svd.matrixU().transpose() * delta_frame;
			tmp = singular_vals.cwiseProduct(tmp);
			delta_q = svd.matrixV() * tmp;
			grad = jacobian.transpose() * delta_frame;
			dnorm = delta_q.lpNorm<Eigen::Infinity>();
			// check for errors
			if(dnorm < eps_joints){
				return (error = E_JOINTS_INCREMENT_TOO_SMALL);
			}
			if(grad.transpose() * grad < eps_joints * eps_joints){
				return (error = E_JOINTS_GRADIENT_TOO_SMALL);
			}

			q_new = q + delta_q;
            joint_to_cartesian(q_new);
			delta_frame_new = f_end - f_in;
			norm_delta_frame_new = delta_frame_new.norm();
			rho = norm_delta_frame * norm_delta_frame - norm_delta_frame_new * norm_delta_frame_new;
			rho /= delta_q.transpose() * (lambda * delta_q + grad);
			if (rho > 0) {
				q = q_new;
				delta_frame = delta_frame_new;
				norm_delta_frame = norm_delta_frame_new;
				if (norm_delta_frame < eps) {
					delta_frame = f_end - f_in;
					q_out = q;
					return (error = E_NO_ERROR);
				}
				calculate_jacobian(q_new);
				jacobian = L.asDiagonal() * jacobian;
				double tmp = 2 * rho - 1;
				lambda = lambda * std::max(1 / 3.0, 1-tmp*tmp*tmp);
				v = 2;
			} 
			else {
				lambda = lambda * v;
				v      = 2 * v;
			}
			q_out = q;
		}
        q_out = q;
        return (error = E_MAX_ITERATIONS);
	}

	void Kinematics::calculate_jacobian(const Eigen::VectorXd& q){
		int iter_joint = 0;
		for(int iter_link=0; iter_link<chain.nr_links; ++iter_link){
			if (chain.links[iter_link].has_joint()) {
				// compute twist of the end effector motion caused by joint [jointndx]; expressed in base frame, with vel. ref. point equal to the end effector
				Twist unit_twist = rotate_twist(joint_roots[iter_joint].orientation, chain.links[iter_link].twist(q(iter_joint), 1.0));
				Twist end_twist = change_twist_reference(unit_twist, f_end.origin - joint_tips[iter_joint].origin);
				jacobian.block<3,1>(0, iter_joint) << end_twist.linear;
				jacobian.block<3,1>(3, iter_joint) << end_twist.rotation;
				++iter_joint;
			}
		}
	}
}	
