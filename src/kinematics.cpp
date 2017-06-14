#include "../include/kinematics.hpp"
#include <Eigen/Dense>

namespace robo{

	// Constructors
	Kinematics::Kinematics(const Chain& chain_, int max_iter_, double eps_, double eps_joints_):
    chain(chain_), f_end(), joint_roots(chain_.nr_joints), joint_tips(chain_.nr_joints), link_tips(chain.nr_links),
	jacobian(6, chain_.nr_joints), svd(6, chain_.nr_joints,Eigen::ComputeThinU | Eigen::ComputeThinV),
    max_iter(max_iter_), eps(eps_), eps_joints(eps_joints_)
	{
		weights_IK << 1, 1, 1, 0.1, 0.1, 0.1;
	}
	
	Kinematics::Kinematics(const Chain& chain_, Vector6d weigths_IK_, int max_iter_, double eps_, double eps_joints_):
    chain(chain_), f_end(), joint_roots(chain_.nr_joints), joint_tips(chain_.nr_joints), link_tips(chain.nr_links),
	jacobian(6, chain_.nr_joints), svd(6, chain_.nr_joints,Eigen::ComputeThinU | Eigen::ComputeThinV),
    max_iter(max_iter_), eps(eps_), eps_joints(eps_joints_), weights_IK(weigths_IK_){}
	
	// Member functions
	void Kinematics::joint_to_cartesian(const Eigen::VectorXd& q){
		int iter_joint = 0;
		f_end = Frame();
		for(int iter_link=0; iter_link<chain.nr_links; iter_link++){
			if(chain.links[iter_link].has_joint()){
				joint_roots[iter_joint] = f_end;
				f_end = f_end * chain.links[iter_link].pose(q[iter_joint]);
				joint_tips[iter_joint] = f_end;
				iter_joint++;
			}
			else{
				f_end = f_end * chain.links[iter_link].pose(0.0);
			}
			link_tips[iter_link] = f_end;
		}
	}

	int Kinematics::cartesian_to_joint(const Frame& f_in, const Eigen::VectorXd& q_init){
		Vector6d delta_twist;
		Vector6d delta_twist_new;
		q = q_init;
		joint_to_cartesian(q);
		delta_twist = f_end - f_in;
		delta_twist = weigths_IK.asDiagonal() * delta_twist;
		double norm_delta_twist = delta_twist.norm();
		calculate_jacobian(q);
		
		if(norm_delta_twist < eps){
			delta_twist = f_end - f_in;
			svd.compute(jacobian);
			q_out = q;
			return (error = E_NO_ERROR);
		}
		
		Eigen::MatrixXd jacobian_weighted = weigths_IK.asDiagonal() * jacobian;

		double step_multiplier = 2;
		double lambda = 10;
		double dnorm = 1;
		double norm_delta_twist_new;
		double rho;

		for(int i=0; i<max_iter; ++i){
			svd.compute(jacobian_weighted);
			singular_vals = svd.singularValues();
			for(int j=0; j<singular_vals.rows(); ++j){
				singular_vals(j) = singular_vals(j) / (singular_vals(j) * singular_vals(j) + lambda);
			}
			tmp_q = svd.matrixU().transpose() * delta_twist;
			tmp_q = singular_vals.cwiseProduct(tmp_q);
			delta_q = svd.matrixV() * tmp_q;
			grad = jacobian_weighted.transpose() * delta_twist;
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
			delta_twist_new = f_end - f_in;
			norm_delta_twist_new = delta_twist_new.norm();
			rho = norm_delta_twist * norm_delta_twist - norm_delta_twist_new * norm_delta_twist_new;
			rho /= delta_q.transpose() * (lambda * delta_q + grad);
			if (rho > 0) {
				q = q_new;
				delta_twist = delta_twist_new;
				norm_delta_twist = norm_delta_twist_new;
				if (norm_delta_twist < eps) {
					delta_twist = f_end - f_in;
					q_out = q;
					return (error = E_NO_ERROR);
				}
				calculate_jacobian(q_new);
				jacobian_weighted = weigths_IK.asDiagonal() * jacobian;
				double tmp = 2 * rho - 1;
				lambda = lambda * std::max(1 / 3.0, 1-tmp*tmp*tmp);
				step_multiplier = 2;
			} 
			else {
				lambda = lambda * step_multiplier;
				step_multiplier = 2 * step_multiplier;
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
				Vector6d unit_twist = rotate_twist(joint_roots[iter_joint].orientation, chain.links[iter_link].twist(q(iter_joint), 1.0));
				Vector6d end_twist = change_twist_reference(unit_twist, f_end.origin - joint_tips[iter_joint].origin);
				jacobian.block<6,1>(0, iter_joint) << end_twist;
				++iter_joint;
			}
		}
	}
}	
