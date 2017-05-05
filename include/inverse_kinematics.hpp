#pragma once

#include "chain.hpp"
#include <Eigen/Dense>
#include <vector>

namespace robo{

	class InverseKinematics{
	public:
		Chain chain;
		unsigned int nr_joints;
		Eigen::VectorXd& q_min;
		Eigen::VectorXd& q_max;
		Eigen::VectorXd& dq;
		unsigned int max_iter;
		double eps;
		Frame frame;


		InverseKinematics(const Chain& chain_, const ForwardKinematics& fk_,
						  unsigned int max_iter_=100, double& eps_=1e-6);
		
		void cartesian_to_joint(const Frame f_in, const Eigen::VectorXd& q_start, 
		                     Eigen::VectorXd& q_out);
	};
}	
