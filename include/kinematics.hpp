#pragma once

#include "chain.hpp"
#include <Eigen/Dense>
#include <vector>

namespace robo{

	class Kinematics{
	public:
		Chain chain;
		Frame f_end;
		VectorXd q;
		std::vector<Frame> link_tips;
		std::vector<Frame> joint_roots;
		std::vector<Frame> joint_tips;
		Eigen::MatrixXd jacobian;
		Eigen::JacobiSVD<MatrixXd> svd;
		Vector6d L;
		int max_iter;
		double eps;
    	double eps_joints;

    	static const int E_NO_ERROR = 					 1;
    	static const int E_JOINTS_GRADIENT_TOO_SMALL =  -1;
    	static const int E_JOINTS_INCREMENT_TOO_SMALL = -2;
    	static const int E_MAX_ITERATIONS = 			-3;
		

		Kinematics(const Chain& chain,
				   int max_iter=500,
				   double eps=1e-5,
				   double eps_joints=1e-10);

		Kinematics(const Chain& chain, Vector6d L,
				   int max_iter=500,
				   double eps=1e-5,
				   double eps_joints=1e-10);
		
		void joint_to_cartesian(const Eigen::VectorXd& q);

		void cartesian_to_joint(const Frame& f_in, Eigen::VectorXd& q);

		void calculate_jacobian(const Eigen::VectorXd& q);
		
	private:
		VectorXd q_new;
		VectorXq tmp;
		VectorXq delta_q;
		VectorXq grad;
		VectorXq singular_vals; // TODO rename
	};

}	
