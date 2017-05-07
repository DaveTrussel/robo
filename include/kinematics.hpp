#pragma once

#include "chain.hpp"
#include <Eigen/Dense>
#include <vector>

namespace robo{

	class Kinematics{
	public:
		Chain chain;
		Frame f_end;
		Eigen::VectorXd q;
		Eigen::VectorXd q_out;
		std::vector<Frame> link_tips;
		std::vector<Frame> joint_roots;
		std::vector<Frame> joint_tips;
		Eigen::MatrixXd jacobian;
		Eigen::JacobiSVD<Eigen::MatrixXd> svd;
		Vector6d L;
		int max_iter;
		double eps;
    	double eps_joints;
    	int error;

    	static const int E_NO_ERROR = 					 1;
    	static const int E_JOINTS_GRADIENT_TOO_SMALL =  -1;
    	static const int E_JOINTS_INCREMENT_TOO_SMALL = -2;
    	static const int E_MAX_ITERATIONS = 			-3;
		

		Kinematics(const Chain& chain,
				   int max_iter=500,
                   double eps=1e-4,
                   double eps_joints=1e-15);

		Kinematics(const Chain& chain, Vector6d L,
				   int max_iter=500,
                   double eps=1e-4,
                   double eps_joints=1e-15);
		
		void joint_to_cartesian(const Eigen::VectorXd& q);

		int cartesian_to_joint(const Frame& f_in, const Eigen::VectorXd& q_init);

		void calculate_jacobian(const Eigen::VectorXd& q);
		
	private:
		Eigen::VectorXd q_new;
		Eigen::VectorXd tmp;
		Eigen::VectorXd delta_q;
		Eigen::VectorXd grad;
		Eigen::VectorXd singular_vals; // TODO rename
	};

}	
