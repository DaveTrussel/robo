#pragma once

#include "chain.hpp"
#include <Eigen/Dense>
#include <vector>

namespace robo{

	class Kinematics{
	public:
		Chain chain;
		Frame f_end;
		std::vector<Frame> joint_roots;
		std::vector<Frame> joint_tips;
		Eigen::MatrixXd jacobian;
		Eigen::JacobiSVD<MatrixXd> svd;

		Kinematics(const Chain& chain);
		
		void joint_to_cartesian(const Eigen::VectorXd& q, std::vector<Frame>& f_out);

		void calculate_jacobian(const Eigen::VectorXd& q);
		
	};

}	
