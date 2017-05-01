#pragma once

#include "chain.hpp"
#include <Eigen/Dense>
#include <vector>

namespace robo{

	class ForwardKinematics{
	public:
		Chain chain;

		ForwardKinematics(const Chain& chain);
		
		void joint2cartesian(const Eigen::VectorXd& q; std::vector<Frame> f_out);
		
	};

}	
