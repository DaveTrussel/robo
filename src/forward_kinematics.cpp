#pragma once

#include "forward_kinematics.h"

namespace robo{

	class ForwardKinematics{
	public:
		Chain chain;

		ForwardKinematics(const Chain& chain){
			this->chain = chain;
		}
		
		void joint2cartesian(const Eigen::VectorXd& q; std::vector<Frame> f_out){

		}
		
	}

}	
