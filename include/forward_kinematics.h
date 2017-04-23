#pragma once

#include "chain.h"

#include <vector>

namespace robo{

	class ForwardKinematics{
	public:
		Chain chain;

		ForwardKinematics(const Chain& chain);
		
		void joint2cartesian(std::vector<Frame> out)
		
	}

}	