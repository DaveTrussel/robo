#include "../include/forward_kinematics.hpp"
#include <Eigen/Dense>

namespace robo{

	class ForwardKinematics{
	public:
		Chain chain;

		ForwardKinematics(const Chain& chain){
			this->chain = chain;
		}
		
		void joint2cartesian(const Eigen::VectorXd& q; std::vector<Frame> f_out){
			f_last = Frame

			if(chain.links[0].has_joint()){
				f_out[0] = chain.links[0].pose(q(0))
			}

			
		}
		
	}

}	
