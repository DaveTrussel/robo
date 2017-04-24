#pragma once

namespace robo{
	class Link{
	public:

		int id;
		Joint joint;
		Frame tip;

		Link(int id_in, const Joint& joint_in, const Frame& tip_in):
			id(id_in), joint(joint_in), tip(tip_in){}

		Frame pose(const double& q)const{
			//TODO
		}
		
		Eigen::Vector6d twist(const double& q, const double &dq)const{
			//TODO
		}
		
		bool has_joint(){
			return joint.type != JointType::None;
		}
	}

}	

		
