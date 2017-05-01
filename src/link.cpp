#include "../include/link.hpp"
#include "../include/frame.hpp"

namespace robo {

	Link::Link(int id_in, const Joint& joint_in, const Frame& tip_in):
		id(id_in), joint(joint_in), tip(tip_in){}

	Frame Link::pose(const double& q)const{
		return joint.pose(q)*tip;
	}
	
	Vector6d Link::twist(const double& q, const double &dq)const{
		return Vector6d(); // TODO
	}
	
	bool Link::has_joint(){
		return joint.type != JointType::None;
	}

}	
