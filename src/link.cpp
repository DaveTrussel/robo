#include "../include/link.hpp"

namespace robo {

	// Constructors
	Link::Link(int id_in, const Joint& joint_in, const Frame& tip_in):
		id(id_in), joint(joint_in), tip(tip_in){}

	// Member functions
	Frame Link::pose(const double& q)const{
		return joint.pose(q)*tip;
	}
	
	Twist Link::twist(const double& q, const double &dq)const{
		Eigen::Vector3d ref_point = joint.pose(q).orientation * tip.origin;
		return change_twist_reference(joint.twist(dq), ref_point);
	}
	
	bool Link::has_joint() const{
		return joint.type != JointType::None;
	}

}	
