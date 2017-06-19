#include "../include/joint.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace robo {

	// Constructors
	Joint::Joint(const int id_in, const Frame frame_in, const Eigen::Vector3d axis_in, 
		const JointType type_in):
	id(id_in), type(type_in), axis(axis_in), frame(frame_in){}

	// Member functions
	Frame Joint::pose(const double& q)const{
		if(type == JointType::Rotational){
			Eigen::Matrix3d rotation = (Eigen::Matrix3d)Eigen::AngleAxisd(q, axis);
			return Frame(frame.origin, rotation);
		}
		if(type == JointType::Translational){ 
			return Frame(frame.origin + q*axis, frame.orientation);
		}
		else{
			return Frame(frame);
		}
	}s

	Twist Joint::twist(const double &dq)const{
		Eigen::Vector3d speed_lin;
		Eigen::Vector3d speed_rot;
		speed_lin << 0.0, 0.0, 0.0;
		speed_rot << 0.0, 0.0, 0.0;
		if(type == JointType::Rotational){
			speed_rot << dq*axis;
		}
		if(type == JointType::Translational){
			speed_lin << dq*axis;
		}
		return Twist(speed_lin, speed_rot);
	}
}
