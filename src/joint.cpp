#include "../include/joint.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace robo {

	// Constructors
	Joint::Joint(const int id_in, const Frame frame_in, const Eigen::Vector3d axis_in, 
		const JointType type_in):
	id(id_in), frame(frame_in), axis(axis_in), type(type_in){}

	Joint::Joint(const int id_in, const Eigen::Vector3d origin_in, const Eigen::Vector3d axis_in, 
		const JointType type_in):
	id(id_in), frame(origin_in), axis(axis_in), type(type_in){}

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
	}

	Twist Joint::twist(const double &dq)const{
		Twist twist;
		Eigen::Vector3d speed_lin;
		Eigen::Vector3d speed_rot;
		if(type == JointType::Rotational){
			speed_lin << 0.0, 0.0, 0.0;
			speed_rot << dq*axis;
		}
		if(type == JointType::Translational){
			speed_lin << dq*axis;
			speed_rot << 0.0, 0.0, 0.0;
		}
		else{
			twist << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		}
		twist << speed_lin, speed_rot;
		return twist;
	}
}
