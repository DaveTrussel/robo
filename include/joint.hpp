#pragma once

#include "frame.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace robo {

	enum class JointType { None, Rotational, Translational };

	class Joint{
	public:
		
		// Members
		int id;
		JointType type;
		Vector3d axis;
		Frame frame;
		struct {
			double rotor_inertia = 0.0;
			double linear_friction_coeff = 0.0;
		} parameters;

		// Constructors
		Joint(const int id_in, const Frame frame_in, const Vector3d axis_in, 
			  const JointType type_in=JointType::Rotational):
			id(id_in), type(type_in), axis(axis_in), frame(frame_in){};

		// Member functions
		Frame pose(const double& q)const{
			if(type == JointType::Rotational){
				Matrix3d rotation = (Matrix3d)Eigen::AngleAxisd(q, axis);
				return Frame(frame.origin, rotation);
			}
			if(type == JointType::Translational){ 
				return Frame(frame.origin + q*axis, frame.orientation);
			}
			else{
				return Frame(frame);
			}
		};

		Twist twist(const double &dq)const{
			Vector3d speed_lin = Vector3d::Zero();
			Vector3d speed_rot = Vector3d::Zero();
			if(type == JointType::Rotational){
				speed_rot << dq*axis;
			}
			if(type == JointType::Translational){
				speed_lin << dq*axis;
			}
			return Twist(speed_lin, speed_rot);
		};
	};

}
