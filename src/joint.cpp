#include "../include/frame.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string.h>


namespace robo {

	typedef enum class { None, Rotational, Translational } JointType;

	class Joint{
	public:

		// Members
		int id;
		JointType type;
		Eigen::Vector3d axis;
		Frame frame;

		// Constructors
		Joint(const int id_in, const Frame frame_in, const Eigen::Vector3d axis_in, 
			const JointType type_in=JointType::Rotational):
		id(id_in), frame(frame_in), axis(axis_in), type(type_in){};

		Joint(const int id_in, const Vector3d origin_in, const Eigen::Vector3d axis_in, 
			const JointType type_in=JointType::Rotational):
		id(id_in), frame(origin_in), axis(axis_in), type(type_in){};

		// Member functions
		Frame pose(const double& q)const{
			if(type == JointType::Rotational){
				Eigen::Matrix3d rotation = Eigen::AngleAxisd(q, axis)
				return Frame(frame.origin, rotation)
			}
			if(type == JointType::Translational){ 
				return Frame(frame.origin + q*axis, frame.orientation)
			}
			else{
				return Frame(frame)
			}
		};

		Eigen::Vector6d twist(const double &dq)const{
			Eigen::Vector6d twist;
			Eigen::Vector3d speed_lin;
			Eigen::Vector3d speed_rot;
			if(JointType::Rotational){
				speed_lin << 0.0, 0.0, 0.0;
				speed_rot << dq*axis;
			}
			if(JointType::Translational){
				speed_lin << dq*axis;
				speed_rot << 0.0, 0.0, 0.0;
			}
			else{
				twist << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
			}
			twist << speed_lin, speed_rot;
			return twist
		};

	}

}