#pragma once

#include "core.hpp"

namespace robo {

	typedef enum { None,RotX,RotY,RotZ } JointType;

	class Joint{
	public:
		
		int id;
		JointType type;
		Eigen::Vector3d axis;
		Eigen::Vector3d origin;

		Joint(const int id, const JointType& type=None, Eigen::Vector3d origin){
			this->id = id;
			this->type = type;
			this->origin = origin;
			if(type == RotX){
				axis = Eigen::Vector3d::UnitX();
			}
			if(type == RotY){
				axis = Eigen::Vector3d::UnitY();
			}
			if(type == RotZ){
				axis = Eigen::Vector3d::UnitZ();
			}
			else{
				axis = Eigen::Vector3d::Zero();
			}
		};

		Frame pose(const double& q)const{
		//TODO
		};

		Eigen::Vector6d twist(const double& q, const double &dq)const;

		Eigen::Matrix3d rotation(const double& q)const{
			return Eigen::AngleAxisd(q, axis)
		};

	}

}