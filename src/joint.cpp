#include "../include/core.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string.h>


namespace robo {

	class Joint{
	public:
		
		int id;
		Eigen::Vector3d axis;
		Eigen::Vector3d origin;

		Joint(const int id, const Eigen::Vector3d axis; Eigen::Vector3d origin){
			this->id = id;
			this->type = type;
			this->origin = origin;
			this->axis = axis;
		};


		// Named constructors
		static Joint createRotX(const int id, Eigen::Vector3d origin){
			return Joint(id, Eigen::Vector3d::UnitX(); origin)
		};

		static Joint createRotY(const int id, Eigen::Vector3d origin){
			return Joint(id, Eigen::Vector3d::UnitY(); origin)
		};
		
		static Joint createRotZ(const int id, Eigen::Vector3d origin){
			return Joint(id, Eigen::Vector3d::UnitZ(); origin)
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