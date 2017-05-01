#include "../include/frame.hpp"
#include <Eigen/Dense>

namespace robo {

	// Constructors
	Frame::Frame(const Eigen::Vector3d& vec, const Eigen::Matrix3d& rot):
		origin(vec), orientation(rot){}

	Frame::Frame(const Eigen::Vector3d& vec):
		origin(vec), orientation(Eigen::Matrix3d::Identity()){}

	Frame::Frame():
		origin(Eigen::Vector3d::Zero()), orientation(Eigen::Matrix3d::Identity()){}

	// Named constructors
	Frame Frame::DenavitHartenberg(double a, double alpha, double d, double theta){
		return Frame(); // TODO
	}

	Frame Frame::DenavitHartenberg_Craig1989(double a, double alpha, double d, double theta){
		return Frame(); // TODO
	}

	// Member functions
	Eigen::Matrix4d Frame::as_homogeneous_matrix()const{
		Eigen::Matrix4d homo = Eigen::Matrix4d::Constant(0.0);
		homo.block(0,0,3,3) = orientation;
		homo.block(0,3,3,1) = origin;
		homo.row(3) << 0.0, 0.0, 0.0, 1.0;
		return homo;
	}

	// Operators
	Eigen::Vector3d Frame::operator *(const Eigen::Vector3d & arg) const{
	    return origin + orientation*arg;
	}

	Frame operator *(const Frame& left, const Frame& rigth){
			return Frame(left.orientation*rigth.origin+left.origin,
						 left.orientation*rigth.orientation);
	}
}
