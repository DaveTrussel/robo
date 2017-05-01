#include "../include/frame.hpp"
#include <Eigen/Dense>

namespace robo {

	class Frame{
	public:
		// Members
		Eigen::Vector3d origin;
		Eigen::Matrix3d orientation;

		// Constructors
		Frame(const Eigen::Vector3d& vec, const Eigen::Matrix3d& rot):
			origin(vec), orientation(rot){}

		Frame(const Eigen::Vector3d& vec):
			origin(vec), orientation(Eigen::Matrix3d::Identity()){}

		Frame():
			origin(Eigen::Vector3d::Zero()), orientation(Eigen::Matrix3d::Identity()){}

		// Named constructors
		static Frame DenavitHartenberg(double a, double alpha, double d, double theta){
			return Frame(); // TODO
		}

		static Frame DenavitHartenberg_Craig1989(double a, double alpha, double d, double theta){
			return Frame(); // TODO
		}
		
		// Operators
		Eigen::Vector3d operator *(const Eigen::Vector3d & arg) const{
		    return origin + orientation*arg;
		}
	};

	Frame operator *(const Frame& left, const Frame& rigth){
			return Frame(left.orientation*rigth.origin+left.origin,
						 left.orientation*rigth.orientation)
		}
}