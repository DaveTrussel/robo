#pragma once
#include <Eigen/Dense>

namespace robo {


	class Frame{
	public:

		Eigen::Vector3d origin;
		Eigen::Matrix3d orientation;

		Frame(const Eigen::Vector3d& vec, const Eigen::Matrix3d& rot);

		static Frame DenavitHartenberg(double a, double alpha, double d, double theta);
		static Frame DenavitHartenberg_Craig1989(double a, double alpha, double d, double theta);

	}


}
