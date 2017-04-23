#pragma once

namespace robo {

	class Rotation;
	class Frame;
	class Twist;
	class Wrench;

	class Rotation{
	public:

		Eigen::Matrix3d matrix;

		Rotation(int in, double angle)

		Eigen::Vector3d operator*(const Eigen::Vector3d& v) const;

		Rotation RotX(double angle);
		Rotation RotY(double angle);
		Rotation RotZ(double angle);

		void rotateX(double angle);
		void rotateY(double angle);
		void rotateZ(double angle);

		void getRollPitchYaw(double& roll, double& pitch, double& yaw);
	}

	class Frame{
	public:

		Eigen::Vector3d origin;
		Rotation orientation;

		Frame(const Eigen::Vector3d& vec, const Rotation& rot);

		static Frame DenavitHartenberg(double a, double alpha, double d, double theta);
		static Frame DenavitHartenberg_Craig1989(double a, double alpha, double d, double theta);

	}

	class Twist{
	public:
		Eigen::Vector3d velocity_translation;
		Eigen::Vector3d velocity_rotation;

		Twist(const Eigen::Vector3d& vel, const Eigen::Vector3d& rot_vel);

	}

	class Wrench{
	public:
		Eigen::Vector3d force;
		Eigen::Vector3d torque;

		Wrench(const Eigen::Vector3d& f,const Eigen::Vector3d& t):force(f),torque(t) {};
	}

}
