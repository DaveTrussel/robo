#pragma once

namespace robo {

	class Rotation;
	class Frame;
	class Wrench;

	class Rotation{
	public:

		Matrix3d matrix;

		Rotation(int in, double angle)

		Vector3d operator*(const Vector3d& v) const;

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

		Vector3d origin;
		Rotation orientation;

		Frame(const Vector3d& vec, const Rotation& rot);

		static Frame DenavitHartenberg(double a, double alpha, double d, double theta);
		static Frame DenavitHartenberg_Craig1989(double a, double alpha, double d, double theta);

	}

	class Twist{
	public:
		Vector3d velocity_translation;
		Vector3d velocity_rotation;

		Twist(const Vector3d& vel, const Vector3d& rot_vel);

	}

	class Wrench{
	public:
		Vector3d force;
		Vector3d torque;

		Wrench(const Vector& f,const Vector& t):force(f),torque(t) {};
	}

}
