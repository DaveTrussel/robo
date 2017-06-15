#pragma once

#include "chain.hpp"
#include <Eigen/Dense>
#include <vector>

namespace robo{

	class Dynamics{
	public:
		// Members
		Eigen::VectorXd joint_torques;

    	// Constructors
		Dynamics(const Chain& chain);
		
		// Member functions
		int calculate_torques(const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
							  const Eigen::VectorXd& ddq, const Eigen::Vector3d& gravity);
	private:
		// Members
		Chain chain;
		std::vector<Frame> link_frames; // X
		std::vector<Vector6d> unit_twists; // S
		std::vector<Vector6d> velocities;
		std::vector<Vector6d> accelerations;
	};

}	
