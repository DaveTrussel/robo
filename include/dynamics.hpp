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
							  const Eigen::VectorXd& ddq, const Eigen::Vector3d& gravity,
							  const std::vector<Wrench>& wrenches_extern);
	private:
		// Members
		Chain chain;
		std::vector<Frame> link_frames; // X
		std::vector<Twist> unit_twists; // S
		std::vector<Twist> velocities;
		std::vector<Twist> accelerations;
		std::vector<Wrench> wrenches;
	};

}	
