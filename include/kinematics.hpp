#pragma once

#include "chain.hpp"
#include <Eigen/Dense>
#include <vector>

namespace robo{

	class Kinematics{
	public:
		// Members
		Chain chain;
		Frame f_end; // frame at the end of the chain 
		Eigen::VectorXd q_out; // result of inverse kinematics is stored here
		std::vector<Frame> link_tips; // Frames at end of each link
		Eigen::MatrixXd jacobian;
		Vector6d weights_IK; // weigths of the IK algorithm (3 position and 3 orientation)
		int max_iter;
		double eps;
    	double eps_joints;
    	int error;

    	// Error code constants 
    	// TODO make this an enum class for clarity e.g. InvKinStatus::no_error?
    	static constexpr int E_NO_ERROR = 					 1;
    	static constexpr int E_JOINTS_GRADIENT_TOO_SMALL =  -1;
    	static constexpr int E_JOINTS_INCREMENT_TOO_SMALL = -2;
    	static constexpr int E_MAX_ITERATIONS = 			-3;
		
    	// Constructors
		Kinematics(const Chain& chain,
                   int max_iter=500,
                   double eps=1e-6,
                   double eps_joints=1e-16);

		Kinematics(const Chain& chain, Vector6d weights_IK,
                   int max_iter=500,
                   double eps=1e-6,
                   double eps_joints=1e-16);
		
		// Member functions
		void joint_to_cartesian(const Eigen::VectorXd& q);
		/**
		* Calculates the forward kinematics
		* The result is stored in link_tips and f_end
		*/

		int cartesian_to_joint(const Frame& f_in, const Eigen::VectorXd& q_init);
		/**
		* Calculates the inverse kinematics
		* The result is stored in q_out
		* Check the return value to see if the solver was sucessfull 
		*/

		void calculate_jacobian(const Eigen::VectorXd& q);
		/**
		* Calculates the jacobian
		* The result is stored in jacobian
		*/
		
	private:
		// Members
		Eigen::VectorXd q; // Joint positions
		Eigen::VectorXd q_new;
		Eigen::VectorXd tmp_q;
		Eigen::VectorXd delta_q;
		Eigen::VectorXd grad; // gradient
		Eigen::VectorXd singular_vals; // TODO rename
		std::vector<Frame> joint_roots;
		std::vector<Frame> joint_tips;
		Eigen::JacobiSVD<Eigen::MatrixXd> svd;
	};

}	
