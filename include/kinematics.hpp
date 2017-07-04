#pragma once

#include "chain.hpp"
#include <Eigen/Dense>
#include <vector>

namespace robo{

    class Kinematics{
    public:
        // Members
        double error_norm_IK;
        Chain chain;
        Frame f_end; // frame at the end of the chain 
        VectorXd q_out; // result of inverse kinematics
        std::vector<Frame> link_tips; // Frames at end of each link
        MatrixXd jacobian;
        Vector6d weights_IK; // weigths of the IK algorithm (3 position and 3 orientation)


        // Error code constants 
        // TODO make this an enum class for clarity e.g. InvKinStatus::no_error?
        static constexpr int E_NO_ERROR =                    1;
        static constexpr int E_JOINTS_GRADIENT_TOO_SMALL =  -1;
        static constexpr int E_JOINTS_INCREMENT_TOO_SMALL = -2;
        static constexpr int E_MAX_ITERATIONS =             -3;
        
        // Constructors
        Kinematics(const Chain& chain,
                   int max_iter=500,
                   double eps=1e-5,
                   double eps_joints=1e-16);
        
        // Member functions
        void joint_to_cartesian(const VectorXd& q);
        /**
        * Calculates the forward kinematics
        * The result is stored in link_tips and f_end
        */

        int cartesian_to_joint(const Frame& f_in, const VectorXd& q_init);
        /**
        * Calculates the inverse kinematics (using a dynamically damped Levenberg-Marquardt algorithm)
        * The result is stored in q_out
        * Check the return value to see if the solver was sucessfull (==1)
        */

        void calculate_jacobian(const VectorXd& q);
        /**
        * Calculates the jacobian (calculate forward position before)
        * The result is stored in jacobian
        */
        
    private:
        // Members
        int nr_joints;
        int nr_links;
        int error;

        VectorXd q; // Joint positions
        VectorXd q_min;
        VectorXd q_max;
        VectorXd delta_q;
        VectorXd weights_damping;
        VectorXd bias;
        
        MatrixXd jacobian_weighted;
        
        MatrixXd A; // Linear system: Ax = b
        VectorXd b; // Linear system: Ax = b
        
        std::vector<Frame> joint_roots;
        std::vector<Frame> joint_tips;

        int max_iter;
        double eps;
        double eps_joints;
    };

}   
