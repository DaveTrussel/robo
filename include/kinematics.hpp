#pragma once

#include "chain.hpp"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <iomanip>

namespace robo{

    enum class Error_type { no_error,
                            max_iterations,
                            joint_limit_violation };


    class Kinematics{
    public:
        // Members
        double error_norm_IK;
        Chain chain;
        int nr_joints;
        int nr_links;
        Frame f_end; // frame at the end of the chain 
        VectorXd q_out; // result of inverse kinematics
        std::vector<Frame> link_tips; // Frames at end of each link
        MatrixXd jacobian;
        Vector6d weights_IK; // weigths of the IK algorithm (3 position and 3 orientation)

        // Constructors
        Kinematics(const Chain& chain,
                   int max_iter=500,
                   double eps=1e-5);
        
        // Member functions
        inline bool check_joint_limits(const VectorXd& q){
            bool is_within_joint_limits = true;
            for(int i=0; i<nr_joints; ++i){
                if(q_min[i] > q[i] || q[i] > q_max[i]){ is_within_joint_limits = false; }
            }
            // DEBUG
            if(!is_within_joint_limits){ 
                std::cout << "q_min: " << q_min.transpose() << std::endl;
                std::cout << "q    : " << q.transpose()     << std::endl;
                std::cout << "q_max: " << q_max.transpose() << std::endl;
            }
            return is_within_joint_limits;
        };

        inline void enforce_joint_limits(VectorXd& q){
            for(int i=0; i<nr_joints; ++i){
                if(q[i] < q_min[i]){ q[i] = q_min[i]; } 
                if(q[i] > q_max[i]){ q[i] = q_max[i]; }
            }
        };

        void joint_to_cartesian(const VectorXd& q);
        /**
        * Calculates the forward kinematics
        * The result is stored in members 'link_tips' and 'f_end'
        */

        void calculate_jacobian(const VectorXd& q);
        /**
        * Calculates the jacobian (do calculate forward position before calling this function!)
        * The result is stored in member 'jacobian'
        */

        Error_type cartesian_to_joint(const Frame& f_target, const VectorXd& q_init);
        /**
        * Calculates the inverse kinematics (using a combined algorithm)
        * The result is stored in q_out
        * Check the return value to see if the solver was sucessfull (==1)
        */

        Error_type cartesian_to_joint_jacobian_transpose(const Frame& f_target, const VectorXd& q_init);
        /**
        * Calculates the inverse kinematics (using a jacobian transpose algorithm)
        * The result is stored in q_out
        * Check the return value to see if the solver was sucessfull (==1)
        */

        Error_type cartesian_to_joint_levenberg(const Frame& f_target, const VectorXd& q_init);
        /**
        * Calculates the inverse kinematics (using a dynamically damped Levenberg-Marquardt algorithm)
        * The result is stored in q_out
        * Check the return value to see if the solver was sucessfull (==1)
        */

        Error_type cartesian_to_joint_sugihara_joint_limits(const Frame& f_target, const VectorXd& q_init);
        /**
        * Calculates the inverse kinematics (using a damped least squares algorithm) with "soft" joint limits
        * The result is stored in q_out
        * Check the return value to see if the solver was sucessfull (==1)
        */

        Error_type cartesian_to_joint_ccd(const Frame& f_target, const VectorXd& q_init);
        /**
        * Calculates the inverse kinematics (using a cyclic coordinate descent algorithm)
        * The result is stored in q_out
        * Check the return value to see if the solver was sucessfull (==1)
        */

        Error_type cartesian_to_joint_sugihara(const Frame& f_target, const VectorXd& q_init);
        /**
        * Calculates the inverse kinematics (using a dynamically damped Levenberg-Marquardt algorithm)
        * The result is stored in q_out
        * Check the return value to see if the solver was sucessfull (==1)
        */

    private:
        // Members
        Error_type error;

        VectorXd q; // Joint positions
        VectorXd q_min;
        VectorXd q_max;
        VectorXd delta_q;
        VectorXd bias;
        
        MatrixXd jacobian_weighted;
        
        MatrixXd A; // Linear system: Ax = b
        VectorXd b; // Linear system: Ax = b
        
        std::vector<Frame> joint_roots;
        std::vector<Frame> joint_tips;

        int max_iter;
        double eps;

        void clamp_magnitude(Vector6d& residual, double max_norm);
    };

}
