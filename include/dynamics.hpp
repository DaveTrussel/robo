#pragma once

#include "chain.hpp"
#include "inertia.hpp"

#include <Eigen/Dense>

#include <vector>

namespace robo{

    class Dynamics{
    public:
        // Members
        VectorXd joint_torques;

        // Constructors
        Dynamics(const Chain& chain);
        
        // Member functions
        int calculate_generalized_forces(const VectorXd& q, const VectorXd& dq, const VectorXd& ddq,
                              const std::vector<Wrench>& wrenches_extern,
                              const Vector3d& gravity=Vector3d(0.0, 0.0, -9.80665));
        /*
         * solves the inverse dynamics based on a recursive newton-euler algorithm
         * TODO velocities, accelerations, wrenches  of each link are calculated "for free" -> make them accessable
         */

        inline int calculate_generalized_forces(const VectorXd& q, const VectorXd& dq,const VectorXd& ddq,
                              const Vector3d& gravity=Vector3d(0.0, 0.0, -9.80665)){
            return calculate_generalized_forces(q, dq, ddq, std::vector<Wrench>(chain.nr_links), gravity);
        };

        inline int calculate_generalized_forces(const VectorXd& q, const Vector3d& gravity=Vector3d(0.0, 0.0, -9.80665)){
            return calculate_generalized_forces(q, VectorXd::Zero(chain.nr_joints), VectorXd::Zero(chain.nr_joints), std::vector<Wrench>(chain.nr_links), gravity);
        };
    
    private:
        // Members
        Chain chain;
        int nr_links;
        int nr_joints;
        std::vector<Frame> trans_to_parent; // i_X_lambda
        std::vector<Twist> motion_subspace; // S
        std::vector<Twist> velocities;
        std::vector<Twist> accelerations;
        std::vector<Wrench> wrenches;
    };

}   
