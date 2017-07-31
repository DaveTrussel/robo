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
        int calculate_torques(const VectorXd& q, const VectorXd& dq, const VectorXd& ddq,
                              const std::vector<Wrench>& wrenches_extern,
                              const Vector3d& gravity=Vector3d(0.0, 0.0, -9.80665));

        inline int calculate_torques(const VectorXd& q, const VectorXd& dq,const VectorXd& ddq,
                              const Vector3d& gravity=Vector3d(0.0, 0.0, -9.80665)){
            return calculate_torques(q, dq, ddq, std::vector<Wrench>(chain.nr_links), gravity);
        };

        inline int calculate_torques(const VectorXd& q, const Vector3d& gravity=Vector3d(0.0, 0.0, -9.80665)){
            return calculate_torques(q, VectorXd::Zero(chain.nr_joints), VectorXd::Zero(chain.nr_joints), std::vector<Wrench>(chain.nr_links), gravity);
        };
    
    private:
        // Members
        Chain chain;
        int nr_links;
        int nr_joints;
        std::vector<Frame> trans_from_parent; // X_lambda
        std::vector<Twist> motion_subspace; // S
        std::vector<Twist> velocities;
        std::vector<Twist> accelerations;
        std::vector<Wrench> wrenches;
    };

}   
