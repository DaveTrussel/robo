#include "../include/kinematics.hpp"
#include <Eigen/Dense>
#include <iostream>

namespace robo{

    // Constructors
    Kinematics::Kinematics(const Chain& chain_, int max_iter_, double eps_, double eps_joints_):
    chain(chain_), nr_joints(chain.nr_joints), nr_links(chain.nr_links),
    joint_roots(nr_joints), joint_tips(nr_joints),
    max_iter(max_iter_), eps(eps_), eps_joints(eps_joints_)
    {   
        link_tips = std::vector<Frame>(nr_links);

        f_end = Frame();

        q =                 VectorXd::Zero(nr_joints);
        delta_q =           VectorXd::Zero(nr_joints);
        q_out =             VectorXd::Zero(nr_joints);
        weights_damping =   VectorXd::Zero(nr_joints);
        bias =              VectorXd::Constant(chain.nr_joints, 1e-16);
        jacobian =          MatrixXd::Zero(6, nr_joints);
        jacobian_weighted = MatrixXd::Zero(6, nr_joints);
        A =                 MatrixXd::Zero(nr_joints, nr_joints);
        b =                 VectorXd::Zero(nr_joints);
        
        weights_IK << 1, 1, 1, 0.1, 0.1, 0.1;

        int iter_joint = 0;
        for(auto link: chain.links){
            if(link.has_joint()){
                q_min[iter_joint] = link.joint.parameters.q_min;
                q_max[iter_joint] = link.joint.parameters.q_max;
                ++iter_joint;
            }
        }
    }
    
    // Member functions
    void Kinematics::joint_to_cartesian(const VectorXd& q_target){
        int iter_joint = 0;
        f_end = Frame();
        for(int iter_link=0; iter_link<nr_links; iter_link++){
            if(chain.links[iter_link].has_joint()){
                joint_roots[iter_joint] = f_end;
                f_end = f_end * chain.links[iter_link].pose(q_target[iter_joint]);
                joint_tips[iter_joint] = f_end;
                iter_joint++;
            }
            else{
                f_end = f_end * chain.links[iter_link].pose(0.0);
            }
            link_tips[iter_link] = f_end;
        }
    }

    int Kinematics::cartesian_to_joint(const Frame& f_in, const VectorXd& q_init){
        int error_code_ccd = cartesian_to_joint_ccd(f_in, q_init);
        int error_code_levenverg = cartesian_to_joint_levenberg(f_in, q_out);
        return 0;
    }

    int Kinematics::cartesian_to_joint_levenberg(const Frame& f_in, const VectorXd& q_init){
        // modified version of
        //https://groups.csail.mit.edu/drl/journal_club/papers/033005/buss-2004.pdf
        Vector6d residual = Vector6d::Zero();
        Vector6d factor_damp = Vector6d::Constant(0.1);
        double residual_norm = std::numeric_limits<double>::max();
        double residual_norm_squared = std::numeric_limits<double>::max();
        q = q_init;
        
        for(int i=0; i<max_iter; ++i){
            joint_to_cartesian(q);
            calculate_jacobian(q);
            residual = f_end - f_in;
            residual_norm = (weights_IK.asDiagonal()*residual).norm();
            if(residual_norm < eps){
                q_out = q;
                error_norm_IK = residual_norm;
                return (error = E_NO_ERROR);
            }
            A = jacobian * jacobian.transpose();
            residual_norm_squared = residual_norm * residual_norm;
            factor_damp = Vector6d::Constant(residual_norm_squared);
            A.diagonal() += factor_damp;
            b = residual;
            delta_q = A.colPivHouseholderQr().solve(b);
            q += jacobian.transpose() * delta_q;

            // check joint limits
            for(int i=0; i<nr_joints; ++i){
                if(q[i] < q_min[i]){ 
                    q[i] = q_min[i];
                } 
                if(q[i] > q_max[i]){ 
                    q[i] = q_max[i];
                }
            }

        }
        q_out = q;
        error_norm_IK = residual_norm;
        return (error = E_MAX_ITERATIONS);
    }

    int Kinematics::cartesian_to_joint_ccd(const Frame& f_in, const VectorXd& q_init){
        // i = iter_joint
        // Pih (vec from current frame i to end effector)
        // Pi (pos of frame i)
        // Pid (vec from current frame i to desired end effector)
        
        // calculate forward kinematics
        // Calculate Pih = P_end - Pi (vector from each joint to current end eff.)
        // Set Ph = P_endeff_current
        // Compute current error
        // Check switch criterion (change to levenberg?)

        // cycle through joints (top to bottom)
        // if rotational
        //      compute theta:
        //      calculate weights wp, w0
        //      calculate k1, k2, k3
        //      calculate theta = atan(-k3/(k1-k2))
        //      check +- pi solutions (+-2pi as well?)
        //      check if (k1 - k2)cos(theat) - k3*sin(theta) < 0 --> max
        // if translational
        //      compute lambda = delta_pos.dot(axis)
        //      check boundary constraints
        // else nothing
        return 0;
    }

    void Kinematics::calculate_jacobian(const VectorXd& q){
        int iter_joint = 0;
        for(int iter_link=0; iter_link<chain.nr_links; ++iter_link){
            if (chain.links[iter_link].has_joint()) {
                // compute twist of the end effector motion caused by joint [jointndx]; expressed in base frame, with vel. ref. point equal to the end effector
                Twist unit_twist = rotate_twist(joint_roots[iter_joint].orientation, chain.links[iter_link].twist(q(iter_joint), 1.0));
                Twist end_twist = change_twist_reference(unit_twist, f_end.origin - joint_tips[iter_joint].origin);
                jacobian.block<3,1>(0, iter_joint) << end_twist.linear;
                jacobian.block<3,1>(3, iter_joint) << end_twist.rotation;
                ++iter_joint;
            }
        }
    }

}
