#include "../include/kinematics.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <cmath>

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
        q_min =             VectorXd::Zero(nr_joints);
        q_max =             VectorXd::Zero(nr_joints);
        q_out =             VectorXd::Zero(nr_joints);
        weights_damping =   VectorXd::Zero(nr_joints);
        bias =              VectorXd::Constant(chain.nr_joints, 1e-16);
        jacobian =          MatrixXd::Zero(6, nr_joints);
        jacobian_weighted = MatrixXd::Zero(6, nr_joints);
        A =                 MatrixXd::Zero(nr_joints, nr_joints);
        b =                 VectorXd::Zero(nr_joints);
        
        weights_IK << 1, 1, 1, 0.1, 0.1, 0.1;

        int iter_joint = 0;
        for(const auto& link: chain.links){
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
        for(int iter_link=0; iter_link<nr_links; ++iter_link){
            if(chain.links[iter_link].has_joint()){
                joint_roots[iter_joint] = f_end;
                f_end = f_end * chain.links[iter_link].pose(q_target[iter_joint]);
                joint_tips[iter_joint] = f_end;
                ++iter_joint;
            }
            else{
                f_end = f_end * chain.links[iter_link].pose(0.0);
            }
            link_tips[iter_link] = f_end;
        }
    }

    int Kinematics::cartesian_to_joint(const Frame& f_target, const VectorXd& q_init){
        cartesian_to_joint_ccd(f_target, q_init, max_iter/10);
        int error_code_sugihara = cartesian_to_joint_sugihara(f_target, q_out);
        //int error_code_sugihara = cartesian_to_joint_sugihara(f_target, q_init);
        return error_code_sugihara;
    }

    int Kinematics::cartesian_to_joint_jacobian_transpose(const Frame& f_target, const VectorXd& q_init){
        /*
        / Simple jacobian transpose method (not robust, but fast) will not handle singularities well
        */
        Vector6d residual = Vector6d::Zero();
        double residual_norm = std::numeric_limits<double>::max();
        q = q_init;
        double alpha = 0;
        Vector6d tmp = Vector6d::Zero();
        for(int i=0; i<max_iter; ++i){
            joint_to_cartesian(q);
            calculate_jacobian(q);
            residual = f_target - f_end;
            residual_norm = (weights_IK.asDiagonal()*residual).norm();
            if(residual_norm < eps){
                q_out = q;
                error_norm_IK = residual_norm;
                return (error = E_NO_ERROR);
            }
            tmp = jacobian * jacobian.transpose() * residual;
            alpha = residual.dot(tmp) / tmp.dot(tmp);
            q += alpha * jacobian.transpose() * residual;
        }
        q_out = q;
        error_norm_IK = residual_norm;
        return (error = E_MAX_ITERATIONS);
    }

    int Kinematics::cartesian_to_joint_levenberg(const Frame& f_target, const VectorXd& q_init){
        // modified version of
        // https://groups.csail.mit.edu/drl/journal_club/papers/033005/buss-2004.pdf
        Vector6d residual = Vector6d::Zero();
        Vector6d factor_damp = Vector6d::Constant(0.1);
        double residual_norm = std::numeric_limits<double>::max();
        double residual_norm_squared = std::numeric_limits<double>::max();
        q = q_init;
        
        for(int i=0; i<max_iter; ++i){
            joint_to_cartesian(q);
            calculate_jacobian(q);
            residual = f_target - f_end;
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
            delta_q = A.colPivHouseholderQr().solve(b); // TODO not actually joint increments but use as placeholder for result of A\b = x
            q += jacobian.transpose() * delta_q;

            // check joint limits
            // enforce_joint_limits(q);
        }
        q_out = q;
        error_norm_IK = residual_norm;
        return (error = E_MAX_ITERATIONS);
    }

    int Kinematics::cartesian_to_joint_ccd(const Frame& f_target, const VectorXd& q_init, const int max_iter_ccd){
        /*
        * Based on " A Combined Optimization Method for Solving the Inverse Kinematics Problem of 
        * Mechanical Manipulators" by Wang & Chen 1991
        */
        q = q_init;
        std::vector<Vector3d> deltas_to_end_effector; // TODO make this more efficient
        deltas_to_end_effector.reserve(nr_joints);
        std::vector<Vector3d> deltas_to_target; // TODO make this more efficient
        deltas_to_target.reserve(nr_joints);
        Vector6d residual = Vector6d::Zero();
        double residual_norm = std::numeric_limits<double>::max();
        Vector3d Pih; // TODO rename
        Vector3d Pid;

        for(int iter=0; iter<max_iter_ccd; iter++){
            joint_to_cartesian(q);
            residual = f_target - f_end;
            residual_norm = (weights_IK.asDiagonal()*residual).norm();
            if(residual_norm < eps){
                q_out = q;
                error_norm_IK = residual_norm;
                return (error = E_NO_ERROR);
            }
            //Matrix3d hh = f_end.orientation;
            //Matrix3d dd = f_target.orientation;


/*
            // calculate vectors to end effector for each joint
            int iter_joint = nr_joints-1;
            for(int iter_link=nr_links-1; iter_link>=0; --iter_link){
                if(chain.links[iter_link].has_joint()){
                    deltas_to_end_effector[iter_joint] = f_end.origin - joint_roots[iter_joint].origin;
                    --iter_joint;
                }
            }
*/

            // sweep from last joint to first and calculate joint increments
            int iter_joint = nr_joints-1;
            for(int iter_link=nr_links-1; iter_link>=0; --iter_link){
                if(chain.links[iter_link].has_joint()){
                    joint_to_cartesian(q); // TODO not the whole forward kinematics needed just upper part would needed to be updated
                    Matrix3d hh = f_end.orientation;
                    Matrix3d dd = f_target.orientation;
                    Joint& joint = chain.links[iter_link].joint;
                    Pih = f_end.origin - joint_roots[iter_joint].origin;;
                    Pid = f_target.origin - joint_roots[iter_joint].origin;
                    Matrix3d linkframe_to_world = joint_roots[iter_joint].orientation.transpose();
                    Vector3d axis = linkframe_to_world * joint.axis;
                    // calculate joint increments depending on joint type
                    if(joint.type == Joint_type::Rotational){
                        // calculate weights
                        double pid_norm = Pid.norm();
                        double pih_norm = Pih.norm();
                        // calculate coefficients of the objective function which is to be maximised
                        double rho = std::min(pid_norm, pih_norm)/std::max(pid_norm, pih_norm);
                        double wp = 1 * (1.0 + rho);
                        double k1 = wp * axis.dot(Pid) * axis.dot(Pih);
                        double k2 = wp * Pid.dot(Pih);
                        Vector3d k3_tmp = wp * Pih.cross(Pid);
                        for(int i=0; i<3; ++i){
                            k1      += axis.dot(dd.col(i)) * axis.dot(hh.col(i));
                            k2      += dd.col(i).dot(hh.col(i));
                            k3_tmp  += hh.col(i).cross(dd.col(i));
                        }
                        double k3 = axis.dot(k3_tmp);
                        auto objective_func = [&](double t){ return k1*(1-std::cos(t) + k2*std::cos(t) + k3*std::sin(t)); };
                        //auto second_derivative = [&](double t){ return (k1-k2) * std::cos(t) - k3 * std::sin(t);             };
                        double theta = std::atan2(k3, (k2-k1)); // 1st derivative equals zero
                        // tangens is periodic therefore more possible maxima
                        std::vector<double> candidates {theta, theta + M_PI, theta - M_PI};
                        double delta_q = std::numeric_limits<double>::min();
                        // handle joint limits
                        double delta_q_min = q_min[iter_joint] - q[iter_joint];
                        double delta_q_max = q_max[iter_joint] - q[iter_joint];
                        for(auto& candidate : candidates){
                            // check if candidate is within joint limits, if not, replace with limit
                            if(candidate > delta_q_max){ candidate = delta_q_max; }
                            if(candidate < delta_q_min){ candidate = delta_q_min; }
                            // check which candidate is the maximum
                            if(objective_func(candidate) > objective_func(delta_q)){ delta_q = candidate; }
                        }
                        // limit iteration step size
                        double max_step = 0.1;
                        if(delta_q >  max_step){delta_q =  max_step;}
                        if(delta_q < -max_step){delta_q = -max_step;}
                        q[iter_joint] += delta_q;
                    }
                    if(joint.type == Joint_type::Translational){
                        Vector3d delta_pos = Pid - Pih;
                        double lambda = axis.dot(delta_pos);
                        q[iter_joint] += lambda;
                    }
                    --iter_joint;
                }
            }
        }

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
        //      calculate tan(theta)= -k3/(k1-k2)
        //      check +- pi solutions (+-2pi as well?)
        //      check if (k1 - k2)cos(theat) - k3*sin(theta) < 0 --> max
        // if translational
        //      compute lambda = delta_pos.dot(axis)
        //      check boundary constraints
        // else nothing
        q_out = q;
        return E_MAX_ITERATIONS;
    }

    int Kinematics::cartesian_to_joint_sugihara(const Frame& f_target, const VectorXd& q_init){
        /* 
        / Based on "Solvability-unconcerned Inverse Kinematics based on Levenberg-Marquardt method with Robuts Damping"
        / by Tomomichi Sugihara, 2009, International Conference on Humanoid Robots
        */ 
        Vector6d residual = Vector6d::Zero();
        VectorXd factor_damp = VectorXd::Constant(nr_joints, 0.1);
        double residual_norm = std::numeric_limits<double>::max();
        double residual_norm_squared = std::numeric_limits<double>::max();
        MatrixXd H(nr_joints, nr_joints);
        VectorXd g(nr_joints);
        q = q_init;
        
        for(int i=0; i<max_iter; ++i){
            joint_to_cartesian(q);
            calculate_jacobian(q);
            residual = f_target - f_end;
            residual_norm = (weights_IK.asDiagonal()*residual).norm();
            if(residual_norm < eps){
                q_out = q;
                error_norm_IK = residual_norm;
                return (error = E_NO_ERROR);
            }
            g = jacobian.transpose() * weights_IK.asDiagonal() * residual;
            H = jacobian.transpose() * weights_IK.asDiagonal() * jacobian;
            residual_norm_squared = residual_norm * residual_norm;
            factor_damp = VectorXd::Constant(nr_joints, residual_norm_squared + 1e-10);
            H.diagonal() += factor_damp;
            delta_q = H.colPivHouseholderQr().solve(g); 
            q += delta_q;


            // check joint limits
            // enforce_joint_limits(q);
        }
        q_out = q;
        error_norm_IK = residual_norm;
        return (error = E_MAX_ITERATIONS);
    }

    void Kinematics::calculate_jacobian(const VectorXd& q){
        int iter_joint = 0;
        for(int iter_link=0; iter_link<chain.nr_links; ++iter_link){
            if (chain.links[iter_link].has_joint()) {
                // compute twist of the end effector motion caused by joint [jointndx]; expressed in world frame, with vel. ref. point equal to the end effector
                Twist unit_twist = rotate_twist(joint_roots[iter_joint].orientation, chain.links[iter_link].twist(q(iter_joint), 1.0)); // represent joint unit twist in world frame
                Twist end_twist = change_twist_reference(unit_twist, f_end.origin - joint_tips[iter_joint].origin); // change from joint_i seen on endeffector
                jacobian.block<3,1>(0, iter_joint) << end_twist.linear;
                jacobian.block<3,1>(3, iter_joint) << end_twist.rotation;
                ++iter_joint;
            }
        }
    }

}
