#include "kinematics.hpp"

#include <Eigen/Dense>

#include <iostream>
#include <iomanip>
#include <cmath>

namespace robo{

    // Constructors
    Kinematics::Kinematics(const Chain& chain_, int max_iter_, double eps_):
    chain(chain_), nr_joints(chain_.nr_joints), nr_links(chain_.nr_links),
    joint_roots(chain_.nr_joints), joint_tips(chain_.nr_joints),
    max_iter(max_iter_), eps(eps_)
    {   
        link_tips = std::vector<Frame>(nr_links);

        f_end = Frame();

        q =                 VectorXd::Zero(nr_joints);
        delta_q =           VectorXd::Zero(nr_joints);
        q_min =             VectorXd::Zero(nr_joints);
        q_max =             VectorXd::Zero(nr_joints);
        q_out =             VectorXd::Zero(nr_joints);
        bias =              VectorXd::Constant(chain.nr_joints, 1e-16);
        jacobian =          MatrixXd::Zero(6, nr_joints);
        jacobian_weighted = MatrixXd::Zero(6, nr_joints);
        A =                 MatrixXd::Zero(nr_joints, nr_joints);
        b =                 VectorXd::Zero(nr_joints);
        
        weights_cartesian << 1, 1, 1, 0.1, 0.1, 0.1;

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

    void Kinematics::calculate_jacobian(const VectorXd& q){
    // always call joint_to_cartesian(q) before calling this function!
        int iter_joint = 0;
        for(int iter_link=0; iter_link<chain.nr_links; ++iter_link){
            if (chain.links[iter_link].has_joint()) {
                // compute twist of the end effector motion caused by joint [jointndx]; expressed in world frame, with vel. ref. point equal to the end effector
                Twist unit_twist = joint_roots[iter_joint].orientation * chain.links[iter_link].twist(q(iter_joint), 1.0); // represent joint unit twist in world frame
                Twist end_twist = change_twist_reference(unit_twist, f_end.origin - joint_tips[iter_joint].origin); // change from joint_i seen on endeffector
                jacobian.block<3,1>(0, iter_joint) << end_twist.linear;
                jacobian.block<3,1>(3, iter_joint) << end_twist.rotation;
                ++iter_joint;
            }
        }
    }

    Error_type Kinematics::cartesian_to_joint(const Frame& f_target, const VectorXd& q_init){
    // this is just a wrapper for the alorithm which is currently deemed "the best"
        //cartesian_to_joint_ccd(f_target, q_init, max_iter/10);
        Error_type error_sugihara = cartesian_to_joint_sugihara(f_target, q_init);
        return error_sugihara;
    }

    Error_type Kinematics::cartesian_to_joint_jacobian_transpose(const Frame& f_target, const VectorXd& q_init){
    /*
     * Simple jacobian transpose method (not robust, but fast) will not handle singularities well
     * does NOT account for joint limits
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
            clamp_magnitude(residual, 0.1);
            residual_norm = (weights_cartesian.asDiagonal()*residual).norm();
            if(residual_norm < eps){
                q_out = q;
                error_norm_IK = residual_norm;
                return (error = Error_type::no_error);
            }
            tmp = jacobian * jacobian.transpose() * residual;
            alpha = residual.dot(tmp) / tmp.dot(tmp);
            q += alpha * jacobian.transpose() * residual;
        }
        q_out = q;
        error_norm_IK = residual_norm;
        return (error = Error_type::max_iterations);
    }

    Error_type Kinematics::cartesian_to_joint_levenberg(const Frame& f_target, const VectorXd& q_init){
    // modified version of
    // https://groups.csail.mit.edu/drl/journal_club/papers/033005/buss-2004.pdf
    // does NOT account for joint limits
        Vector6d residual = Vector6d::Zero();
        Vector6d factor_damp = Vector6d::Constant(0.1);
        double residual_norm = std::numeric_limits<double>::max();
        double residual_norm_squared = std::numeric_limits<double>::max();
        q = q_init;
        
        for(int i=0; i<max_iter; ++i){
            joint_to_cartesian(q);
            calculate_jacobian(q);
            residual = f_target - f_end;
            clamp_magnitude(residual, 0.1);
            residual_norm = (weights_cartesian.asDiagonal()*residual).norm();
            if(residual_norm < eps){
                q_out = q;
                error_norm_IK = residual_norm;
                return (error = Error_type::no_error);
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
        return (error = Error_type::max_iterations);
    }

    Error_type Kinematics::cartesian_to_joint_sugihara(const Frame& f_target, const VectorXd& q_init){
    /* 
     * Based on "Solvability-unconcerned Inverse Kinematics based on Levenberg-Marquardt method with Robuts Damping"
     * by Tomomichi Sugihara, 2009, International Conference on Humanoid Robots
     * does NOT account for joint_limits
     */ 
        Vector6d residual = Vector6d::Zero();
        VectorXd factor_damp = VectorXd::Constant(nr_joints, 0.1);
        double residual_norm = std::numeric_limits<double>::max();
        double residual_norm_squared = std::numeric_limits<double>::max();
        q = q_init;
        
        for(int i=0; i<max_iter; ++i){
            joint_to_cartesian(q);
            calculate_jacobian(q);
            residual = f_target - f_end;
            clamp_magnitude(residual, 0.5);
            residual_norm = (weights_cartesian.asDiagonal()*residual).norm();
            if(residual_norm < eps){
                q_out = q;
                error_norm_IK = residual_norm;
                return (error = Error_type::no_error);
            }
            b = jacobian.transpose() * weights_cartesian.asDiagonal() * residual;
            A = jacobian.transpose() * weights_cartesian.asDiagonal() * jacobian;
            residual_norm_squared = residual_norm * residual_norm;
            factor_damp = VectorXd::Constant(nr_joints, residual_norm_squared + 1e-10);
            A.diagonal() += factor_damp;
            delta_q = A.colPivHouseholderQr().solve(b);
            limit_step_size(delta_q, 0.25);
            q += delta_q;


            // check joint limits
            // enforce_joint_limits(q);
        }
        q_out = q;
        error_norm_IK = residual_norm;
        return (error = Error_type::max_iterations);
    }

    Error_type Kinematics::cartesian_to_joint_sugihara_joint_limits(const Frame& f_target, const VectorXd& q_init){
    /*
     * Variation of damped least squares (levenberg-marquardt) with selective weights
     * based on "ROBUST INVERSE KINEMATICS USING DAMPED LEAST SQUARES WITH DYNAMIC WEIGHTING" by Schinstock et al.
     */
        Vector6d residual = Vector6d::Zero();
        Vector6d residual_w = Vector6d::Zero();
        VectorXd factor_damp = VectorXd::Constant(nr_joints, 0.1);
        VectorXd weights_joints = VectorXd::Constant(nr_joints, 1.0);
        double residual_norm = std::numeric_limits<double>::max();
        double alpha_0 = 0.1;
        double w_q_0 = 1e-10;
        std::vector<double> u_l(nr_joints, 1.0);
        constexpr double du = 0.5;
        constexpr double d_lim_0 = 0.5;
        MatrixXd H(nr_joints, nr_joints);
        q = q_init;
        
        for(int i=0; i<max_iter; ++i){
            joint_to_cartesian(q);
            calculate_jacobian(q);
            residual = f_target - f_end;
            clamp_magnitude(residual, 0.5);
            //std::cout << "residual: " << residual.transpose() << std::endl;
            residual_w = weights_cartesian.asDiagonal() * residual;
            residual_norm = residual_w.norm();
            if(residual_norm < eps){
                q_out = q;
                error_norm_IK = residual_norm;
                return (error = Error_type::no_error);
            }
            //factor_damp = VectorXd::Constant(nr_joints, residual_norm_squared + 1e-10);
            for(int j=0; j<nr_joints; ++j){
                
                // distance until next joint limit
                double d_lim = distance_to_next_joint_limit(j);     
                // joint limit ramp function
                if(d_lim < d_lim_0){ u_l[j] = std::max(u_l[j]-du, d_lim/d_lim_0); }
                else               { u_l[j] = std::min(u_l[j]+du, 1.0); }
                double alpha_li = alpha_0 * (1.0 - u_l[j]*u_l[j]);
                factor_damp[j] =  alpha_li + 1e-5;
                weights_joints[j] = w_q_0 + (1.0 - w_q_0)*u_l[j];
                //std::cout << "Joint " << j << ":" << q[j] << ", d_l:" << d_lim << ", u_l:" << u_l[j] << ", alpha: " << alpha_li << ", w:" << weights_joints[j] << std::endl;
            }
            //std::cout << std::endl;
            jacobian_weighted = weights_cartesian.asDiagonal() * jacobian * weights_joints.asDiagonal();
            H = jacobian_weighted * jacobian_weighted.transpose();
            H.diagonal() += factor_damp;
            delta_q = H.colPivHouseholderQr().solve(residual_w); // not yet delta_q
            delta_q = weights_joints.asDiagonal() * jacobian_weighted.transpose() * delta_q;
            limit_step_size(delta_q, du/3.0);
            q += delta_q;
            enforce_joint_limits(q);
        }
        q_out = q;
        error_norm_IK = residual_norm;
        return (error = Error_type::max_iterations);
    }

    Error_type Kinematics::cartesian_to_joint_ccd(const Frame& f_target, const VectorXd& q_init){
    /*
     * Based on " A Combined Optimization Method for Solving the Inverse Kinematics Problem of 
     * Mechanical Manipulators" by Wang & Chen 1991
     */
        // TODO not working as intended (find out why)
        // TODO only implemented to check feasability, needs work to speed up execution time
        q = q_init;
        std::vector<Vector3d> deltas_to_end_effector; // TODO make this more efficient
        deltas_to_end_effector.reserve(nr_joints);
        std::vector<Vector3d> deltas_to_target; // TODO make this more efficient
        deltas_to_target.reserve(nr_joints);
        Vector6d residual = Vector6d::Zero();
        double residual_norm = std::numeric_limits<double>::max();
        Vector3d Pih; // TODO rename
        Vector3d Pid;

        for(int iter=0; iter<max_iter; iter++){
            joint_to_cartesian(q);
            residual = f_target - f_end;
            clamp_magnitude(residual, 0.5);
            residual_norm = (weights_cartesian.asDiagonal()*residual).norm();
            if(residual_norm < eps){
                q_out = q;
                error_norm_IK = residual_norm;
                return (error = Error_type::no_error);
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
        return Error_type::max_iterations;
    }

    void Kinematics::clamp_magnitude(Vector6d& residual, double max_norm){
        double norm = residual.norm();
        if(norm > max_norm){ residual = max_norm * residual / norm; }
    }

    void Kinematics::limit_step_size(VectorXd& delta_q, double max_joint_step){
        for(int i=0; i<delta_q.size(); ++i){
            if(delta_q[i] >  max_joint_step){ delta_q[i] =  max_joint_step; }
            if(delta_q[i] < -max_joint_step){ delta_q[i] = -max_joint_step; }
        }
    }

    double Kinematics::distance_to_next_joint_limit(const int& j)const{
    // returns the distance to the next joint limit in the direction of the last delta_q step
        double d_lim = 0;
        if(delta_q[j] >= 0){ d_lim = q_max[j] - q[j]; } // if we were moving towards q_max
        else               { d_lim = q[j] - q_min[j]; } // if we were moving towards q_min
        if(d_lim < 0.0){ d_lim = 0.0; } // if we already moved past the limit return 0 instead of negative number
        return d_lim;
    }

}
