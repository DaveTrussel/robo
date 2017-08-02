#define EIGEN_NO_MALLOC
// according to Eigen documentation this macro should not be used in "real-world code"
// however we really dont want any calls to malloc because we are concerned about real-time performance
// results in assertion failure when malloc is called by Eigen.

#include "test_utilities.hpp"
#include "kinematics.hpp"
#include "dynamics.hpp"

#include <Eigen/Dense>

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <chrono>
#include <algorithm>
#include <random>
#include <sys/mman.h>

using namespace robo;
using namespace std;
using namespace std::chrono;

constexpr int nr_steps = 10;
constexpr int nr_iter = 300;
constexpr double abs_tol = 1e-4;
constexpr auto pi = 3.141592653589793238462643383279502884L;

void track_inverse_kinematics(int method, Kinematics kin, int nr_steps){
/*
 * track behaviour of different algorithms
 */
    vector<Error_type> error_types;
    vector<bool>       joint_limit_compliances;
    error_types.reserve(nr_steps);
    joint_limit_compliances.reserve(nr_steps);
    std::function<Error_type(Kinematics*, const Frame&, const VectorXd&)> func = &Kinematics::cartesian_to_joint;
    switch(method){
        case 0: func = &Kinematics::cartesian_to_joint_jacobian_transpose;
                std::cout << "=== Jacobian transpose ==="       << std::endl;
                break;
        case 1: func = &Kinematics::cartesian_to_joint_ccd;
                std::cout << "=== Cyclic Coordinate Descent ===" << std::endl;
                break;
        case 2: func = &Kinematics::cartesian_to_joint_levenberg;
                std::cout << "=== Levenberg-Marquardt ==="       << std::endl;
                break;
        case 3: func = &Kinematics::cartesian_to_joint_sugihara;
                std::cout << "=== Sugihara ==="                  << std::endl;
                break;
        case 4: func = &Kinematics::cartesian_to_joint_sugihara_joint_limits;
                std::cout << "=== Sugihara Joint Limits ==="     << std::endl;
                break;
    }
    bool joint_limit_compliance{false};
    Frame f_target{};
    Error_type error_type{Error_type::no_error};
    VectorXd q, q_target;
    double line_length = 3.0;
    Vector3d starting_point(3, 0, 0);
    double step_size = line_length/nr_steps;
    Frame ff{starting_point};
    q        = VectorXd::Constant(kin.nr_joints, 0.5);
    q_target = q + VectorXd::Constant(kin.nr_joints, step_size);
    error_type = func(&kin, ff, q);
    for(int i=0; i<nr_steps; ++i){
        q = kin.q_out;
        ff.origin += Vector3d(0.0, 0.0, line_length/step_size);

        Frame f_target = kin.f_end;
        error_type = func(&kin, f_target, q);
        if(error_type == Error_type::no_error){
            joint_limit_compliance = kin.check_joint_limits(kin.q_out);
            if(!joint_limit_compliance){ 
                std::cout << "q_init:" << q.transpose() << std::endl;
            }
        }
        else{
            joint_limit_compliance = false; // only relevant for successfull solutions
        }
        if(joint_limit_compliance == false){ std::cout << "FAILED! at step:" << i <<" "; } // either no solution or not within joint limits
        error_types.push_back(error_type);
        joint_limit_compliances.push_back(joint_limit_compliance);
    }
    std::cout << std::endl;
}

int main () {
    std::cout << std::fixed << std::setw( 12 ) << std::setprecision(2);
    std::cout << "=== Tracking tests of kinematic algorithms  ===\n" << 
                 "How well can the inverse kinematic algorithms  \n" <<
                 "follow a \"line\" (evenly spaced points) in    \n" <<
                 "cartesian space." << std::endl;
    std::srand(std::time(0));

    Vector3d axis_z, axis_y;
    axis_y << 0.0, 1.0, 0.0;
    axis_z << 0.0, 0.0, 1.0;
    
    Frame f = Frame();

    double q_min = -170.0/180.0*pi;
    double q_max = 170.0/180.0*pi;

    Joint joint_ellbow = Joint(0, f, axis_y, Joint_type::Rotational, q_min, q_max);
    Joint joint_wrist =  Joint(0, f, axis_z, Joint_type::Rotational, q_min, q_max);
    Joint joint_none =   Joint(0, f, axis_z, Joint_type::None);

    Vector3d length;

    length << 0.0, 0.0, 0.5;
    Frame tip = Frame(length);

    Inertia inertia = Inertia(1.0, length/2, Matrix3d::Identity());
    std::vector<Link> links;
    links.push_back(Link(0, joint_none,   tip, inertia));
    links.push_back(Link(1, joint_wrist,  tip, inertia));
    links.push_back(Link(2, joint_ellbow, tip, inertia));
    links.push_back(Link(3, joint_ellbow, tip, inertia));
    links.push_back(Link(4, joint_wrist,  tip, inertia));
    links.push_back(Link(5, joint_ellbow, tip, inertia));
    links.push_back(Link(6, joint_wrist,  tip, inertia));
    
    Chain chain;
    chain.add_links(links);
    
    Kinematics kin = Kinematics(chain, nr_iter, abs_tol);
    Dynamics dyn   = Dynamics(chain);


    if(mlockall(MCL_CURRENT)){
        cout << "Successfully locked current memory pages into RAM before start of tests." 
        << endl << "(Future memory pages not locked)\n" << endl;
    }
    else{
        cout << "Locking memory pages failed.\n" << endl;
    }

    cout << "==============================" << endl 
         << "Inverse kinematics Tracking"    << endl
         << "==============================" << endl;

    for(int i=0; i<5; ++i){
        track_inverse_kinematics(i, kin, nr_steps);
    }
}
