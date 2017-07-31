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

constexpr int nr_runs = 1e3;
constexpr int nr_iter = 1000;
constexpr auto pi = 3.141592653589793238462643383279502884L;

void time_inverse_kinematics(int method, Kinematics kin, int nr_runs){
/*
 * Take a random start point and a random target point (all reachable) and check how the different
 * nummerical IK solvers compare to each other.
 * TODO instead of choosing completely random points, the tracking behaviour could be of interest. So
 * check instead how well a IK algorithm would follow a reference path given in cartesian space.
 */
    vector<double>     timings;
    vector<Error_type> error_types;
    vector<bool>       joint_limit_compliances;
    timings.reserve(nr_runs);
    error_types.reserve(nr_runs);
    joint_limit_compliances.reserve(nr_runs);
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
    auto tic = now();
    auto toc = now();
    auto duration = duration_cast<microseconds>( toc - tic ).count();
    bool joint_limit_compliance{false};
    Frame f_target{};
    Error_type error_type{Error_type::no_error};
    VectorXd q, q_init;
    double q_min = -165.0/180.0*pi; // TODO make this more generic
    double q_max = 165.0/180.0*pi;
    for(int i=0; i<nr_runs; ++i){
        q = rand_joint_vector(kin.nr_joints, q_min, q_max); // TODO make this more generic
        q_init = rand_joint_vector(kin.nr_joints, q_min, q_max);
        kin.joint_to_cartesian(q);
        Frame f_target = kin.f_end;
        tic = now();
        error_type = func(&kin, f_target, q_init);
        toc = now();
        duration = duration_cast<microseconds>( toc - tic ).count();
        joint_limit_compliance = kin.check_joint_limits(kin.q_out);
        timings.push_back(duration);
        error_types.push_back(error_type);
        joint_limit_compliances.push_back(joint_limit_compliance);
    }
    print_timing_result(timings);
    print_success_rates_IK(error_types);
    print_joint_limits_compliance(joint_limit_compliances);
    std::cout << std::endl;
}

int main () {
    std::cout << std::fixed << std::setw( 12 ) << std::setprecision(2);
    std::cout << "=== Timing tests of kinematic and dynamic algorithms  ===\n" << 
                 "" << std::endl;
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
    
    Kinematics kin = Kinematics(chain, nr_iter);
    Dynamics dyn   = Dynamics(chain);

    VectorXd q(chain.nr_joints);
    VectorXd dq(chain.nr_joints);
    VectorXd ddq(chain.nr_joints);
    VectorXd q_init(chain.nr_joints);

    q      = rand_joint_vector(chain.nr_joints, q_min, q_max);
    dq     = rand_joint_vector(chain.nr_joints, q_min, q_max);
    ddq    = rand_joint_vector(chain.nr_joints, q_min, q_max);
    q_init = rand_joint_vector(chain.nr_joints, q_min, q_max);

    auto tic = now();
    kin.joint_to_cartesian(q);
    auto toc = now();
    auto duration = duration_cast<microseconds>( toc - tic ).count();

    vector<double> timings;
    timings.reserve(nr_runs);

    if(mlockall(MCL_CURRENT)){
        cout << "Successfully locked current memory pages into RAM before start of tests." 
        << endl << "(Future memory pages not locked)\n" << endl;
    }
    else{
        cout << "Locking memory pages failed.\n" << endl;
    }


    // Time Forward Kinematics
    cout << "==============================" << endl 
         << "Forward kinematics"             << endl
         << "==============================" << endl;
    for(int i=0; i<nr_runs; ++i){
        q = rand_joint_vector(chain.nr_joints, q_min, q_max);
        tic = now();
        kin.joint_to_cartesian(q);
        toc = now();
        duration = duration_cast<microseconds>( toc - tic ).count();
        timings.push_back(duration);
    }
    print_timing_result(timings);
    cout << endl;
    cout << "==============================" << endl 
         << "Inverse kinematics"             << endl
         << "==============================" << endl;

    for(int i=0; i<5; ++i){
        time_inverse_kinematics(i, kin, nr_runs);
    }
}
