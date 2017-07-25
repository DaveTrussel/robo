#define EIGEN_NO_MALLOC
// according to Eigen documentation this macro should not be used in "real-world code"
// however we really dont want any calls to malloc because we are concerned about real-time issues
// results in assertion failure when malloc is called by Eigen.

#include "kinematics.hpp"
#include "dynamics.hpp"

#include <Eigen/Dense>

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <algorithm>
#include <random>
#include <sys/mman.h>

using namespace robo;
using namespace std;
using namespace std::chrono;

constexpr int nr_runs = 1e4;
constexpr int nr_iter = 500;
constexpr auto pi = 3.141592653589793238462643383279502884L;

// helper functions
#define now() high_resolution_clock::now()

VectorXd rand_joint_vector(int size, double min, double max){
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> distri(min, max);
    VectorXd vec(size);
    for(int i=0; i<size; ++i){
        vec[i] = distri(gen);
    }
    return vec;
}

void print_timing_result(vector<double> timings){
    double sum = 0;
    double median = 0;
    size_t len = timings.size();
    sort(timings.begin(), timings.end());
    double max = timings.back();
    double min = timings.front();
    for (double &dt: timings) {
        sum += dt;
    }
    double average = sum / len;
    if(len % 2 == 0){
        median = (timings[len/2] + timings[len/2-1]) / 2.0;
    }
    else{
        median = timings[len/2];
    }
    cout << "Timing results (in microseconds) from " << len << " runs:"<< endl
         << "Average: " << average << endl
         << "Median: " << median << endl
         << "Min: " << min << endl
         << "Max: " << max << endl;
}


int main () {
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

    Inertia inertia = Inertia(1.0, length/2);
    
    Link link_0 = Link(0, joint_none, tip, inertia);
    Link link_1 = Link(1, joint_wrist, tip, inertia);
    Link link_2 = Link(2, joint_ellbow, tip, inertia);
    Link link_3 = Link(3, joint_ellbow, tip, inertia);
    Link link_4 = Link(4, joint_wrist, tip, inertia);
    Link link_5 = Link(5, joint_ellbow, tip, inertia);
    Link link_6 = Link(6, joint_wrist, tip, inertia);
    
    Chain chain;
    chain.add_link(link_0);
    chain.add_link(link_1);
    chain.add_link(link_2);
    chain.add_link(link_3);
    chain.add_link(link_4);
    chain.add_link(link_5);
    chain.add_link(link_6);
    
    Kinematics kin = Kinematics(chain, nr_iter);
    Dynamics dyn = Dynamics(chain);

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

    // Time Inverse Kinematics
    cout << "==============================" << endl 
         << "Inverse kinematics Combined"    << endl
         << "==============================" << endl;
    timings.clear();
    timings.reserve(nr_runs);
    vector<Error_type> error_types;
    error_types.reserve(nr_runs);
    Error_type error_type = Error_type::no_error;
    vector<bool> joint_limit_compliances;
    joint_limit_compliances.reserve(nr_runs);
    bool joint_limit_compliance = false;
    for(int i=0; i<nr_runs; ++i){
        q = rand_joint_vector(chain.nr_joints, q_min, q_max);
        q_init = rand_joint_vector(chain.nr_joints, q_min, q_max);
        kin.joint_to_cartesian(q);
        Frame f_target = kin.f_end;
        tic = now();
        error_type = kin.cartesian_to_joint(f_target, q_init);
        toc = now();
        duration = duration_cast<microseconds>( toc - tic ).count();
        joint_limit_compliance = kin.check_joint_limits(kin.q_out);
        timings.push_back(duration);
        error_types.push_back(error_type);
        joint_limit_compliances.push_back(joint_limit_compliance);
    }
    int nr_no_error = count(error_types.begin(), error_types.end(), Error_type::no_error);
    double successrate = 100.0 * double(nr_no_error)/nr_runs;
    cout << "Successrate: " << successrate << "%" << endl;
    int nr_in_JL = count(joint_limit_compliances.begin(), joint_limit_compliances.end(), true);
    double rate_in_JL = 100.0 * double(nr_in_JL)/nr_runs;
    cout << "Within joint limits: " << rate_in_JL << "%" << endl;
    print_timing_result(timings);
    cout << endl;

    // Time Inverse Kinematics
    cout << "==============================" << endl 
         << "Inverse kinematics LM"          << endl
         << "==============================" << endl;
    timings.clear();
    timings.reserve(nr_runs);
    error_types.clear();
    error_types.reserve(nr_runs);
    error_type = Error_type::no_error;
    for(int i=0; i<nr_runs; ++i){
        q = rand_joint_vector(chain.nr_joints, q_min, q_max);
        q_init = rand_joint_vector(chain.nr_joints, q_min, q_max);
        kin.joint_to_cartesian(q);
        Frame f_target = kin.f_end;
        tic = now();
        error_type = kin.cartesian_to_joint_levenberg(f_target, q_init);
        toc = now();
        duration = duration_cast<microseconds>( toc - tic ).count();
        timings.push_back(duration);
        error_types.push_back(error_type);
    }
    nr_no_error = count(error_types.begin(), error_types.end(), Error_type::no_error);
    successrate = 100.0 * double(nr_no_error)/nr_runs;
    cout << "Successrate: " << successrate << "%" << endl;
    print_timing_result(timings);
    cout << endl;


    // Time Inverse Kinematics
    cout << "==============================" << endl 
         << "Inverse kinematics Sugihara"    << endl
         << "==============================" << endl;
    timings.clear();
    timings.reserve(nr_runs);
    error_types.clear();
    error_types.reserve(nr_runs);
    error_type = Error_type::no_error;
    for(int i=0; i<nr_runs; ++i){
        q = rand_joint_vector(chain.nr_joints, q_min, q_max);
        q_init = rand_joint_vector(chain.nr_joints, q_min, q_max);
        kin.joint_to_cartesian(q);
        Frame f_target = kin.f_end;
        tic = now();
        error_type = kin.cartesian_to_joint_sugihara(f_target, q_init);
        toc = now();
        duration = duration_cast<microseconds>( toc - tic ).count();
        timings.push_back(duration);
        error_types.push_back(error_type);
    }
    nr_no_error = count(error_types.begin(), error_types.end(), Error_type::no_error);
    successrate = 100.0 * double(nr_no_error)/nr_runs;
    cout << "Successrate: " << successrate << "%" << endl;
    print_timing_result(timings);
    cout << endl;

    cout << "==============================" << endl 
         << "Inverse kinematics CCD"         << endl
         << "==============================" << endl;
    timings.clear();
    timings.reserve(nr_runs);
    error_types.clear();
    error_types.reserve(nr_runs);
    error_type = Error_type::no_error;
    for(int i=0; i<nr_runs; ++i){
        q = rand_joint_vector(chain.nr_joints, q_min, q_max);
        q_init = rand_joint_vector(chain.nr_joints, q_min, q_max);
        kin.joint_to_cartesian(q);
        Frame f_target = kin.f_end;
        tic = now();
        error_type = kin.cartesian_to_joint_ccd(f_target, q_init, nr_iter);
        toc = now();
        duration = duration_cast<microseconds>( toc - tic ).count();
        timings.push_back(duration);
        error_types.push_back(error_type);
    }
    nr_no_error = count(error_types.begin(), error_types.end(), Error_type::no_error);
    successrate = 100.0 * double(nr_no_error)/nr_runs;
    cout << "Successrate: " << successrate << "%" << endl;
    print_timing_result(timings);
    cout << endl;


    // Time Inverse Dynamics
    cout << "==============================" << endl 
         << "Inverse dynamics"               << endl
         << "==============================" << endl;
    timings.clear();
    timings.reserve(nr_runs);
    for(int i=0; i<nr_runs; ++i){
        q = rand_joint_vector(chain.nr_joints, q_min, q_max);
        dq = rand_joint_vector(chain.nr_joints, q_min, q_max);
        ddq = rand_joint_vector(chain.nr_joints, q_min, q_max);
        tic = now();
        dyn.calculate_torques(q, dq, ddq);
        toc = now();
        duration = duration_cast<microseconds>( toc - tic ).count();
        timings.push_back(duration);
    }
    print_timing_result(timings);
    cout << endl;
}
