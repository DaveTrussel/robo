#define EIGEN_NO_MALLOC
// According to Eigen documentation this macro should not be used in "real-world code".
// However we really dont want any calls to malloc because we are concerned about performance.
// Results in assertion failure when malloc is called by Eigen.

#include "robo/dynamics.hpp"
#include "robo/kinematics_forward.hpp"
#include "robo/util.hpp"
#include "robo/test/test_util.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

int main () {
  using namespace robo;

  constexpr int num_runs = 2e3;
  constexpr auto pi = 3.141592653589793238462643383279502884L;
  constexpr auto BOLD = "\033[1;37m";
  constexpr auto CLEAR = "\033[0m";

  std::cout << std::fixed << std::setprecision(2);
  std::cout << BOLD
            << "=== Timing tests of kinemtics' dynamics' algorithms  ===\n"
            << CLEAR << std::endl;
  std::srand(std::time(0));

  double q_min = -pi*0.9;
  double q_max =  pi*0.9;

  double gg   = 9.8;
  Vector3d grav;
  grav   << 0, 0, -gg;

  Chain chain = make_test_chain(q_min, q_max);

  ForwardKinematics kin{chain};
  Dynamics   dyn{chain};

  VectorXd q(chain.num_bodies), dq(chain.num_bodies), ddq(chain.num_bodies);

  std::vector<double> timings;
  timings.reserve(num_runs);

  // Time Forward Kinematics
  std::cout << BOLD
            << "=============================================\n"
            << "  Forward Kinematics                         \n"
            << "=============================================\n"
            << CLEAR;
  for(int i=0; i<num_runs; ++i){
    q   = rand_joint_vector(chain.num_joints, q_min, q_max);
    util::stop_watch timer;
    kin.calculate_forward(q);
    timings.push_back(timer.elapsed_time()*1e6);
  }
  print_timing_result(timings);
  timings.clear();
  std::cout << "\nForward kinematics timing tests PASSED OK\n\n";


  // Time Jacobian
  std::cout << BOLD
            << "=============================================\n"
            << "  Jacobian                                   \n"
            << "=============================================\n"
            << CLEAR;
  for(int i=0; i<num_runs; ++i){
    q   = rand_joint_vector(chain.num_joints, q_min, q_max);
    util::stop_watch timer;
    kin.calculate_jacobian(q);
    timings.push_back(timer.elapsed_time()*1e6);
  }
  print_timing_result(timings);
  timings.clear();
  std::cout << "\nJacobian timing tests PASSED OK\n\n";

  // Time Inverse Dynamics
  std::cout << BOLD
            << "=============================================\n"
            << "  Inverse Dynamics (Recursive Newton-Euler)  \n"
            << "=============================================\n"
            << CLEAR;
  for(int i=0; i<num_runs; ++i){
    q   = rand_joint_vector(chain.num_joints, q_min, q_max);
    dq  = rand_joint_vector(chain.num_joints, q_min, q_max);
    ddq = rand_joint_vector(chain.num_joints, q_min, q_max);
    util::stop_watch timer;
    dyn.calculate_inverse_dynamics(q, dq, ddq, grav);
    timings.push_back(timer.elapsed_time()*1e6);
  }
  print_timing_result(timings);
  timings.clear();
  std::cout << "\nInverse Dynamics timing tests PASSED OK\n\n";
}
