#define EIGEN_NO_MALLOC
// According to Eigen documentation this macro should not be used in "real-world code".
// However we really dont want any calls to malloc because we are concerned about performance.
// Results in assertion failure when malloc is called by Eigen.

#include "robo/dynamics.hpp"
#include "robo/kinematics.hpp"
#include "robo/util.hpp"
#include "robo/test/test_util.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

int main () {
  using namespace robo;
  constexpr int num_runs = 1e3;
  constexpr auto pi = 3.141592653589793238462643383279502884L;
  constexpr auto BOLD = "\033[1;37m";
  constexpr auto CLEAR = "\033[0m";

  std::cout << std::fixed << std::setprecision(2);
  std::cout << BOLD
            << "=== Timing tests of inverse kinemtics' algorithms  ===\n"
            << CLEAR << std::endl;
  std::srand(0);

  const double q_min = -pi*0.9;
  const double q_max =  pi*0.9;

  const Chain chain = make_test_chain(1.01*q_min, 1.01*q_max);

  Kinematics kin{chain};

  std::vector<double> timings;
  std::vector<robo::IK_Result> ik_results;
  timings.reserve(num_runs);
  ik_results.reserve(num_runs);

  std::cout << BOLD
            << "=============================================\n"
            << " Inverse Kinematics (Random)                 \n"
            << "=============================================\n"
            << CLEAR
            << " Random start point and random target point. \n"
            << "=============================================\n";

  for(int i=0; i<num_runs; ++i){
    const auto q = rand_joint_vector(chain.num_joints, q_min, q_max);
    const auto target = kin.calculate_forward(q);
    util::stop_watch timer;
    ik_results.emplace_back(kin.calculate_inverse(target));
    timings.push_back(timer.elapsed_time()*1e6);
  }

  print_timing_result(timings);
  print_success_rates_IK(ik_results);
  timings.clear();
  ik_results.clear();

  std::cout << '\n'
            << BOLD
            << "=============================================\n"
            << " Inverse Kinematics (Tracking)               \n"
            << "=============================================\n"
            << CLEAR
            << " Move along a line.                          \n"
            << "=============================================\n";

  Transform target(Vector3d(0.35, 0.35, 3.0));
  // TODO this should be tested for different lines
  // TODO but how do we generate lines that are reachable?
  const double line_length = 0.5;
  const double step_size   = 0.001; // do 1mm steps
  const int    num_steps   = static_cast<int>(line_length/step_size);

  // start point for solver
  VectorXd q_init = VectorXd::Constant(kin.num_joints(), 0.2);

  for(int i=0; i<num_steps; ++i){
    util::stop_watch timer;
    ik_results.emplace_back(kin.calculate_inverse(target, q_init));
    timings.push_back(timer.elapsed_time()*1e6);
    q_init = ik_results.back().q;
    target.translation += Vector3d::Constant(step_size);
  }

  print_timing_result(timings);
  print_success_rates_IK(ik_results);
  timings.clear();
  ik_results.clear();
  std::cout << "\nInverse kinematics timing tests PASSED OK\n\n";

}
