/**
@file kinematics_inverse.hpp

Inverse kinematics

This file provides functions to compute joint angles based on
positions in the workspace space. It is implemented using a numerical non-linear
optimization solver (NLopt).
*/


#pragma once

#include "robo/kinematics_forward.hpp"

#include <iosfwd>

namespace robo{

enum class Error { None, MaxIter };

// TODO use string instead of Error enum
struct IK_Result {
  VectorXd q; // joint positions [rad]
  Error error;
};


/**
 *  Inverse Kinematics
 *  Solving the inverse kinematics is the problem of finding joint positions
 *  for a given transformation in world coordinates e.g.:
 *  q = solve_inverse_kinematics(translation, rotation)
 *  Here we formulate the problem as an optimization problem of minimizing the
 *  (weighted) error between target and found solution in cartesian space.
 *  The joint limits impose boundary constraints.
 *  The problem is solved using numerical solvers.
 *  Since the solutions are not analytical the solver can fail depending on the
 *  inital conditions.
*/
class InverseKinematics{
private:
  ForwardKinematics fwd_;
  /// Desired tranform (translation [m], orientation) in world frame
  Transform target_;
  /// weights for the different dimensions [x, y, z, x-rot, y-rot, z-rot]
  Vector6d  weights_cartesian_ = Vector6d::Ones();
  /// desired max. value of error function
  double    abs_tol_           = 1e-5;

  /// Sets the target (needed because NLopt requires special function signature)
  void set_target(const Transform& target){ target_ = target; }

  /// The error is weighted by these factors [x, y, z, x-rot, y-rot, z-rot]
  void set_weights(const Vector6d& w){ weights_cartesian_ = w; }

  /// The solver stops if the error is smaller than this value.
  void set_absolute_error_tolerance(const double abs_tol){ abs_tol_ = abs_tol; }

/**
 * @todo Currently unused because the SQP solver seems to be better, decide if
 * we get rid of it.
 *
 *  Tries to solve the inverse kinematics for a given starting point <q_start>
 *  Damped and weighted least squares algorithm
 *  Based on:
 *  "Solvability-unconcerned Inverse Kinematics based on Levenberg-Marquardt
 *  method with Robuts Damping"
 *  by Tomomichi Sugihara, 2009, International Conference on Humanoid Robots.
 *  Opposed to the paper above, this algorithm does enforce joint_limits.
 *  (Leads to quite low successrate.)
 */
  IK_Result calculate_inverse_sugihara(
      const Transform& target,
      const VectorXd& q_start,
      const double max_step_size=0.25); // [m]/[rad]

/**
 *  Tries to solve the inverse kinematics for a given starting point <q_start>
 *  Use a Sequential Quadratic Program solver to solve the IK.
 *  The NLopt library is used for the solvers https://nlopt.readthedocs.io
 *  If no starting point is provided a global solver is first used to find a
 *  starting point for the local SQP solver.
*/
  IK_Result calculate_inverse_SQP(
      // target needs to be specified with set_target() because of NLopt API
      const VectorXd& q_start = VectorXd(),
      const int max_iter      = 500);

  /// Calculates the weighted norm of the error between the two transforms.
  double calculate_weighted_residual(const Transform& t1, const Transform& t2);

  /// Check if a solution is within the specified tolerance of the target
  bool check(const VectorXd& q);

public:
  /**
   * Create InverseKinematics solver instance
  */
  explicit InverseKinematics(const ForwardKinematics& fwd): fwd_(fwd) {};

  /**
   * Calculates the weighted norm of the error between target and actual value.
   * Returns a scalar that represents the error betweent he set target and
   * the postion of the end effector for the given joint positions q
   *
   * Needs to be public because of NLopt objective function
  */
  double calculate_weighted_residual(const VectorXd& q);

  /// Public API wrapper for caluclate_inverse_SQP
  /// The user does not get to choose the algorithm.
  IK_Result calculate_inverse(
      const Transform& target,
      const VectorXd& q_start = VectorXd());

  // TODO implement FABRIK algorithm and compare to Sugihara, SQP

};

} // namespace robo

std::ostream& operator <<(std::ostream& os, const robo::Error& res);
