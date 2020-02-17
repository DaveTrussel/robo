#include "robo/kinematics_inverse.hpp"
#include "robo/util.hpp"

#include <nlopt.hpp>

#include <algorithm>
#include <iostream>
#include <stdexcept>


namespace { // anonymous
// gew review: I suggest formatting according the Google C++ style:
// void f() {
//   for (int i = 0; i < N; ++i) {
//     // ...
//   }

// clamp norm of vector
[[gnu::unused]]
void clamp_mag(robo::Vector6d& vec, double max_norm) {
  // gew review: const double norm = ...
  // gew review: Handle division by zero
  double norm = vec.norm();
  if(norm > max_norm){ vec = max_norm * vec / norm; }
}

// elementwise clamp mag
void limit_step_size(robo::VectorXd& delta, double max_step) {
  for(int i=0; i<delta.size(); ++i){
    delta[i] = std::clamp(delta[i], -max_step, max_step);
  }
}

/**
 * Wrapper function
 * NLopt needs the function to be optimized to have this signature.
 * `x` are the optimization variables and `gradient` is empty if the algorithm
 * used is not gradient-based.
*/
double
objective_wrapper(
    const std::vector<double>& x,
    std::vector<double>& gradient,
    void* inverse_kinematics) {
  using namespace robo;
  InverseKinematics* ik = static_cast<InverseKinematics*>(inverse_kinematics);
  // TODO find best value for h and document why we use it.
  constexpr double h = 2*std::numeric_limits<float>::epsilon(); // step size
  VectorXd q = eigen_view(x);
  const double val = ik->calculate_weighted_residual(q);

  // only set the gradient if the nlopt-algorithm is gradient based. If gradient
  // is not empty it has the same size as the optimization variables.
  if (not gradient.empty()) {
    // calculate the centered gradient
    // TODO maybe we are faster by only using the forward/backward gradient
    for (auto i = 0; i < q.size(); ++i) {
      const double tmp = q[i];
      q[i] = tmp + h;
      const double val_plus = ik->calculate_weighted_residual(q);
      q[i] = tmp - h;
      const double val_minus = ik->calculate_weighted_residual(q);
      q[i] = tmp; // restore original value
      gradient[i] = (val_plus - val_minus)/(2.0*h);
    }
  }
  return val;
}

std::ostream& operator <<(std::ostream& os, const nlopt::result& res) {
  switch (res) {
    case nlopt::FAILURE:          return os << "failure";
    case nlopt::INVALID_ARGS:     return os << "invalid arguments";
    case nlopt::OUT_OF_MEMORY:    return os << "out of memory";
    case nlopt::ROUNDOFF_LIMITED: return os << "roundoff limited";
    case nlopt::FORCED_STOP:      return os << "forced stop";
    case nlopt::SUCCESS:          return os << "success";
    case nlopt::STOPVAL_REACHED:  return os << "stop value reached";
    case nlopt::FTOL_REACHED:     return os << "function tolerance reached";
    case nlopt::XTOL_REACHED:     return os << "parameter tolerance reached";
    case nlopt::MAXEVAL_REACHED:  return os << "max. evaluations reached";
    case nlopt::MAXTIME_REACHED:  return os << "max. time reached";
    default: return os << "unsupported result type";
  }
}

[[gnu::unused]]
std::string to_string(const nlopt::result& res) {
  std::ostringstream oss;
  oss << res;
  return oss.str();
}


} // namespace anonymous

robo::IK_Result
robo::InverseKinematics::calculate_inverse(
    const Transform& target,
    const VectorXd& q_start) {
  set_target(target);
  return calculate_inverse_SQP(q_start);
}

robo::IK_Result
robo::InverseKinematics::calculate_inverse_SQP(
    const VectorXd& q_start, const int max_iter) {

  /// global solver to find a starting point if none is provided
  nlopt::opt solver_global = nlopt::opt(nlopt::GN_DIRECT_L, fwd_.num_joints());
  /// local solver to find the optimal joint position
  nlopt::opt solver_local  = nlopt::opt(nlopt::LD_SLSQP   , fwd_.num_joints());

  const std::vector<double> q_min =
    from_eigen<double>(fwd_.get_lower_joint_limits());
  const std::vector<double> q_max =
    from_eigen<double>(fwd_.get_upper_joint_limits());

  solver_global.set_min_objective(::objective_wrapper, this);
  solver_global.set_lower_bounds(q_min);
  solver_global.set_upper_bounds(q_max);
  // solver stops if value smaller than stopval is reached
  solver_global.set_stopval(5e-3); // we just want a rough starting point
  solver_global.set_maxeval(max_iter); // TODO this needs tweaking

  solver_local.set_min_objective(::objective_wrapper, this);
  solver_local.set_lower_bounds(q_min);
  solver_local.set_upper_bounds(q_max);
  solver_local.set_stopval(abs_tol_); // desired value of objective function
  solver_local.set_maxeval(max_iter);

  std::vector<double> q(fwd_.num_joints(), 0.0);
  double result_value = std::numeric_limits<double>::max();
  nlopt::result ans;
  if (0 == q_start.size()) {
    // no initial guess was provided
    // so we search for one globally
    try {
      ans = solver_global.optimize(q, result_value);
    }
    catch(const std::runtime_error&) { /* TODO handle erros. However we do not want to
                                    stop if global solver fails still try with
                                    local solver. */ }
  }
  else {
    q = from_eigen<double>(q_start);
  }
  // we have a rough estimate and improve it with a local solver
  try {
    ans = solver_local.optimize(q, result_value); }
  catch (const nlopt::roundoff_limited& e) {}
    //std::cout << "DEBUG: Roundoff limited: " << e.what() << '\n'; }
    // TODO
    // algorithm could not progress further due to round off errors
    // result might still be usefull. what do we do?
  catch (const std::invalid_argument& e) {
    // std::cout << "DEBUG: Invalid argument: " << e.what() << '\n';
    // TODO: Throw or return error message
  }
    // TODO sometimes there seems to be a numerical problem with joint limits
    // e.g. 2.834 <= 2.834 throws an error

  IK_Result ret;
  ret.q = eigen_copy(q);
  if ((nlopt::SUCCESS == ans) or (nlopt::STOPVAL_REACHED == ans)) {
    ret.error = Error::None;
  } else if (nlopt::MAXEVAL_REACHED == ans) {
    // TODO: Maybe return a string instead of error code?
    // value might still be useful, just not accurate enough
    ret.error = Error::MaxIter;
  } else {
    ret.error = Error::MaxIter;
    // TODO properly handle other cases
    // throw std::runtime_error("INVERSE KINEMATICS: unexpected solver result.");
  }
  // check because we do not trust solver blindly.
  if (not check(ret.q)){
    ret.error = Error::MaxIter;
  }
  return ret;
}

robo::IK_Result
robo::InverseKinematics::calculate_inverse_sugihara(
    const Transform& target,
    const VectorXd& q_start,
    const double max_step_size) {

  VectorXd q = q_start;
  for (int i=0; i<100; ++i) {
    // gew review: Better: const auto jacobian = ..., don't declare outside the loop.
    // Same for most/all others, see above
    const Matrix6Xd jacobian = fwd_.calculate_jacobian(q);
    // calculating the jacobian also calculates the forward kinematics
    const Motion tmp = target - fwd_.tip(); // how far are we away
    Vector6d residual;
    residual << tmp.linear, tmp.angular;
    const double residual_norm = weights_cartesian_.dot(residual);
    // check if we are close enough and within joint limits
    if(residual_norm < abs_tol_ and fwd_.check_joint_limits(q)){
      return IK_Result{q, Error::None};
    }
    const VectorXd b =
      jacobian.transpose() * weights_cartesian_.asDiagonal() * residual;
    MatrixXd A =
      jacobian.transpose() * weights_cartesian_.asDiagonal() * jacobian;
    // damp to avoid instabilities near singularities
    const double residual_norm_squared = residual_norm * residual_norm;
    VectorXd factor_damp =
      VectorXd::Constant(fwd_.num_joints(), residual_norm_squared + 1e-10);
    A.diagonal() += factor_damp;
    // solve the linear system
    VectorXd delta_q = A.colPivHouseholderQr().solve(b);
    ::limit_step_size(delta_q, max_step_size); // avoid "wild" jumps
    q += delta_q;
    fwd_.enforce_joint_limits(q); // stay within boundaries
  }
  // we did not reach the target, return the position anyway
  return IK_Result{q, Error::MaxIter};
}

double
robo::InverseKinematics::calculate_weighted_residual(
    const Transform& t1,
    const Transform& t2) {
  const Motion residual = t1 - t2;
  return weights_cartesian_.head<3>().cwiseProduct(residual.linear ).norm() +
         weights_cartesian_.tail<3>().cwiseProduct(residual.angular).norm();
}

double
robo::InverseKinematics::calculate_weighted_residual(const VectorXd& q) {
  fwd_.calculate_forward(q);
  return calculate_weighted_residual(target_, fwd_.tip());
}

bool
robo::InverseKinematics::check(
    const VectorXd& q) {
  const double error = calculate_weighted_residual(q);
  return abs_tol_ > error;
}

std::ostream& operator <<(std::ostream& os, const robo::Error& res) {
  switch (res) {
    case robo::Error::None   : return os << "OK";
    case robo::Error::MaxIter: return os << "Error::MaxIter";
  }
}
