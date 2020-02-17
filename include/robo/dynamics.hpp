#pragma once

#include "robo/chain.hpp"
#include "robo/force.hpp"
#include "robo/motion.hpp"
#include "robo/transform.hpp"
#include "robo/typedef.hpp"

#include <vector>

namespace robo {

/**
** Dynamics' algorithms based on Chain model.
*/
class Dynamics{
private:
// Members
  Chain chain_;
  int num_bodies_;
  int num_joints_;
public:
  // Members
  VectorXd               generalized_forces;   /// forces / torques of Joints
  std::vector<Transform> transform_base_to_body; /// from base to current body
  std::vector<Transform> transform_to_child;   /// from current body to child
  std::vector<Motion>    motion_subspace;      /// motion subspace of joint
  std::vector<Motion>    velocities;           /// velocities of each Body
  std::vector<Motion>    accelerations;        /// acceleration of each Body
  std::vector<Force>     wrenches;             /// Force acting on each Body's root
  std::vector<double>    friction_forces;      /// friction acting on each Joint
  std::vector<double>    joint_inertia_forces; /// rotor inertia of each Joint

  // Constructors
  Dynamics() = default;
  Dynamics(const Chain& chain);
  
  // Methods
  int num_joints() const { return num_joints_; }
  int num_bodies() const { return num_bodies_; }

  /**
  ** Finds the forces required to produce a given acceleration in a rigid-body
  ** system.
  ** Recursive Newton-Euler algorithm (based on "Rigid body dynamics algorithms" 
  ** by R.Featherstone, 2007, ISBN 978-0-387-74314-1)
  */
  void calculate_inverse_dynamics(
    const VectorXd& q,
    const VectorXd& dq,
    const VectorXd& ddq,
    const std::vector<Force>& wrenches_extern,
    const Vector3d& gravity=Vector3d(0.0, 0.0, -9.80665));

  void calculate_inverse_dynamics(
      const VectorXd& q,
      const VectorXd& dq,
      const VectorXd& ddq,
      const Vector3d& gravity=Vector3d(0.0, 0.0, -9.80665)){
    calculate_inverse_dynamics(q, dq, ddq,
                               std::vector<Force>(num_bodies()),
                               gravity);
  }

  void calculate_inverse_dynamics(const VectorXd& q,
                                  const VectorXd& dq,
                                  const Vector3d& gravity=Vector3d(0.0, 0.0, -9.80665)){
    calculate_inverse_dynamics(q, dq, VectorXd::Zero(num_joints()),
                               std::vector<Force>(num_bodies()), gravity);
  }

  void calculate_inverse_dynamics(const VectorXd& q,
                                  const Vector3d& gravity=Vector3d(0.0, 0.0, -9.80665)){
    calculate_inverse_dynamics(q,
                               VectorXd::Zero(num_joints()),
                               VectorXd::Zero(num_joints()),
                               std::vector<Force>(num_bodies()), gravity);
  }

  // TODO above functions should not return void but VectorXd generalize_forces
  // TODO introduce encapsulation

  VectorXd calculate_friction_forces(const VectorXd& dq);

};

} // namespace robo
