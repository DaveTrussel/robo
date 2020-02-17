#pragma once

#include "robo/chain.hpp"
#include "robo/transform.hpp"
#include "robo/typedef.hpp"

#include <vector>

namespace robo{

using TransformVector = std::vector<Transform>;

/**
** Kinematics' algorithms based on Chain model.
*/
class ForwardKinematics{
private:
  Chain chain_;
  int num_bodies_;
  int num_joints_;

  TransformVector link_tips_; /// frames at the root of each link
  TransformVector link_roots_;

  Matrix6Xd jacobian_;


public:
  explicit ForwardKinematics(const Chain& chain);

  int num_joints() const { return num_joints_; }
  int num_bodies() const { return num_bodies_; }
  /// Transform from world frame to end of the kinematic chain
  Transform       tip       () const { return link_tips_.at(num_bodies_-1); }
  /// Transform from world frame to the joint positions
  TransformVector link_tips () const { return link_tips_;  }
  /// Transform from world frame to the link root frames
  TransformVector link_roots() const { return link_roots_; }

  /// Returns if all joints are within joint limits
  bool check_joint_limits(const VectorXd& q) const;

  /// Set the joints that violate the limits to the closer limit.
  void enforce_joint_limits(VectorXd& q) const;

  VectorXd get_upper_joint_limits() const;

  VectorXd get_lower_joint_limits() const;

  /// Returns all the link tips for the given joint positions
  void calculate_forward(const VectorXd& q);

  /// Compute the matrix that represents the motion at the end effector caused
  /// by each joint, expressed in world coordinates
  Matrix6Xd calculate_jacobian(const VectorXd& q);

  /// Calculates the velocity in carthesian coordinates
  Motion calculate_forward_velocity(const VectorXd& q, const VectorXd& dq);
};

} // namespace robo

