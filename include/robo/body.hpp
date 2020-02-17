#pragma once

#include "robo/inertia.hpp"
#include "robo/joint.hpp"

#include <Eigen/Dense>

namespace robo{

/**
** Rigid body (each Body has a joint at a defined position, where the next body
** is attached)
*/
class Body{
public:
  // Members
  Joint joint; /// located at joint_position
  Transform joint_position; /// point of interest on link
  Inertia inertia; /// Reference point is the root of the Body and expressed in the body frame

  // Constructors
  Body() = default;
  Body(const Joint& joint_in,
       const Transform& joint_position_in,
       const Inertia& inertia_in):
    joint(joint_in), joint_position(joint_position_in), inertia(inertia_in){}
  Body(const Joint& joint_in,
       const Transform& joint_position_in): 
    joint(joint_in), joint_position(joint_position_in), inertia(){}
  Body(const Joint& joint_in,
       const Inertia& inertia_):
    joint(joint_in), joint_position(), inertia(inertia_){}
  Body(const Joint& joint_in): 
    joint(joint_in), joint_position(), inertia(){}

  /// transformation from the current body to its child
  Transform transform(const double q) const {
    return joint.transform(q) * joint_position;
  }

  Motion twist(const double dq){
    return joint.twist(dq);
  }

  /// if the body has an active joint
  bool has_joint()const{ return joint.is_active(); }
};

} // namespace robo
