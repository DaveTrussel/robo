
#include "robo/kinematics_forward.hpp"

#include "robo/motion.hpp"

#include <ext/alloc_traits.h>
#include <limits>
#include <memory>

#include <iostream>

namespace { // anonymous

robo::Motion
change_reference(
    const robo::Motion& mv,
    const robo::Vector3d& delta){
  return robo::Motion(mv.angular, mv.linear + mv.angular.cross(delta));
}

} // namespace anonymous

robo::ForwardKinematics::ForwardKinematics(const Chain& chain):
    chain_(chain),
    num_bodies_(chain.num_bodies),
    num_joints_(chain.num_joints),
    link_tips_(chain.num_bodies),
    link_roots_(chain.num_bodies),
    jacobian_(Matrix6Xd::Zero(6, chain.num_joints)){}

bool
robo::ForwardKinematics::check_joint_limits(const VectorXd& q) const {
  bool is_within_joint_limits = true;
  int j = 0;
  for(int i = 0; i<num_bodies(); ++i){
    const Body& body = chain_.body(i);
    if(body.has_joint()){
      is_within_joint_limits &= body.joint.check_joint_limits(q[j]);
      ++j;
    }
  }
  return is_within_joint_limits;
}

void
robo::ForwardKinematics::enforce_joint_limits(VectorXd& q) const {
  int j = 0;
  for(int i = 0; i<num_bodies(); ++i){
    const Body& body = chain_.body(i);
    if(body.has_joint()){
      body.joint.enforce_joint_limits(q[j]);
      ++j;
    }
  }
}

void
robo::ForwardKinematics::calculate_forward(const VectorXd& q){
  robo::Transform tip;
  int iter_joint = 0;
  for (int iter_link=0; iter_link<num_bodies(); ++iter_link) {
    link_roots_.at(iter_link) = tip;
    if (chain_.body(iter_link).has_joint()){
      tip = chain_.body(iter_link).transform(q[iter_joint]) * tip;
      ++iter_joint;
    }
    else{ // rigid connection
      tip = chain_.body(iter_link).transform(0.0) * tip;
    }
    link_tips_.at(iter_link) = tip;
  }
}

// TODO this needs heavy testing / debuging for translational joints
robo::Matrix6Xd
robo::ForwardKinematics::calculate_jacobian(const VectorXd& q) {
  using namespace robo;
  calculate_forward(q);
  int iter_joint = 0;
  for (int iter_link=0; iter_link<num_bodies(); ++iter_link){
    if (chain_.body(iter_link).has_joint()) {
      Motion unit_twist =  chain_.body(iter_link).twist(1.0);
      // express in world coordinates (with same reference point)
      // we do not apply Transformation because we want to express the unit
      // twist in world coordinates with reference to the end effector
      Matrix3d to_world = link_roots_.at(iter_link).rotation.transpose();
      Motion world_unit_twist = to_world * unit_twist;
      // compute twist of the end effector motion caused by joint i, expressed in world frame
      Vector3d delta = tip().translation - link_tips_.at(iter_link).translation;
      Motion tip_unit_twist = change_reference(world_unit_twist, delta);
      jacobian_.block<3,1>(0, iter_joint) << tip_unit_twist.linear;
      jacobian_.block<3,1>(3, iter_joint) << tip_unit_twist.angular;
      ++iter_joint;
    }
  }
  return jacobian_;
}

robo::Motion
robo::ForwardKinematics::calculate_forward_velocity(
    const VectorXd& q,
    const robo::VectorXd& dq) {
   Matrix6Xd jacobian = calculate_jacobian(q);
   Vector6d tmp = jacobian * dq;
   return Motion(tmp.tail<3>(), tmp.head<3>());
}

robo::VectorXd
robo::ForwardKinematics::get_upper_joint_limits() const {
  VectorXd ret(num_joints());
  int j = 0;
  for(int i = 0; i<num_bodies(); ++i) {
    const Body& body = chain_.body(i);
    if(body.has_joint()) {
      ret[j] = body.joint.parameters.q_max;
      ++j;
    }
  }
  return ret;
}

robo::VectorXd
robo::ForwardKinematics::get_lower_joint_limits() const {
  VectorXd ret(num_joints());
  int j = 0;
  for(int i = 0; i<num_bodies(); ++i) {
    const Body& body = chain_.body(i);
    if(body.has_joint()) {
      ret[j] = body.joint.parameters.q_min;
      ++j;
    }
  }
  return ret;
}
