#pragma once

#include "robo/kinematics_forward.hpp"
#include "robo/kinematics_inverse.hpp"
#include "robo/chain.hpp"

namespace robo {

/**
 * Kinematics' algorithms based on Chain model.
 * Wrapper around ForwardKinematics and InverseKinematics for ease-of-use
*/
class Kinematics {
private:
  ForwardKinematics fk_;
  InverseKinematics ik_;

public:
  Kinematics(const Chain& chain): fk_(chain), ik_(fk_) {}

  int num_joints() const { return fk_.num_joints(); }
  int num_bodies() const { return fk_.num_bodies(); }

  Transform calculate_forward(const VectorXd& q) {
    fk_.calculate_forward(q);
    return fk_.tip();
  }

  Motion calculate_forward_velocity(const VectorXd& q, const VectorXd& dq) {
    return fk_.calculate_forward_velocity(q, dq);
  }

  IK_Result calculate_inverse(
      const Transform& target,
      const VectorXd& q_start = VectorXd()) {
    return ik_.calculate_inverse(target, q_start);
  }

};

} // namespace robo
