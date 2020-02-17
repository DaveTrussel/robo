#pragma once

#include "robo/body.hpp"

#include <vector>

#include <cassert>

namespace robo{

/**
** Kinematic & Dynamic model of rigid bodies. Each Body has a joint at its tip.
*/
class Chain{
private:
  std::vector<Body> bodies_;

public:
  // Members
  int num_joints;
  int num_bodies;

  Body body(int nr) const { return bodies_.at(nr); }

  std::vector<Body> bodies() { return bodies_; }

  // Constructors
  Chain():bodies_(), num_joints(0), num_bodies(0){};

  // Named Constructors
  Chain(const std::vector<Body>& bodies):bodies_(), num_joints(0), num_bodies(0){
    add_bodies(bodies);
  }

  // Methods

  /// adds a body at the end of the chain
  void add_body(const Body& body){
    bodies_.push_back(body);
    ++num_bodies;
    if(body.has_joint()){
      ++num_joints;
    }
  }

  /// adds bodies from bottom to top
  void add_bodies(const std::vector<Body>& bodies){
    for(const auto& body : bodies){
      add_body(body);
    }
  }

  ///
  /// Adds the rotor inertia of each parent's joint to the respective body.
  /// This neglects gyroscopic effects, which are small when the gear ratio
  /// is sufficiently high
  ///
  // TODO this should be done in the constructor (get rid of default constructor)
  void add_rotor_inertias_to_body_inertias(){
    // TODO this needs a rework for translational joints
    for(int iter_body=1; iter_body<num_bodies; ++iter_body){
      // inertia of the parent joint needs to be added to the current body
      const Joint& joint_parent = body(iter_body-1).joint;
      double theta_m = joint_parent.parameters.rotor_inertia_equivalent;
      Vector3d diag_ele = joint_parent.axis * theta_m;
      Inertia rotor_inertia{0, Vector3d(0, 0, 0), diag_ele.asDiagonal()};
      bodies_[iter_body].inertia += rotor_inertia;
    }
  }
};

} // namespace robo

inline std::ostream& operator <<(std::ostream& os, const robo::Chain& ch)
{
  os << "Chain model: num_bodies: " << ch.num_bodies << ", num_joints" <<
        ch.num_joints << "\n";
  return os;
}
