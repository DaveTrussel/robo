#pragma once

#include "robo/friction.hpp"
#include "robo/rotation.hpp"
#include "robo/transform.hpp"

#include <cassert>
#include <cmath>


namespace robo {

/**
** Describes possible joint types
*/
enum class Joint_type { None, Rotational, Translational };


/**
** Calculates joint quantities depending on its type
*/
class Joint {
public:
  // Members
  Joint_type type = Joint_type::None;
  Vector3d   axis = Vector3d(0, 0, 1);

  FrictionModel friction;

  struct {
    // inertia that is spinning at rotor speed (converted with gear ratio)
    double rotor_inertia_equivalent = 0.0;
    double q_min                    = std::numeric_limits<double>::lowest();
    double q_max                    = std::numeric_limits<double>::max();
  } parameters;

  // Constructors
  Joint(const Vector3d axis_in=Vector3d(0,0,1),
    const Joint_type type_in=Joint_type::Rotational,
    const double q_min_in=std::numeric_limits<double>::lowest(),
    const double q_max_in=std::numeric_limits<double>::max()):
  type(type_in), axis(axis_in.normalized())
  {
    parameters.q_min = q_min_in;
    parameters.q_max = q_max_in;
  }

  // Methods

  /**
  ** Calculates the joint transformation represented in the joint root frame
  */
  Transform transform(double q)const{
    if(type == Joint_type::Rotational){
      return Transform(rotate_around_axis(axis, q), Vector3d::Zero());
    }
    if(type == Joint_type::Translational){
      return Transform(Matrix3d::Identity(), q*axis);
    }
    else{
      return Transform();
    }
  }

  /**
  ** Calculates the 6D velocity of the joint represented in the joint root frame
  */
  Motion twist(double dq)const{
    Vector3d speed_lin = Vector3d::Zero();
    Vector3d speed_rot = Vector3d::Zero();
    if(type == Joint_type::Rotational){
      speed_rot << dq*axis;
    }
    if(type == Joint_type::Translational){
      speed_lin << dq*axis;
    }
    return Motion(speed_rot, speed_lin);
  }

  double calculate_friction_force(const double dq) const {
    return friction.calculate_friction_force(dq);
  }

  double calculate_inertia_force(const double ddq) const {
    return parameters.rotor_inertia_equivalent * ddq;
  }

  /**
  ** Returns the subspace of motion for the given joint type represented in the
  ** joint root frame
  */
  Motion motion_subspace() const {
    return twist(1.0);
  }

  void set_friction_coefficients(const FrictionModel& in){
    friction = in;
  }

  void set_joint_limits(const double q_min, const double q_max){
    parameters.q_min = q_min;
    parameters.q_max = q_max;
  }

  bool check_joint_limits(const double q) const {
    return (parameters.q_min <= q) and (q <= parameters.q_max);
  }

  void enforce_joint_limits(double& q) const {
    if(q < parameters.q_min){ q = parameters.q_min; }
    if(q > parameters.q_max){ q = parameters.q_max; }
  }

  ///
  /// Sets the rotor inertia, taking the gear ratio into account
  /// rotor_inertia_original = inertia as seen antriebsseitig
  /// rotor_inertia_equivalent = inertia as seen abtriebsseitig
  /// this common 'trick' neglects gyroscopic effects of the motors (small)
  ///
  void set_rotor_inertia_equivalent(const double transmission_ratio,
                                    const double rotor_inertia_original){
    assert(transmission_ratio >= 1); // DEBUG for PRob2
    parameters.rotor_inertia_equivalent = rotor_inertia_original * transmission_ratio * transmission_ratio;
  }

  bool is_active()const{
    return type != Joint_type::None;
  }
};

} // namespace robo
