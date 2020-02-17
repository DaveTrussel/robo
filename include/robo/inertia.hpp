#pragma once

#include "robo/force.hpp"
#include "robo/motion.hpp"
#include "robo/transform.hpp"


namespace robo{

/**
** Mass, center of mass and rotational inertia fully describe the inertia of a 
** rigid body.
** Inertia is a property of a body expressed at a specified point.
** All values must be given with respect to the specified frame.
*/
class Inertia{
public:
  // Members
  double mass;
  Vector3d h; /// center of mass times mass TODO find out how this is called (with respect to reference point)
  Matrix3d rot_inertia; /// angular inertia (with respect to reference point)

  // Constructors
  explicit Inertia(const double& mass_in,const Vector3d& com_in,const Matrix3d& rot_in): 
    mass(mass_in), h(com_in*mass_in), rot_inertia(rot_in){};

  explicit Inertia(const double& mass_in,const Vector3d& com_in): 
    mass(mass_in), h(com_in*mass_in), rot_inertia(Matrix3d::Zero()){};

  Inertia(): mass(0.0), h(Vector3d::Zero()), rot_inertia(Matrix3d::Zero()){};

  Vector3d center_of_mass() const {
    if(mass == 0.0){ return Vector3d::Zero(); }
    else           { return h/mass;           }
  }

  /**
  ** Both the inertia and the motion vector MUST be expressed in the same
  ** coordinate system and with respect to the same point.
  */
  Force operator *(const Motion& mv) const {
    return Force(rot_inertia * mv.angular + h.cross(mv.linear),
                 mass * mv.linear - h.cross(mv.angular));
  }

  Inertia& operator +=(const Inertia& other){
    mass        += other.mass;
    h           += other.h;
    rot_inertia += other.rot_inertia;
    return *this;
  }
};

inline Inertia operator +(const Inertia& lhs, const Inertia& rhs){
  return Inertia(lhs.mass + rhs.mass,
                 lhs.center_of_mass() + rhs.center_of_mass(),
                 lhs.rot_inertia + rhs.rot_inertia);
}

} // namespace robo

