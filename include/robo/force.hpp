#pragma once

#include "robo/typedef.hpp"

namespace robo {

/**
** 6D Force Vector
*/
class Force{
public:
  // Members
  Vector3d torque;
  Vector3d force;

  // Constructors
  explicit Force(const Vector3d& torque_in, const Vector3d& force_in):
   torque(torque_in), force(force_in){};

  Force(): torque(Vector3d(0.0, 0.0, 0.0)), force(Vector3d(0.0, 0.0, 0.0)){};

  // Operators
  Force& operator +=(const Force& other){
    torque += other.torque;
    force += other.force;
    return *this;
  }

  bool operator ==(const Force& other){
    return (torque==other.torque)&&(force==other.force);
  }

  Force operator -()const{
    return Force(-torque, -force);
  }

};

inline Force operator +(const Force& lhs, const Force& rhs){
  return Force(lhs.torque + rhs.torque, lhs.force + rhs.force);
}

inline Force operator -(const Force& lhs, const Force& rhs){
  return Force(lhs.torque - rhs.torque, lhs.force - rhs.force);
}

inline Force operator *(const double& val, const Force& wrench){
  return Force(val*wrench.torque, val*wrench.force);
}

inline Force operator *(const Force& wrench, const double& val){
  return Force(val*wrench.torque, val*wrench.force);
}

}  // namespace robo

inline std::ostream& operator <<(std::ostream& os, const robo::Force& fv)
{
  os << "Torque: " << fv.torque.transpose() << "\t Force: " << fv.force.transpose();
  return os;
}
