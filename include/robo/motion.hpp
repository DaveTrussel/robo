#pragma once

#include "robo/force.hpp"

namespace robo {

/**
** 6D Spatial Motion Vector
*/
class Motion{
public:
  // Members
  Vector3d angular;
  Vector3d linear;

  // Constructors
  explicit Motion(const Vector3d& ang, const Vector3d& lin):  angular(ang), linear(lin){};
  
  // TODO can this be confused too easily with a pure angular motion vector? Named constructor?
  explicit Motion(const Vector3d& lin): angular(Vector3d(0.0, 0.0, 0.0)), linear(lin){};
  
  Motion(): angular(Vector3d(0.0, 0.0, 0.0)), linear(Vector3d(0.0, 0.0, 0.0)){};

  // Methods
  Motion cross(const Motion& other){
    return Motion(angular.cross(other.angular),
                  angular.cross(other.linear) + linear.cross(other.angular));
  }

  Force cross(const Force& fv){
    return Force(angular.cross(fv.torque) + linear.cross(fv.force),
                 angular.cross(fv.force));
  }

  double dot(const Force& fv){
    return angular.dot(fv.torque) + linear.dot(fv.force);
  }

  // Operators
  bool operator ==(const Motion& other){
    return (linear==other.linear)&&(angular==other.angular);
  }

  Motion operator -()const{
    return Motion(-angular, -linear);
  }
};

inline Motion operator +(const Motion& lhs, const Motion& rhs){
  return Motion(lhs.angular+rhs.angular, lhs.linear+rhs.linear);
}

inline Motion operator -(const Motion& lhs, const Motion& rhs){
  return Motion(lhs.angular-rhs.angular, lhs.linear-rhs.linear);
}

inline Motion operator *(const double& val, const Motion& twist){
  return Motion(twist.angular*val, twist.linear*val);
}

inline Motion operator *(const Motion& mv, const double& val){
  return Motion(mv.angular*val, mv.linear*val);
}

inline Motion operator *(const Matrix3d& rot, const Motion& mv){
  return Motion(rot * mv.angular, rot * mv.linear);
}

} // namespace robo

inline std::ostream& operator <<(std::ostream& os, const robo::Motion& mv)
{
  os << "Angular: " << mv.angular.transpose() << "\t Linear: " << mv.linear.transpose();
  return os;
}
