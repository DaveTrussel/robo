#pragma once

#include "robo/force.hpp"
#include "robo/motion.hpp"
#include "robo/util.hpp"

#include <Eigen/Dense>

namespace robo {

/**
** Pluecker Transformation (compact representation of translation and rotation).
*/
class Transform{
public:
  // Members
  Matrix3d rotation;
  Vector3d translation;

  // Constructors
  Transform(const Matrix3d& rot, const Vector3d& vec):
    rotation(rot), translation(vec){};

  Transform(const Vector3d& vec):
    rotation(Matrix3d::Identity()), translation(vec){};

  Transform(const Matrix3d& mat):
    rotation(mat), translation(){};

  Transform():
    rotation(Matrix3d::Identity()), translation(Vector3d::Zero()){};

  // Member functions
  Transform inverse(){
    return Transform(rotation.transpose(), -rotation*translation);
  }

  /**
  ** Get the rotation as nautical angles (roll, pitch, yaw) [deg].
  ** This might not be very accurate, prefer rotation matrices instead
  */
  Vector3d nautical_angles()const{
    // transpose comes from definition of rotations (active vs. passive)
    // between featherstone and eigen
    Vector3d radians = rotation.transpose().eulerAngles(0,1,2);
    return math::rad_to_deg(radians);
  }

  /**
  ** Applies the transformation to a Motion.
  ** A motion vector is a property of a body and a transform is only valid to a
  ** frame on the same body .
  */
  Motion apply(const Motion& mv){
    return Motion(rotation * mv.angular,
                  rotation * (mv.linear - translation.cross(mv.angular)));
  }

  /**
  ** Applies the inverse transformation to a Motion..
  */
  Motion apply_inverse(const Motion& mv){
    Eigen::Transpose<Matrix3d> rot_inv = rotation.transpose();
    return Motion(rot_inv * mv.angular,
                  rot_inv * mv.linear + translation.cross(rot_inv * mv.angular));
  }

  /**
  ** Applies the transformation to a Force.
  ** A force vector is a property of a body and a transform is only valid to a
  ** frame on the same body
  */
  Force apply(const Force& fv){
    return Force(rotation * (fv.torque - translation.cross(fv.force)),
                 rotation * fv.force);
  }

  /**
  ** Applies the inverse transformation to a Force.
  */
  Force apply_inverse(const Force& fv){
    Eigen::Transpose<Matrix3d> rot_inv = rotation.transpose();
    return Force(rot_inv * fv.torque + translation.cross(rot_inv * fv.force),
                 rot_inv * fv.force);
  }

  // Operators
  bool operator ==(const Transform& other){
    return (translation==other.translation)&&(rotation==other.rotation);
  }
};

/*
** Chains two transformations (right first).
*/
inline Transform operator *(const Transform& left, const Transform& right){
  return Transform(left.rotation * right.rotation,
                   right.translation + right.rotation.transpose() * left.translation);
}

/**
** Calculates the difference of two transforms (position and rotation)
*/
inline Motion operator -(const Transform& left, const Transform& right){
  Motion delta;
  delta.linear = left.translation - right.translation;
  Matrix3d rotinv_rot;
  rotinv_rot << left.rotation.transpose() * right.rotation; // transp == inv, for rotations
  Eigen::AngleAxisd angle_axis(rotinv_rot);
  delta.angular = angle_axis.axis() * angle_axis.angle();
  return delta;
}

} // namespace robo

inline std::ostream& operator <<(std::ostream& os, const robo::Transform& tf)
{
  os << "Rotation:\n" << tf.rotation << "\nTranslation:\n" << tf.translation.transpose();
  return os;
}
