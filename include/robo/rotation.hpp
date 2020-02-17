#pragma once

#include "robo/typedef.hpp"

namespace robo{

/**
** Calculates the rotation matrix based on the Euler-Rodrigues formula 
** (rotation around axis by angle)
** The rotations are used as defined in:
** "Rigid body dynamics algorithms" by R.Featherstone, 2007,
**  ISBN 978-0-387-74314-1, Page 22
** (this is actually the inverse from the rotations defined e.g. by 
** Eigen::AngleAxis)
*/
inline Matrix3d rotate_around_axis(const Vector3d& axis, const double& angle){
  double theta = -angle/2.0;
  double a = std::cos(theta);
  double sine = std::sin(theta);
  double b = -sine * axis[0];
  double c = -sine * axis[1];
  double d = -sine * axis[2];
  double aa = a*a;
  double bb = b*b;
  double cc = c*c;
  double dd = d*d;
  double bc = b*c;
  double ad = a*d;
  double ac = a*c;
  double ab = a*b;
  double bd = b*d;
  double cd = c*d;
  Matrix3d rot;
  rot << aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac),
         2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab),
         2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc;
  return rot;
}

}