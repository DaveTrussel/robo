#pragma once
// According to Eigen documentation this macro should not be used in "real-world code".
// However we really dont want any calls to malloc because we are concerned about performance.
// Results in assertion failure when malloc is called by Eigen.

#include "robo/dynamics.hpp"
#include "robo/kinematics_inverse.hpp"

#include <algorithm>
#include <iostream>

namespace robo {

/// Create a chain with predefined parameters for testing
Chain make_test_chain(const double q_min, const double q_max) {

  double mass = 1.2;
  double ll   = 0.5;

  Vector3d axis_z, axis_y, length, grav;
  axis_y << 0, 1, 0;
  axis_z << 0, 0, 1;
  length << 0, 0, ll;


  Joint joint_trans {axis_y, Joint_type::Translational, q_min, q_max};
  Joint joint_ellbow{axis_y, Joint_type::Rotational   , q_min, q_max};
  Joint joint_wrist {axis_z, Joint_type::Rotational   , q_min, q_max};
  Joint joint_none  {axis_z, Joint_type::None         };

  Inertia inertia{mass, length/2, Matrix3d::Identity()};

  Body body_trans {joint_trans,  length, inertia};
  Body body_ellbow{joint_ellbow, length, inertia};
  Body body_wrist {joint_wrist,  length, inertia};
  Body body_end   {joint_none,   length, inertia};
  std::vector<Body> bodies;
  bodies.push_back(body_trans );
  bodies.push_back(body_wrist );
  bodies.push_back(body_ellbow);
  bodies.push_back(body_ellbow);
  bodies.push_back(body_wrist );
  bodies.push_back(body_ellbow);
  bodies.push_back(body_wrist );
  bodies.push_back(body_end   );

  return Chain(bodies);
}

void print_success_rates_IK(const std::vector<IK_Result>& results){
    int num_no_error = 0;
    int num_max_iter = 0;
    for(const auto res : results) {
      if(Error::None == res.error) { ++num_no_error; }
      if(Error::MaxIter == res.error) { ++num_max_iter; }
    }
    double successrate = 100.0 * double(num_no_error)/results.size();
    double maxiterrate = 100.0 * double(num_max_iter)/results.size();
    std::cout << "Success rate: " << successrate << "%\n";
    std::cout << "MaxIter rate: " << maxiterrate << "%\n";
}

} // namespace robo
