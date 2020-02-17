#pragma once

#include <array>

namespace robo {

/**
** Per Joint Friction model. Based on:
** Page 1 of "Friction Compensation of Harmonic Drive Actuators" Eq. (2)
** Hausschild et al.
** http://naca.central.cranfield.ac.uk/dcsss/2004/E24_hauschild-dcsss04c.pdf
**
** The model is direction dependent i.e. wether the joint is moving in
** positive or negative direction, another set of parameters is used.
*/

class FrictionModel {

  /// Save two parameters for each joint, since friction is direction dependent
  /// index 0 is positive direction, index 1 is negative direction
  std::array<double, 2> static_coeff_{};
  std::array<double, 2> viscous_coeff_{};
  std::array<double, 2> stiction_coeff_{};
  std::array<double, 2> stribeck_vel_{};
  std::array<double, 2> form_factor_{};

public:

  FrictionModel() = default;

  /// Parameters must be passed in the right order
  FrictionModel(std::array<double, 10> params)
  : static_coeff_  ({{params[0], params[1]}}),
    viscous_coeff_ ({{params[2], params[3]}}),
    stiction_coeff_({{params[4], params[5]}}),
    stribeck_vel_  ({{params[6], params[7]}}),
    form_factor_   ({{params[8], params[9]}}) {}

/// Returns the friction force / torque for the joint [N or Nm]
double calculate_friction_force(double dq)const{
    // TODO should we use another model for translational joints?
    const int D = dq < 0; // direction pos or neg
    // in the case the velocity is exactly 0, friction is undefined
    // so we just use the positive model
    const double alpha0 = static_coeff_[D];
    const double alpha1 = stiction_coeff_[D] - static_coeff_[D];
    const double alpha2 = viscous_coeff_[D];
    const double vs = stribeck_vel_[D];
    const double delta = form_factor_[D];
    const double ret =
      (alpha0 + alpha1 * std::exp(-std::pow(std::abs(dq)/vs, delta))) *
      std::copysign(1.0, dq) + alpha2 * dq;
    return ret;
  }
};

} // namespace robo
