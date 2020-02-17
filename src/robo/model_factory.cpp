#include "robo/model_factory.hpp"

#include "config/axis_parameters.hpp"
#include "config/config.hpp"

#include "robo/inertia.hpp"
#include "robo/transform.hpp"
#include "robo/util.hpp"


#include <array>
#include <string>


namespace robo{

Joint make_joint(const config::AxisParameters& cfg_joint){
  Vector3d axis(0,0,1);
  Joint_type joint_type = Joint_type::None;
  util::verify(cfg_joint.type == "ELBOW" || cfg_joint.type == "WRIST",
               "joint type not valid: ELBOW or WRIST required");
  if(cfg_joint.type == "ELBOW"){
    axis << 0,1,0;
    joint_type = Joint_type::Rotational;
  }
  if(cfg_joint.type == "WRIST"){
    axis << 0,0,1;
    joint_type = Joint_type::Rotational;
  }
  // TODO translational joints (linear axis)
  Joint joint{axis, joint_type};
  // TODO make constructur that takes additional parameters
  constexpr double offset = 0.5; // degrees TODO for safety and not colliding with limit switches
  joint.set_joint_limits(math::deg_to_rad(cfg_joint.position_limit_low + offset),
                         math::deg_to_rad(cfg_joint.position_limit_high - offset)); // TOOD not yet used
  joint.set_friction_coefficients(cfg_joint.friction_coefficients);
  joint.set_rotor_inertia_equivalent(cfg_joint.transmission_ratio, cfg_joint.rotor_inertia);
  return joint;
}

Body make_body(const config::Link& cfg_link, const Joint& joint){
  Vector3d length         = eigen_copy(cfg_link.translation);
  Matrix3d orientation    = array_to_matrix(cfg_link.rotation);
  Vector3d center_of_mass = eigen_copy(cfg_link.center_of_mass);
  Matrix3d rot_inertia    = array_to_matrix(cfg_link.rotational_inertia);
  Inertia inertia {cfg_link.mass, center_of_mass, rot_inertia};

  Transform trans{orientation, length};
  return Body(joint, trans, inertia);
}

///
/// Create a Chain out of the vectors of config objects.
/// Each Joint is added to the tip/end of each Body.
/// If the number of link configs is larger than the number of joints configs,
/// the joint type of each link is assumed to be Joint_type::None i.e. a rigid
/// connection.
///
Chain make_chain(
    const std::vector<config::Link>& cfg_links,
    const std::vector<config::AxisParameters>& cfg_joints){
  util::verify(cfg_links.size() >= cfg_joints.size(),
               "cfg_links.size() >= cfg_joints.size()");
  Joint joint;
  Body body;
  config::AxisParameters cfg_joint;
  config::Link cfg_link;
  std::vector<Body> bodies;

  // Create links with a joint at the end
  for(unsigned int iter_link=0; iter_link<cfg_links.size(); ++iter_link){
    cfg_link = cfg_links.at(iter_link);
    if(iter_link >= cfg_joints.size()){
      // no more active joints to put at end of body
      joint = Joint(Vector3d(0,0,1), Joint_type::None);
    }
    else{
      cfg_joint = cfg_joints.at(iter_link);
      joint = make_joint(cfg_joint);
    }
    body = make_body(cfg_link, joint);
    bodies.push_back(body);
  }
  Chain chain{bodies};
  chain.add_rotor_inertias_to_body_inertias();
  return chain;
}

} // namespace robo
