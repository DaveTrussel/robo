#pragma once

#include "robo/chain.hpp"

#include "util/assert.hpp"

#include "robo/body.hpp"
#include "robo/joint.hpp"
#include "robo/typedef.hpp"

#include <vector>

namespace config { struct AxisParameters; }
namespace config { struct Link; }


namespace robo{

/**
** Provides an interface to create a Joint from a config object
*/
Joint make_joint(const config::AxisParameters& cfg_joint);

/**
** Provides an interface to create a Body from a config and a Joint object
*/
Body make_body(const config::Link& cfg_link, const Joint& joint);

/**
** Provides an interface to create a Chain model from config objects
*/
Chain make_chain(
    const std::vector<config::Link>& cfg_links,
    const std::vector<config::AxisParameters>& cfg_joints);

/**
** Takes a STL Container of rotational inertia matrix elements and creates a
** robo::Matrix3d out of it. Order of elements is XX, YY, ZZ, XY, XZ, YZ.
*/
template<typename Container>
inline Matrix3d create_rot_inertia(const Container& v){
  util::verify(6 == v.size(), "Size mismatch: rot_inertia vector != 6");
  Matrix3d ret;
  ret << v.at(0), v.at(3), v.at(4),
         v.at(3), v.at(1), v.at(5),
         v.at(4), v.at(5), v.at(2);
  return ret;
}

} // namespace robo
