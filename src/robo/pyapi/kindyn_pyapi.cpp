/*
Python interface for kinematics and dynamics
*/

#include "config/config_parser.hpp"
#include "robo/dynamics.hpp"
#include "robo/kinematics.hpp"
#include "robo/model_factory.hpp"
#include "pyapi/helpers.hpp"

#include "util/assert.hpp"

#include "version/version.hpp"

#include <fstream>

namespace kd = robo;

namespace { // anonymous

kd::Chain get_chain(
    const std::string& config_directory,
    const std::string& config_section_end,
    const std::string& voltage,
    const char         robot_version) {
  config::RobotInfo rinfo;
  rinfo.voltage = voltage;
  rinfo.version = std::string(1, robot_version);
  std::ofstream logstream(kd::logfile());
  config::ConfigParser cp{logstream, rinfo, config_directory};
  std::vector<config::Link> cfg_links;
  if (config_section_end.empty()) {
    cfg_links = cp.read_arm_link_configs(); }
  else {
    cfg_links = cp.read_complete_link_configs(config_section_end); }
  const std::vector<config::AxisParameters> cfg_joints =
    cp.read_axes_parameters("ARM");
  return kd::make_chain(cfg_links, cfg_joints);
}

} // namespace anonymous

struct DynamicsPythonAPI{
  DynamicsPythonAPI(
    const std::string& cfg_dir,
    const std::string& cfg_end,
    const std::string& voltage,
    const char         robot_version)
  {
    kd::Chain chain = ::get_chain(cfg_dir, cfg_end, voltage, robot_version);
    dynamics = kd::Dynamics(chain);
  }

  /// Constructor without gripper
  DynamicsPythonAPI(
    const std::string& cfg_dir,
    const std::string& voltage,
    const char         robot_version)
  {
    kd::Chain chain = ::get_chain(cfg_dir, "", voltage, robot_version);
    dynamics = kd::Dynamics(chain);
  }

  list pyapi_calculate_inverse_dynamics(list   q_py,
                                        list  dq_py,
                                        list ddq_py,
                                        list gravity_py){
    using namespace robo;
    util::verify(len(  q_py) == dynamics.num_joints(), "robo: Wrong size");
    util::verify(len( dq_py) == dynamics.num_joints(), "robo: Wrong size");
    util::verify(len(ddq_py) == dynamics.num_joints(), "robo: Wrong size");
    const VectorXd q   = detail_::py_list_to_eigen_vector(  q_py);
    const VectorXd dq  = detail_::py_list_to_eigen_vector( dq_py);
    const VectorXd ddq = detail_::py_list_to_eigen_vector(ddq_py);
    const Vector3d g   = detail_::py_list_to_eigen_vector(gravity_py);
    // for now expose only interface without external forces
    dynamics.calculate_inverse_dynamics(q, dq, ddq, g);
    return detail_::eigen_vector_to_py_list(dynamics.generalized_forces);
  }

  list pyapi_friction_forces(list dq_py){
    util::verify(len( dq_py) == dynamics.num_joints(), "robo: Wrong size");
    const kd::VectorXd dq  = detail_::py_list_to_eigen_vector(dq_py);
    const kd::VectorXd ret = dynamics.calculate_friction_forces(dq); 
    return detail_::eigen_vector_to_py_list(ret);
  }
  
private:
  kd::Dynamics dynamics;
};

struct KinematicsPythonAPI{
  KinematicsPythonAPI(
    const std::string& cfg_dir,
    const std::string& cfg_end,
    const std::string& voltage,
    const char         robot_version):
  kin_(::get_chain(cfg_dir, cfg_end, voltage, robot_version)){}

  /// Constructor without gripper
  KinematicsPythonAPI(
    const std::string& cfg_dir,
    const std::string& voltage,
    const char         robot_version):
  kin_(::get_chain(cfg_dir, "", voltage, robot_version)){}

  p::tuple pyapi_calculate_forward(list q_py) {
    util::verify(p::len(q_py) == kin_.num_joints(), "robo: Wrong size");
    const kd::VectorXd q   = detail_::py_list_to_eigen_vector(q_py);
    const kd::Transform tip = kin_.calculate_forward(q);
    p::list t   = detail_::eigen_vector_to_py_list(tip.translation);
    p::list rot = detail_::eigen_matrix_to_py_list(tip.rotation);
    return p::make_tuple(t, rot);
  }

  p::list pyapi_calculate_inverse(
      p::list translation,
      p::list rotation,
      p::object q_start_py) {
    using namespace robo;
    util::verify(len(translation) == 3,
      "robo: Wrong size of `translation`");
    const Vector3d t = detail_::py_list_to_eigen_vector(translation);
    const Matrix3d r = detail_::py_list_to_eigen_matrix(rotation);
    const Transform target{r, t};
    VectorXd q_start;
    if (q_start_py == p::object()) { // python: if q_start_py is None:
      q_start = VectorXd(); // setting to empty will start global solver
    }
    else {
      p::list q_start_lst(q_start_py);
      util::verify(p::len(q_start_lst) == kin_.num_joints(),
        "robo: Wrong size of `q_start`");
      q_start = detail_::py_list_to_eigen_vector(q_start_lst);
    }
    const auto ret = kin_.calculate_inverse(target, q_start);
    if (kd::Error::None == ret.error) {
      return detail_::eigen_vector_to_py_list(ret.q);
    }
    else {
      return p::list();
    }
  }
  
private:
  kd::Kinematics kin_;
};

BOOST_PYTHON_MODULE(robo)
{
  PyEval_InitThreads();

  def("long_version", version::long_version, 
      ":return: A text describing the MCM and its version");

  def("build_number", version::build_number, 
      ":return: A string describing the MCM build");

  def("copyright", version::copyright, 
      ":return: Copyright string for the MCM");

    class_<DynamicsPythonAPI>("Dynamics",
R"(Creates a (inverse) dynamic model of the robot, based on the provided 
configuration files for the robot arm and end effector.

:param config_directory: path to config root directory where 'default' and 
'custom' directories are located (string)
:param configsection_end: name of end effector configuration (string) (optional)
:param voltage: motor supply voltage of the robot (string)
:param robot_version: version of the robot (char)",
    init<const std::string&,
         const std::string&,
         const std::string&,
         const char>())
      .def(init<const std::string&,
                const std::string&,
                const char>()) // constructor without gripper
      .def("calculate_inverse_dynamics",
           &DynamicsPythonAPI::pyapi_calculate_inverse_dynamics,
R"(Finds the generalized forces required to produce a given acceleration in a 
rigid-body system. Everything in SI-Units (radians for angles!))
:param q:   list of joint positions    [rad] or [m]
:param dq:  list of joint velocities   [rad/s] or [m/s]
:param ddq: list of joint acceleration [rad/s^2] or [m/s^2]
:param gravity: gravity vector as a list [m/s^2]
:return: generalized forces [N] or [Nm])")
      .def("friction_forces", &DynamicsPythonAPI::pyapi_friction_forces,
R"(Calculates the friction forces only.
:param dq: list of joint velocities [rad/s] or [m/s]
:return:   list of conversion factors.)");

  class_<KinematicsPythonAPI>("Kinematics",
R"(Creates a kinematic model of the robot, based on the provided 
configuration files for the robot arm and end effector.

:param config_directory: path to config root directory where 'default' and 
'custom' directories are located (string)
:param configsection_end: name of end effector configuration (string) (optional)
:param voltage: motor supply voltage of the robot (string)
:param robot_version: version of the robot (char)",
    init<const std::string&,
         const std::string&,
         const std::string&,
         const char>())
      .def(init<const std::string&,
                const std::string&,
                const char>()) // constructor without gripper
      .def("calculate_inverse",
           &KinematicsPythonAPI::pyapi_calculate_inverse,
R"(Runs a numeric solver to solve the inverse kinematic problem. If successful
returns one solution to the problem. Everything in SI-Units (radians for angles!))
:param translation: list [x,y,z] of desired world coordinates (expressed in the
base frame) [m]
:param rotation: rotation matrix expressing the rotation from base to end in the
base frame. Since we can not easily take numpy arrays, a list of the matrix
elements needs to be passed: [xx, xy, xz, yx, yy, yz, zx, zy, zz]
:param q_start: list of joint positions with inital guess where the solver will
start [rad] or [m]
:return: list of found joint position [m] or [rad] if successfull or empty
list if unsuccessfull)")
      .def("calculate_forward",
           &KinematicsPythonAPI::pyapi_calculate_forward,
R"(Calculates the forward kinematics. Returns translation from base frame to 
end of the kinematic chain expressed in the base frame [m]. Returns the rotation
matrix from the base frame to the end of the kinematic chain expressed in the
base frame.
Since we can not easily return numpy arrays we return the matrix elements as a
list: [xx, xy, xz, yx, yy, yz, zx, zy, zz]
:param q: list of joint positions [rad] or [m]
:return:  tuple of ([x,y,z], rotation_matrix) [m])");

}
