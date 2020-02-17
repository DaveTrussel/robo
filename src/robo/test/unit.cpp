/**
** Unit tests of kinematics and dynamics
*/

#define CATCH_CONFIG_MAIN

///#include "config/axis_parameters.hpp"
//#include "config/config.hpp"

#include "robo/dynamics.hpp"
#include "robo/kinematics.hpp"
//#include "robo/model_factory.hpp"
#include "robo/typedef.hpp"
#include "robo/util.hpp"

#include "catch.hpp" // unit testing framework
#include <Eigen/Dense>

#include <iomanip>
#include <iostream>
#include <vector>

#include <cmath>

constexpr auto pi = 3.141592653589793238462643383279502884L;
// absolute tolerance for all tests
constexpr auto abstol = 1e-12;

using namespace robo;

// Helper functions
template<typename DerivedA, typename DerivedB>
bool all_close(const Eigen::DenseBase<DerivedA>& a,
              const Eigen::DenseBase<DerivedB>& b,
              const typename DerivedA::RealScalar& rtol
                  = Eigen::NumTraits<typename DerivedA::RealScalar>::dummy_precision(),
              const typename DerivedA::RealScalar& atol
                  = Eigen::NumTraits<typename DerivedA::RealScalar>::epsilon())
{
  return ((a.derived() - b.derived()).array().abs()
          <= (atol + rtol * b.derived().array().abs())).all();
}

Matrix3d rx(double angle){
  Matrix3d rotation;
  double c = std::cos(angle);
  double s = std::sin(angle);
  rotation <<  1,  0,  0,
               0,  c,  s,
               0, -s,  c;
  return rotation;
}

Matrix3d ry(double angle){
  Matrix3d rotation;
  double c = std::cos(angle);
  double s = std::sin(angle);
  rotation <<  c,  0, -s,
               0,  1,  0,
               s,  0,  c;
  return rotation;
}

Matrix3d rz(double angle){
  Matrix3d rotation;
  double c = std::cos(angle);
  double s = std::sin(angle);
  rotation <<  c,  s,  0,
              -s,  c,  0,
               0,  0,  1;
  return rotation;
}

// Test scenarios
SCENARIO("Joint tests"){
  GIVEN("y and z axis"){
    std::cout << std::fixed << std::setprecision(2);
    Vector3d axis_z, axis_y;
    axis_y << 0.0, 1.0, 0.0;
    axis_z << 0.0, 0.0, 1.0;
    SECTION("Joint transformations"){
      Joint joint_trans = Joint(axis_z, Joint_type::Translational);
      Joint joint_ellbow = Joint(axis_y, Joint_type::Rotational);
      Joint joint_wrist = Joint(axis_z, Joint_type::Rotational);
      Joint joint_none = Joint(axis_z, Joint_type::None);

      REQUIRE(joint_trans.transform(0.0).translation  == Vector3d::Zero());
      REQUIRE(joint_ellbow.transform(0.0).translation == Vector3d::Zero());
      REQUIRE(joint_wrist.transform(0.0).translation  == Vector3d::Zero());
      REQUIRE(joint_none.transform(0.0).translation   == Vector3d::Zero());
      REQUIRE(joint_trans.transform(0.0).rotation     == Matrix3d::Identity());
      REQUIRE(joint_ellbow.transform(0.0).rotation    == Matrix3d::Identity());
      REQUIRE(joint_wrist.transform(0.0).rotation     == Matrix3d::Identity());
      REQUIRE(joint_none.transform(0.0).rotation      == Matrix3d::Identity());

      double angle = pi/3;
      REQUIRE(joint_trans.transform(1.0).translation     == Vector3d(0.0, 0.0, 1.0));
      REQUIRE(joint_ellbow.transform(angle).translation  == Vector3d::Zero());
      REQUIRE(joint_wrist.transform(angle).translation   == Vector3d::Zero());
      REQUIRE(joint_none.transform(angle).translation    == Vector3d::Zero());
      REQUIRE(joint_trans.transform(1.0).rotation        == Matrix3d::Identity());
      REQUIRE(all_close(joint_ellbow.transform(angle).rotation, ry(angle), 0, abstol));
      REQUIRE(all_close(joint_wrist.transform(angle).rotation, rz(angle), 0, abstol));
      REQUIRE(joint_none.transform(angle).rotation       == Matrix3d::Identity());
    }
  }
}

SCENARIO("Transform tests"){
  GIVEN("Nautical angles in degrees"){
    std::array<double, 3> nautical_angles{{0.0, 0.0, 0.0}};
    SECTION("roll=0, pitch=0, yaw=0"){
      Matrix3d orientation = nautical_angles_to_matrix(nautical_angles);
      REQUIRE(all_close(orientation, Matrix3d::Identity(), 0, abstol));
    }
    SECTION("roll=45, pitch=0, yaw=0"){
      nautical_angles.at(0) = 45.0;
      Matrix3d orientation = nautical_angles_to_matrix(nautical_angles);
      Matrix3d expected = rx(robo::math::deg_to_rad(nautical_angles.at(0)));
      REQUIRE(all_close(orientation, expected, 0, abstol));
    }
    SECTION("roll=0, pitch=30, yaw=0"){
      nautical_angles.at(1) = 30.0;
      Matrix3d orientation = nautical_angles_to_matrix(nautical_angles);
      Matrix3d expected = ry(robo::math::deg_to_rad(nautical_angles.at(1)));
      REQUIRE(all_close(orientation, expected, 0, abstol));
    }
    SECTION("roll=0, pitch=0, yaw=60"){
      nautical_angles.at(2) = 60.0;
      Matrix3d orientation = nautical_angles_to_matrix(nautical_angles);
      Matrix3d expected = rz(robo::math::deg_to_rad(nautical_angles.at(2)));
      REQUIRE(all_close(orientation, expected, 0, abstol));
    }
    SECTION("nautical_angles -> matrix -> nautical_angles -> matrix"){
      nautical_angles = {{10.0, 5.0, 1.0}};
      Matrix3d orientation = nautical_angles_to_matrix(nautical_angles);
      //std::cout << "DEBUG: orientation:\n" << orientation << std::endl;
      Vector3d tmp = Transform(orientation).nautical_angles();
      //std::cout << "DEBUG: nautical_angles:" << tmp.transpose() << std::endl;
      Matrix3d res = nautical_angles_to_matrix(tmp[0], tmp[1], tmp[2]);
      //std::cout << "DEBUG: res:\n" << res << std::endl;
      REQUIRE(all_close(res, orientation, 0, 0.05)); // higher abstol
    }

  }
  GIVEN("Two transformations"){
    Vector3d pos_a, pos_b, axis_y, axis_z;
    pos_a << 0, 0, 1;
    axis_y << 0, 200, 0;
    axis_z << 0, 0, 0.5;
    SECTION("Difference of two transformations (1)"){
      Joint joint = Joint(axis_z, Joint_type::Rotational);
      Transform f_a(pos_a);
      Transform f_b = joint.transform(1.337);
      Motion diff = f_b - f_a;
      Motion diff_expected{Vector3d(0, 0, 1.337), Vector3d(0, 0, -1)};
      REQUIRE(all_close(diff.angular, diff_expected.angular, 0, abstol));
      REQUIRE(all_close(diff.linear , diff_expected.linear , 0, abstol));
    }
    SECTION("Difference of two transformations (2)"){
      Joint joint = Joint(axis_y, Joint_type::Rotational);
      Transform f_a(pos_a);
      Transform f_b = joint.transform(0.42);
      Motion diff = f_b - f_a;
      Motion diff_expected{Vector3d(0, 0.42, 0), Vector3d(0, 0, -1)};
      REQUIRE(all_close(diff.angular, diff_expected.angular, 0, abstol));
      REQUIRE(all_close(diff.linear , diff_expected.linear , 0, abstol));
    }
  }

  GIVEN("6D Motion, 6D Force & Plucker Transformation from body.transform(q):\n" +
        "Random"){
    Vector3d length, axis;
    length  << 1, 4, 5;
    axis    << 1, 2, 3;
    Joint joint{axis, Joint_type::Rotational};
    Body link{joint, length};
    Transform transform = link.transform(pi/3.0);
    Force wrench(Vector3d(2,4,6), Vector3d(1,3,5)); // some random wrench
    Motion twist(Vector3d(2,4,6), Vector3d(1,3,5)); // some random twist
    WHEN("Transform forward and then backward"){
      Force wrench_in_new_frame = transform.apply(wrench);
      Motion twist_in_new_frame = transform.apply(twist);
      Force wrench_in_old_frame = transform.apply_inverse(wrench_in_new_frame);
      Motion twist_in_old_frame = transform.apply_inverse(twist_in_new_frame);
      THEN("Motion and Force need to be the same again"){
        REQUIRE(all_close(wrench_in_old_frame.force, wrench.force));
        REQUIRE(all_close(wrench_in_old_frame.torque, wrench.torque));
        REQUIRE(all_close(twist_in_old_frame.linear, twist.linear));
        REQUIRE(all_close(twist_in_old_frame.angular, twist.angular));
      }
    }
  }
  GIVEN("6D Motion, 6D Force & Plucker Transformation from body.transform(q):\n" +
        " 2D check"){
    const double mag = std::sqrt(2.0);
    Vector3d length, axis_y;
    length  << 1, 0, 1;
    axis_y  << 0, 1, 0;
    Joint joint{axis_y, Joint_type::Rotational};
    Body link{joint, length};
    Transform transform = link.transform(pi/4.0);
    // wrench: force in z and torque in y direction
    Force wrench(Vector3d(0,1,0), Vector3d(0,0,mag));
    // twist: linear velocity in z and angular in y direction
    Motion twist(Vector3d(0,1,0), Vector3d(0,0,mag));
    // compare with hand calculation
    Force wrench_check(Vector3d(0,1.0+mag,0), Vector3d(-1,0,1));
    Motion twist_check(Vector3d(0,1,0), Vector3d(mag-1.0,0,1));
    WHEN("Transform forward"){
      Force wrench_in_new_frame = transform.apply(wrench);
      Motion twist_in_new_frame = transform.apply(twist);
      THEN("Motion and Force are the same as calculation by hand"){
        REQUIRE(all_close(wrench_in_new_frame.force, wrench_check.force));
        REQUIRE(all_close(wrench_in_new_frame.torque, wrench_check.torque));
        REQUIRE(all_close(twist_in_new_frame.linear, twist_check.linear));
        REQUIRE(all_close(twist_in_new_frame.angular, twist_check.angular));
      }
    }
    WHEN("Transform backwards (1)"){
      Force wrench_in_old_frame = transform.apply_inverse(wrench_check);
      Motion twist_in_old_frame = transform.apply_inverse(twist_check);
      THEN("Motion and Force need to be the same again"){
        REQUIRE(all_close(wrench_in_old_frame.force, wrench.force));
        REQUIRE(all_close(wrench_in_old_frame.torque, wrench.torque));
        REQUIRE(all_close(twist_in_old_frame.linear, twist.linear));
        REQUIRE(all_close(twist_in_old_frame.angular, twist.angular));
      }
    }
    WHEN("Transform backwards (2)"){
      Force wrench_in_old_frame = transform.inverse().apply(wrench_check);
      Motion twist_in_old_frame = transform.inverse().apply(twist_check);
      THEN("Motion and Force need to be the same again"){
        REQUIRE(all_close(wrench_in_old_frame.force, wrench.force));
        REQUIRE(all_close(wrench_in_old_frame.torque, wrench.torque));
        REQUIRE(all_close(twist_in_old_frame.linear, twist.linear));
        REQUIRE(all_close(twist_in_old_frame.angular, twist.angular));
      }
    }
  }
}

/*
SCENARIO("Model factory tests"){
  GIVEN("Two vectors of link and axis parameters"){
    config::AxisParameters cfg_joint;
    std::vector<config::AxisParameters> cfg_joints;
    cfg_joint.type = "ELBOW";

    config::Link cfg_link;
    std::vector<config::Link> cfg_links;

    for(int i=0; i<5; ++i){
      cfg_joint.rotor_inertia = 1.1 * i;
      cfg_joint.transmission_ratio = 160 + i;
      cfg_links.push_back(cfg_link);
      cfg_joints.push_back(cfg_joint);
    }
    cfg_links.push_back(cfg_link);

    Chain chain = make_chain(cfg_links, cfg_joints);
    REQUIRE(chain.num_joints == 5);
    REQUIRE(chain.num_bodies == 6);
  }
}
*/

SCENARIO("Dynamics tests"){
  GIVEN("Simple kinematic chain (ellbow, ellbow) & gravity"){
    double mass = 1.2;
    double gg = 9.8;
    double ll = 1.0;
    Vector3d axis_z, axis_y, length, grav;
    axis_y << 0, 1, 0;
    axis_z << 0, 0, 1;
    length << 0, 0, ll;
    grav   << 0, 0, -gg;
    Joint joint_ellbow{axis_y, Joint_type::Rotational};
    Joint joint_none  {axis_z, Joint_type::None      };
    Inertia inertia{mass, length/2, Matrix3d::Identity()};
    Body body_ellbow(joint_ellbow, length, inertia);
    Body body_end   (joint_none,   length, inertia);
    std::vector<Body> bodies;
    bodies.push_back(body_ellbow);
    bodies.push_back(body_ellbow);
    bodies.push_back(body_end   );
    Chain chain{bodies};
    Dynamics dyn{chain};
    VectorXd q(chain.num_joints), dq(chain.num_joints), ddq(chain.num_joints);
    WHEN("Configuration static (1) q = 0, pi/2"){
      q << 0, pi/2;
      dyn.calculate_inverse_dynamics(q, grav);
      VectorXd gen_forces = dyn.generalized_forces;
      VectorXd gen_forces_expected(chain.num_joints);
      double tmp = ll/2.0 * mass * gg;
      gen_forces_expected << -tmp, -tmp;
      //std::cout << "DEBUG: Expected forces: " << gen_forces_expected.transpose() << std::endl;
      //std::cout << "DEBUG: Calculated forces: " << gen_forces.transpose() << std::endl;
      REQUIRE(all_close(gen_forces, gen_forces_expected, 0, abstol));
    }
    WHEN("Configuration static (2) q = pi/2, 0"){
      q << pi/2, 0;
      dyn.calculate_inverse_dynamics(q, grav);
      VectorXd gen_forces = dyn.generalized_forces;
      VectorXd gen_forces_expected(chain.num_joints);
      double tmp = ll/2.0 * mass * gg;
      gen_forces_expected << -4*tmp, -tmp;
      REQUIRE(all_close(gen_forces, gen_forces_expected, 0, abstol));
    }
    WHEN("Configuration static (3) q = pi/2, -pi/2"){
      q << pi/2, -pi/2;
      dyn.calculate_inverse_dynamics(q, grav);
      VectorXd gen_forces = dyn.generalized_forces;
      VectorXd gen_forces_expected(chain.num_joints);
      double tmp = ll/2 * mass * gg;
      gen_forces_expected << -3*tmp, 0;
      REQUIRE(all_close(gen_forces, gen_forces_expected, 0, abstol));
    }
  }
  GIVEN("PRob2 kinematic chain (wrist, ellbow, ellbow, wrist, ellbow, wrist)\n" +
        " & gravity. Same parameters for all links."){
    double mass = 2;
    double gg = 9.8;
    double ll = 1.0;
    Vector3d axis_z, axis_y, length, grav;
    axis_y << 0, 1, 0;
    axis_z << 0, 0, 1;
    length << 0, 0, ll;
    grav   << 0, 0, -gg;
    //Joint joint_trans{axis_z, Joint_type::Translational};
    Joint joint_ellbow{axis_y, Joint_type::Rotational};
    Joint joint_wrist {axis_z, Joint_type::Rotational};
    Joint joint_none  {axis_z, Joint_type::None      };
    Inertia inertia{mass, length/2, Matrix3d::Identity()};
    Body body_ellbow(joint_ellbow, length, inertia);
    Body body_wrist (joint_wrist,  length, inertia);
    Body body_end   (joint_none,   length, inertia);
    std::vector<Body> bodies;
    bodies.push_back(body_wrist );
    bodies.push_back(body_ellbow);
    bodies.push_back(body_ellbow);
    bodies.push_back(body_wrist );
    bodies.push_back(body_ellbow);
    bodies.push_back(body_wrist );
    bodies.push_back(body_end   );
    Chain chain{bodies};
    Dynamics dyn{chain};
    VectorXd q(chain.num_joints), dq(chain.num_joints), ddq(chain.num_joints);
    WHEN("Configuration static (1) q = 0, 0, 0, 0, pi/2, 0"){
      q << 0, 0, 0, 0, pi/2.0, 0;
      dyn.calculate_inverse_dynamics(q, grav);
      VectorXd gen_forces = dyn.generalized_forces;
      VectorXd gen_forces_expected(chain.num_joints);
      double tmp = (ll/2.0 + 3.0*ll/2.0) * mass * gg;
      gen_forces_expected << 0.0, -tmp, -tmp, 0.0, -tmp, 0.0;
      REQUIRE(all_close(gen_forces, gen_forces_expected, 0, abstol));
    }
    WHEN("Configuration static (2) q = 0, pi/2, 0, 0, 0, 0"){
      q << 0, pi/2, 0, 0, 0, 0;
      dyn.calculate_inverse_dynamics(q, grav);
      VectorXd gen_forces = dyn.generalized_forces;
      VectorXd gen_forces_expected(chain.num_joints);
      double tmp = ll * mass * gg;
      gen_forces_expected << 0.0, -2.5*5*tmp, -2*4*tmp, 0.0, -1*2*tmp, 0.0;
      REQUIRE(all_close(gen_forces, gen_forces_expected, 0, abstol));
    }
    WHEN("Configuration static (3) wall-mounted, q = 0, 0, 0, 0, 0, 0\n"+
         "                                       g = 9.8, 0, 0"){
      q << 0, 0, 0, 0, 0, 0;
      grav << gg, 0, 0;
      dyn.calculate_inverse_dynamics(q, grav);
      VectorXd gen_forces = dyn.generalized_forces;
      VectorXd gen_forces_expected(chain.num_joints);
      double tmp = ll * mass * gg;
      gen_forces_expected << 0.0, -2.5*5*tmp, -2*4*tmp, 0.0, -1*2*tmp, 0.0;
      REQUIRE(all_close(gen_forces, gen_forces_expected, 0, abstol));
    }
    WHEN("Configuration dynamic (1),   q = 0, 0, 0, 0, 0, 0\n"+
         "                            dq = 0, 0, 0, 0, 0, 0\n"+
         "                           ddq = 0, 0, 0, 0, 0, 3"){
      double acc = 3.0;
      q   << 0, 0, 0, 0, 0, 0;
      dq  << 0, 0, 0, 0, 0, 0;
      ddq << 0, 0, 0, 0, 0, acc;
      dyn.calculate_inverse_dynamics(q, dq, ddq, grav);
      VectorXd gen_forces = dyn.generalized_forces;
      VectorXd gen_forces_expected(chain.num_joints);
      gen_forces_expected << acc, 0.0, 0.0, acc, 0.0, acc;
      REQUIRE(all_close(gen_forces, gen_forces_expected, 0, abstol));
    }
    WHEN("Configuration centrifugal (1),   q = 0, 0, 0, 0, -pi/2, 0\n"+
         "                                dq = 2, 0, 0, 2,     0, 0\n"+
         "                               ddq = 0, 0, 0, 0,     0, 0\n"+
         "For easy hand calculation: gravity=0, rot_inertia=0"){
      double w = 2.0;
      q    << 0, 0, 0, 0, -pi/2, 0;
      dq   << w, 0, 0, w, 0, 0;
      ddq  << 0, 0, 0, 0, 0, 0;
      grav << 0, 0, 0;
      for(auto& body : chain.bodies()){
          body.inertia = Inertia(mass, length/2.0, Matrix3d::Zero());
      }
      dyn.calculate_inverse_dynamics(q, dq, ddq, grav);
      VectorXd gen_forces = dyn.generalized_forces;
      VectorXd gen_forces_expected(chain.num_joints);
      double f_centrifugal = 4*w*w * (mass * ll/2.0 + mass * 3.0*ll/2.0);
      double t3 = f_centrifugal * 2.0 * ll;
      double t2 = f_centrifugal * 3.0 * ll;
      gen_forces_expected << 0.0, t2, t3, 0.0, 0.0, 0.0;
      REQUIRE(all_close(gen_forces, gen_forces_expected, 0, abstol));
    }
  }
  GIVEN("PRob2 on linear axis kinematic chain \n" +
        "(trans, wrist, ellbow, ellbow, wrist, ellbow, wrist) & gravity\n" +
        "Same parameters for all links."){
    double mass = 1.2;
    double gg = 9.8;
    double ll = 1.0;
    Vector3d axis_z, axis_y, length, grav;
    axis_y << 0, 1, 0;
    axis_z << 0, 0, 1;
    length << 0, 0, ll;
    grav   << 0, 0, -gg;
    Joint joint_trans {axis_y, Joint_type::Translational};
    Joint joint_ellbow{axis_y, Joint_type::Rotational   };
    Joint joint_wrist {axis_z, Joint_type::Rotational   };
    Joint joint_none  {axis_z, Joint_type::None         };
    Inertia inertia{mass, length/2, Matrix3d::Identity()};
    Body body_trans (joint_trans,  length, inertia);
    Body body_ellbow(joint_ellbow, length, inertia);
    Body body_wrist (joint_wrist,  length, inertia);
    Body body_end   (joint_none,   length, inertia);
    std::vector<Body> bodies;
    bodies.push_back(body_trans );
    bodies.push_back(body_wrist );
    bodies.push_back(body_ellbow);
    bodies.push_back(body_ellbow);
    bodies.push_back(body_wrist );
    bodies.push_back(body_ellbow);
    bodies.push_back(body_wrist );
    bodies.push_back(body_end   );
    Chain chain{bodies};
    Dynamics dyn{chain};
    VectorXd q(chain.num_joints), dq(chain.num_joints), ddq(chain.num_joints);
    WHEN("Configuration static (1) q = 0, 0, 0, 0, 0, pi/2, 0"){
      q << 0, 0, 0, 0, 0, pi/2.0, 0;
      dyn.calculate_inverse_dynamics(q, grav);
      VectorXd gen_forces = dyn.generalized_forces;
      VectorXd gen_forces_expected(chain.num_joints);
      double tmp = (ll/2.0 + 3.0*ll/2.0) * mass * gg;
      gen_forces_expected << 0.0, 0.0, -tmp, -tmp, 0.0, -tmp, 0.0;
      REQUIRE(all_close(gen_forces, gen_forces_expected, 0, abstol));
    }
    WHEN("Configuration static (2) q = 0, pi/2, 0, 0, 0, 0"){
      q << 0, 0, pi/2, 0, 0, 0, 0;
      dyn.calculate_inverse_dynamics(q, grav);
      VectorXd gen_forces = dyn.generalized_forces;
      VectorXd gen_forces_expected(chain.num_joints);
      double tmp = ll * mass * gg;
      gen_forces_expected << 0.0, 0.0, -2.5*5*tmp, -2*4*tmp, 0.0, -1*2*tmp, 0.0;
      REQUIRE(all_close(gen_forces, gen_forces_expected, 0, abstol));
    }
    WHEN("Configuration static (3) wall-mounted, q = 0, pi/2, 0, 0, 0, 0, 0, 0\n"+
         "                                       g = 0, 9.8, 0"){
      q << 0, pi/2.0, 0, 0, 0, 0, 0;
      grav << 0, gg, 0;
      dyn.calculate_inverse_dynamics(q, grav);
      VectorXd gen_forces = dyn.generalized_forces;
      VectorXd gen_forces_expected(chain.num_joints);
      double tmp = ll * mass * gg;
      gen_forces_expected << -7*mass*gg, 0.0, -2.5*5*tmp, -2*4*tmp, 0.0, -1*2*tmp, 0.0;
      REQUIRE(all_close(gen_forces, gen_forces_expected, 0, abstol));
    }
    WHEN("Configuration dynamic (1),   q = 0, 0, 0, 0, 0, 0, 0\n"+
         "                            dq = 0, 0, 0, 0, 0, 0, 0\n"+
         "                           ddq = 0, 0, 0, 0, 0, 0, 3"){
      double acc = 3.0;
      q   << 0, 0, 0, 0, 0, 0, 0;
      dq  << 0, 0, 0, 0, 0, 0, 0;
      ddq << 0, 0, 0, 0, 0, 0, acc;
      dyn.calculate_inverse_dynamics(q, dq, ddq, grav);
      VectorXd gen_forces = dyn.generalized_forces;
      VectorXd gen_forces_expected(chain.num_joints);
      gen_forces_expected << 0.0, acc, 0.0, 0.0, acc, 0.0, acc;
      REQUIRE(all_close(gen_forces, gen_forces_expected, 0, abstol));
    }
    WHEN("Configuration centrifugal (1),   q = 0, pi/2, 0, 0, 0, -pi/2, 0\n"+
         "                                dq = 0,    2, 0, 0, 2,     0, 0\n"+
         "                               ddq = 0,    0, 0, 0, 0,     0, 0\n"+
         "For easy hand calculation: gravity=0, rot_inertia=0"){
      double w = 2.0;
      q    << 0, pi/2, 0, 0, 0, -pi/2, 0;
      dq   << 0, w, 0, 0, w, 0, 0;
      ddq  << 0, 0, 0, 0, 0, 0, 0;
      grav << 0, 0, 0;
      for(auto& body : chain.bodies()){
        body.inertia = Inertia(mass, length/2.0, Matrix3d::Zero());
      }
      dyn.calculate_inverse_dynamics(q, dq, ddq, grav);
      VectorXd gen_forces = dyn.generalized_forces;
      VectorXd gen_forces_expected(chain.num_joints);
      double f_centrifugal = 4*w*w * (mass * ll/2.0 + mass * 3.0*ll/2.0);
      double t3 = f_centrifugal * 2.0 * ll;
      double t2 = f_centrifugal * 3.0 * ll;
      gen_forces_expected << f_centrifugal, 0.0, t2, t3, 0.0, 0.0, 0.0;
      //std::cout << "DEBUG: Expected forces: " << gen_forces_expected.transpose() << std::endl;
      //std::cout << "DEBUG: Calculated forces: " << gen_forces.transpose() << std::endl;
      REQUIRE(all_close(gen_forces, gen_forces_expected, 0, abstol));
    }
  }
}

SCENARIO("Kinematics tests"){
  GIVEN("Simple 2D kinematic chain (ellbow, ellbow)"){
    double ll = 1.0;
    Vector3d axis_z, axis_y, length;
    axis_y << 0, 1, 0;
    axis_z << 0, 0, 1;
    length << 0, 0, ll;
    Joint joint_ellbow{axis_y, Joint_type::Rotational};
    Joint joint_none  {axis_z, Joint_type::None      };
    //Inertia inertia{mass, length/2, Matrix3d::Identity()};
    Inertia inertia;
    Body body_ellbow(joint_ellbow, length, inertia);
    Body body_end   (joint_none,   length, inertia);
    std::vector<Body> bodies;
    bodies.push_back(body_ellbow);
    bodies.push_back(body_ellbow);
    bodies.push_back(body_end   );
    Chain chain{bodies};
    Kinematics kin{chain};
    ForwardKinematics fk{chain};
    VectorXd q(chain.num_joints), dq(chain.num_joints);
    WHEN("Forward kinematics: Home position"){
      q << 0, 0;
      Transform result = kin.calculate_forward(q);
      Vector3d translation = 3.0 * length;
      Transform expected{translation};
      REQUIRE(all_close(expected.rotation, result.rotation, 0, abstol));
      REQUIRE(all_close(expected.translation, result.translation, 0, abstol));
    }
    WHEN("Forward kinematics: q = 0, pi/2"){
      q << 0, pi/2;
      Transform result = kin.calculate_forward(q);
      Matrix3d rotation = ry(pi/2); // passive rotation
      Vector3d translation = 2.0 * length + rotation.transpose() * length;
      Transform expected{rotation, translation};
      //std::cout << "DEBUG:   Expected result: " << expected << std::endl;
      //std::cout << "DEBUG: Calculated result: " << result   << std::endl;
      REQUIRE(all_close(expected.rotation, result.rotation, 0, abstol));
      REQUIRE(all_close(expected.translation, result.translation, 0, abstol));
    }
    WHEN("Forward kinematics: q = pi/4, pi/4"){
      q << pi/4, pi/4;
      Transform result = kin.calculate_forward(q);
      Matrix3d rotation = ry(pi/2); // passive rotation
      Vector3d translation{1.0+std::sin(pi/4), 0.0, 1.0+std::cos(pi/4)};
      Transform expected{rotation, translation};
      REQUIRE(all_close(expected.rotation, result.rotation, 0, abstol));
      REQUIRE(all_close(expected.translation, result.translation, 0, abstol));
    }
    WHEN("Jacobian: q = pi/3, pi/5"){
      q = VectorXd::Zero(kin.num_joints());
      double q1 = pi/3.0;
      double q2 = pi/5.0;
      q << q1, q2;
      Matrix6Xd jacobian = fk.calculate_jacobian(q);
      double q12 = q1 + q2;
      Matrix6Xd J(6, kin.num_joints()); // Analytic solution for jacobian
      using namespace std;
      J <<  cos(q1) + cos(q12),  cos(q12),
                             0,         0,
           -sin(q1) - sin(q12), -sin(q12),
                             0,         0,
                             1,         1,
                             0,         0;
      THEN("Calculated jacobian is equal to analytical jacobian"){
          REQUIRE(all_close(jacobian, J, 0.0, abstol));
      }
    }
    WHEN("Forward kinematics: velocity"){
      double omega = 3.0;
      q <<  pi/2.0, 0.0;
      dq << omega, omega/2;
      Motion velocity = kin.calculate_forward_velocity(q, dq);
      THEN("Calculated carthesian velocity is equal to calulation by hand."){
          REQUIRE(all_close(velocity.angular, 1.5*omega*axis_y, 0.0, abstol));
          REQUIRE(all_close(velocity.linear, -2.5*ll*omega*axis_z, 0.0, abstol));
      }
    }
  }
  GIVEN("Simple 3D kinematic chain (wrist, ellbow, ellbow)"){
    Vector3d axis_z, axis_y, length;
    axis_y << 0, 1, 0;
    axis_z << 0, 0, 1;
    length << 0, 0, 1;
    Joint joint_ellbow{axis_y, Joint_type::Rotational};
    Joint joint_wrist {axis_z, Joint_type::Rotational};
    Joint joint_none  {axis_z, Joint_type::None      };
    //Inertia inertia{mass, length/2, Matrix3d::Identity()};
    Inertia inertia;
    Body body_ellbow(joint_ellbow, length, inertia);
    Body body_wrist (joint_wrist , length, inertia);
    Body body_end   (joint_none  , length, inertia);
    std::vector<Body> bodies;
    bodies.push_back(body_wrist );
    bodies.push_back(body_ellbow);
    bodies.push_back(body_ellbow);
    bodies.push_back(body_end   );
    Chain chain{bodies};
    Kinematics kin{chain};
    ForwardKinematics fk{chain};
    VectorXd q(chain.num_joints);
    WHEN("Forward kinematics: Home position"){
      q << 0, 0, 0;
      Transform result = kin.calculate_forward(q);
      Vector3d translation = 4.0 * length;
      Transform expected{translation};
      REQUIRE(all_close(expected.rotation, result.rotation, 0, abstol));
      REQUIRE(all_close(expected.translation, result.translation, 0, abstol));
    }
    WHEN("Forward kinematics: q = pi/5, 0, pi/3"){
      q << pi/5, 0, pi/3;
      Transform result = kin.calculate_forward(q);
      Matrix3d RZ = rz(q[0]);
      Matrix3d RY = ry(q[2]); // passive rotations
      Vector3d translation = length + RZ.transpose() * ( 2.0 * length + RY.transpose() * length);
      Transform expected{RY * RZ, translation};
      REQUIRE(all_close(expected.rotation, result.rotation, 0, abstol));
      REQUIRE(all_close(expected.translation, result.translation, 0, abstol));
    }
    WHEN("Jacobian: q = pi/3, pi/5, pi/7"){
      VectorXd q(chain.num_joints);
      double q0 = pi/3.0;
      double q1 = pi/5.0;
      double q2 = pi/7.0;
      q << q0, q1, q2;
      Matrix6Xd jacobian = fk.calculate_jacobian(q);
      // Analytic solution for jacobian for above 3D case
      Matrix6Xd J(6,3);
      using namespace std;
      double q12 = q1 + q2;
      double ss = sin(q1) + sin(q12);
      double a = -sin(q0)*ss;
      double b = cos(q0)*ss;
      double c = 0;
      double d = cos(q0)*(cos(q1) + cos(q12));
      double e = sin(q0)*(cos(q1) + cos(q12));
      double f = -sin(q1) - sin(q12);
      double g = cos(q0)*cos(q12);
      double h = sin(q0)*cos(q12);
      double j = -sin(q12);
      double k = -sin(q0);
      double l =  cos(q0);
      J <<  a, d, g,
            b, e, h,
            c, f, j,
            0, k, k,
            0, l, l,
            1, 0, 0;
      //std::cout << "DEBUG:   Expected result:\n" << J        << std::endl;
      //std::cout << "DEBUG: Calculated result:\n" << jacobian << std::endl;
      THEN("Calculated jacobian is equal to analytical jacobian"){
        REQUIRE(all_close(jacobian, J, 0.0, abstol));
      }
    }
    WHEN("Inverse kinematics: q = pi/5, 0, pi/3, q_start = 0, 0 ,0"){
      q << pi/5, 0, pi/3;
      VectorXd q_start = Vector3d::Zero();
      Transform target = kin.calculate_forward(q);
      IK_Result result = kin.calculate_inverse(target, q_start);
      //std::cout << "DEBUG:   Expected result: " <<        q.transpose() << std::endl;
      //std::cout << "DEBUG: Calculated result: " << result.q.transpose() << std::endl;
      //std::cout << "DEBUG:             Error: " << result.error         << std::endl;
      Transform tip_calc = kin.calculate_forward(result.q);
      REQUIRE(all_close(target.translation, tip_calc.translation, 0, 1e-5));
      REQUIRE(all_close(target.rotation   , tip_calc.rotation,    0, 1e-5));
    }
  }
  GIVEN("PRob2 on linear axis kinematic chain \n" +
        "(trans, wrist, ellbow, ellbow, wrist, ellbow, wrist) & gravity\n" +
        "Same parameters for all links."){
    double mass = 1.2;
    double gg = 9.8;
    double ll = 1.0;
    Vector3d axis_z, axis_y, length, grav;
    axis_y << 0, 1, 0;
    axis_z << 0, 0, 1;
    length << 0, 0, ll;
    grav   << 0, 0, -gg;
    Joint joint_trans {axis_y, Joint_type::Translational};
    Joint joint_ellbow{axis_y, Joint_type::Rotational   };
    Joint joint_wrist {axis_z, Joint_type::Rotational   };
    Joint joint_none  {axis_z, Joint_type::None         };
    Inertia inertia{mass, length/2, Matrix3d::Identity()};
    Body body_trans (joint_trans,  length, inertia);
    Body body_ellbow(joint_ellbow, length, inertia);
    Body body_wrist (joint_wrist,  length, inertia);
    Body body_end   (joint_none,   length, inertia);
    std::vector<Body> bodies;
    bodies.push_back(body_trans );
    bodies.push_back(body_wrist );
    bodies.push_back(body_ellbow);
    bodies.push_back(body_ellbow);
    bodies.push_back(body_wrist );
    bodies.push_back(body_ellbow);
    bodies.push_back(body_wrist );
    bodies.push_back(body_end   );
    Chain chain{bodies};
    Kinematics kin{chain};
    VectorXd q(chain.num_joints);
    WHEN("Forward kinematics: Home position"){
      q << VectorXd::Zero(7);
      Transform result = kin.calculate_forward(q);
      Vector3d translation = 8.0 * length;
      Transform expected{translation};
      REQUIRE(all_close(expected.rotation, result.rotation, 0, abstol));
      REQUIRE(all_close(expected.translation, result.translation, 0, abstol));
    }
    WHEN("Forward kinematics: q = 1, pi/4, 0, 0, 0, 0, 0"){
      q << 1.0, pi/4, 0, 0 ,0 ,0 ,0;
      Transform result = kin.calculate_forward(q);
      Vector3d translation = 8.0 * length + Vector3d(0, 1, 0);
      Matrix3d rotation = rz(q[1]);
      Transform expected{rotation, translation};
      REQUIRE(all_close(expected.rotation, result.rotation, 0, abstol));
      REQUIRE(all_close(expected.translation, result.translation, 0, abstol));
    }
    WHEN("Inverse kinematics: q= 1.0, pi/4, pi/3, 0, 0, 0, pi/5;"){
      q << 1.0, pi/4, pi/3, 0 ,0 ,0 , pi/5;
      VectorXd q_start = VectorXd::Constant(kin.num_joints(), -0.2);
      Transform target = kin.calculate_forward(q);
      IK_Result result = kin.calculate_inverse(target, q_start);
      //std::cout << "DEBUG:   Expected result: " <<        q.transpose() << std::endl;
      //std::cout << "DEBUG: Calculated result: " << result.q.transpose() << std::endl;
      //std::cout << "DEBUG:             Error: " << result.error         << std::endl;
      Transform tip_calc = kin.calculate_forward(result.q);
      REQUIRE(all_close(target.translation, tip_calc.translation, 0, 1e-5));
      REQUIRE(all_close(target.rotation   , tip_calc.rotation,    0, 1e-5));
    }
  }
}
