#define CATCH_CONFIG_MAIN

#include "kinematics.hpp"
#include "dynamics.hpp"
#include "../lib/catch.hpp" // unit testing framework (header only)

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>


constexpr auto pi = 3.141592653589793238462643383279502884L;
// absolute tolerance for all tests
constexpr auto abstol = 1e-12;

using namespace robo;
using namespace std;

// Helper functions
template<typename DerivedA, typename DerivedB>
bool allclose(const Eigen::DenseBase<DerivedA>& a,
              const Eigen::DenseBase<DerivedB>& b,
              const typename DerivedA::RealScalar& rtol
                  = Eigen::NumTraits<typename DerivedA::RealScalar>::dummy_precision(),
              const typename DerivedA::RealScalar& atol
                  = Eigen::NumTraits<typename DerivedA::RealScalar>::epsilon())
{
  return ((a.derived() - b.derived()).array().abs()
          <= (atol + rtol * b.derived().array().abs())).all();
}

// Test scenarios
SCENARIO("Joint tests"){
    GIVEN("y and z axis and a frame"){
        cout << std::fixed << std::setw( 12 ) << std::setprecision(2);
        Vector3d axis_z, axis_y;
        axis_y << 0.0, 1.0, 0.0;
        axis_z << 0.0, 0.0, 1.0;
        Frame f = Frame();
        SECTION("Joint creation"){
            Joint joint_trans = Joint(0, f, axis_z, Joint_type::Translational);
            Joint joint_ellbow = Joint(0, f, axis_y, Joint_type::Rotational);
            Joint joint_wrist = Joint(0, f, axis_z, Joint_type::Rotational, -170.0/pi, 170.0/pi);
            Joint joint_none = Joint(0, f, axis_z, Joint_type::None);

            REQUIRE(joint_trans.pose(0.0).origin == Vector3d::Zero());
            REQUIRE(joint_ellbow.pose(0.0).origin == Vector3d::Zero());
            REQUIRE(joint_wrist.pose(0.0).origin == Vector3d::Zero());
            REQUIRE(joint_none.pose(0.0).origin == Vector3d::Zero());

            REQUIRE(joint_trans.pose(1.0).origin == Vector3d(0.0, 0.0, 1.0));
            REQUIRE(joint_ellbow.pose(1.0).origin == Vector3d::Zero());
            REQUIRE(joint_wrist.pose(1.0).origin == Vector3d::Zero());
            REQUIRE(joint_none.pose(1.0).origin == Vector3d::Zero());
        }
    }
}

SCENARIO("Frame tests"){
    GIVEN("Two frames"){
        Vector3d pos_a, pos_b, axis_y, axis_z;
        pos_a << 0, 0, 1;
        pos_b << 0, 0, 3;
        axis_y << 0, 1, 0;
        axis_z << 0, 0, 1;

        SECTION("Difference of two frames (1)"){
            Frame f = Frame(pos_b);
            Joint joint = Joint(0, f, axis_z, Joint_type::Rotational);
            Frame f_a(pos_a);
            Frame f_b = joint.pose(1.337);
            Vector6d diff = f_b - f_a;
            Vector6d diff_expected;
            diff_expected << 0, 0, 2, 0, 0, 1.337;
            REQUIRE(diff == diff_expected);
        }

        SECTION("Difference of two frames (2)"){
            Frame f = Frame(pos_b);
            Joint joint = Joint(0, f, axis_y, Joint_type::Rotational);
            Frame f_a(pos_a);
            Frame f_b = joint.pose(0.42);
            Vector6d diff = f_b - f_a;
            Vector6d diff_expected;
            diff_expected << 0, 0, 2, 0, 0.42, 0;
            REQUIRE(diff == diff_expected);
        }
    }
}

SCENARIO("Kinematics tests Basic (2D)"){
    GIVEN("2-axis robot manipulator (ellbow, ellbow). Links each 1m long."){
        Vector3d axis_z, axis_y;
        axis_y << 0.0, 1.0, 0.0;
        Frame f = Frame();
        Joint joint_ellbow = Joint(0, f, axis_y, Joint_type::Rotational);
        Vector3d length;
        length << 0.0, 0.0, 1.0;
        Frame tip = Frame(length);
        Inertia inertia = Inertia(1.0, length/2);

        Link link_1 = Link(1, joint_ellbow, tip, inertia);
        Link link_2 = Link(2, joint_ellbow, tip, inertia);

        Chain chain;
        chain.add_link(link_1);
        chain.add_link(link_2);

        Kinematics kin = Kinematics(chain);
        VectorXd q(chain.nr_joints);

        WHEN("Forward kinematics: Home position"){
            q = VectorXd::Zero(chain.nr_joints);
            kin.joint_to_cartesian(q);
            Frame res = kin.f_end;
            THEN("Endeffector at (0, 0, 2)"){
                REQUIRE(res.origin == Vector3d(0.0, 0.0, 2.0));
            }
            THEN("Endeffector Orientation Zero"){
                REQUIRE(res.orientation == Matrix3d::Identity());
            }
        }
        WHEN("Forward kinematics: Joint 2 at 90°"){
            q = VectorXd::Zero(chain.nr_joints);
            q[1] = pi/2.0;
            kin.joint_to_cartesian(q);
            Frame res = kin.f_end;
            THEN("Endeffector at (1, 0, 1)"){
                REQUIRE(allclose(res.origin, Vector3d(1.0, 0.0, 1.0), 0.0, abstol));
            }
            THEN("Endeffector orientation +90° around y-axis of world frame"){
                Matrix3d R;
                double q1 = pi/2;
                R <<  cos(q1), 0, sin(q1),
                            0, 1,       0,
                     -sin(q1), 0, cos(q1);
                REQUIRE(allclose(res.orientation, R, 0.0, abstol));
            }
        }
        WHEN("Forward kinematics: Joint 1&2 at 45°"){
            q = VectorXd::Zero(chain.nr_joints);
            double q1 = pi/4.0;
            double q2 = pi/4.0;
            q << q1, q2;
            kin.joint_to_cartesian(q);
            kin.calculate_jacobian(q);
            double q12 = q1 + q2;
            MatrixXd J(6,2); // Analytic solution for jacobian
            J <<  cos(q1) + cos(q12),  cos(q12),
                                   0,         0,
                 -sin(q1) - sin(q12), -sin(q12),
                                   0,         0,
                                   1,         1,
                                   0,         0;
            THEN("Calculated jacobian is equal to analytical jacobian"){
                REQUIRE(allclose(kin.jacobian, J, 0.0, abstol));
            }
        }
    }
}


SCENARIO("Kinematics tests Basic (Simple 3D)"){
    GIVEN("3-axis robot manipulator (wrist, ellbow, ellbow). Links each 1m long."){
    Vector3d axis_z, axis_y;
    axis_y << 0.0, 1.0, 0.0;
    axis_z << 0.0, 0.0, 1.0;
    Frame ff = Frame();
    double q_min = -170.0/180*pi;
    double q_max = 170.0/180*pi;
    Joint joint_ellbow = Joint(0, ff, axis_y, Joint_type::Rotational, q_min, q_max);
    Joint joint_wrist = Joint(0, ff, axis_z, Joint_type::Rotational, q_min, q_max);
    Vector3d length;
    length << 0, 0, 1;
    Frame tip = Frame(length);
    Inertia inertia = Inertia(1.0, length/2);
    Link link_0 = Link(0, joint_wrist,  tip, inertia);
    Link link_1 = Link(0, joint_ellbow, tip, inertia);
    Link link_2 = Link(1, joint_ellbow, tip, inertia);
    Chain chain;
    chain.add_link(link_0);
    chain.add_link(link_1);
    chain.add_link(link_2);
    int max_iter = 500;
    Kinematics kin = Kinematics(chain, max_iter);
        WHEN("Forward kinematics: Some position"){
            VectorXd q(chain.nr_joints);
            double q0 = pi/3.0;
            double q1 = pi/4.0;
            double q2 = pi/1.5;
            q << q0, q1, q2;
            kin.joint_to_cartesian(q);
            kin.calculate_jacobian(q);
            // Analytic solution for jacobian for above 2D case
            MatrixXd J(6,3);
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
            THEN("Calculated jacobian is equal to analytical jacobian"){
                REQUIRE(allclose(kin.jacobian, J, 0.0, abstol));
            }
        }
    }
}


SCENARIO("Kinematics tests Advanced"){
    GIVEN("6-axis robot manipulator (none, wrist, ellbow, ellbow, wrist, ellbow, wrist). Links each 1m."){
        Vector3d axis_z, axis_y;
        axis_y << 0.0, 1.0, 0.0;
        axis_z << 0.0, 0.0, 1.0;
        Frame f = Frame();
        Joint joint_ellbow = Joint(0, f, axis_y, Joint_type::Rotational);
        Joint joint_wrist = Joint(0, f, axis_z, Joint_type::Rotational);
        Joint joint_none = Joint(0, f, axis_z, Joint_type::None);
        Vector3d length;
        length << 0.0, 0.0, 1.0;
        Frame tip = Frame(length);
        Inertia inertia = Inertia(1.0, length/2, Matrix3d::Identity());

        Link link_0 = Link(0, joint_none, tip, inertia);
        Link link_1 = Link(1, joint_wrist, tip, inertia);
        Link link_2 = Link(2, joint_ellbow, tip, inertia);
        Link link_3 = Link(3, joint_ellbow, tip, inertia);
        Link link_4 = Link(4, joint_wrist, tip, inertia);
        Link link_5 = Link(5, joint_ellbow, tip, inertia);
        Link link_6 = Link(6, joint_wrist, tip, inertia);
        
        Chain chain;
        chain.add_link(link_0);
        chain.add_link(link_1);
        chain.add_link(link_2);
        chain.add_link(link_3);
        chain.add_link(link_4);
        chain.add_link(link_5);
        chain.add_link(link_6);

        Kinematics kin = Kinematics(chain);
        VectorXd q(chain.nr_joints);

        WHEN("Forward kinematics: Home position"){
            q = VectorXd::Zero(chain.nr_joints);
            kin.joint_to_cartesian(q);
            Frame res = kin.f_end;
            THEN("Endeffector at (0, 0, 7)"){
                REQUIRE(res.origin == Vector3d(0.0, 0.0, 7.0));
            }
            THEN("Endeffector Orientation Zero"){
                REQUIRE(res.orientation == Matrix3d::Identity());
            }
        }
        WHEN("Forward kinematics: Joint 2 at 90°"){
            q = VectorXd::Zero(chain.nr_joints);
            q[1] = pi/2.0;
            kin.joint_to_cartesian(q);
            Frame res = kin.f_end;
            THEN("Endeffector at (5, 0, 2)"){
                REQUIRE(res.origin == Vector3d(5.0, 0.0, 2.0));
            }
            THEN("Endeffector orientation +90° around y-axis of world frame"){
                Matrix3d R;
                double q1 = pi/2;
                R <<  cos(q1), 0, sin(q1),
                            0, 1,       0,
                     -sin(q1), 0, cos(q1);
                REQUIRE(allclose(res.orientation, R, 0.0, abstol));
            }
        }
        WHEN("Forward kinematics: Joint 1 & 2 at 90°"){
            q = VectorXd::Zero(chain.nr_joints);
            q[0] = pi/2.0;
            q[1] = pi/2.0;
            kin.joint_to_cartesian(q);
            Frame res = kin.f_end;
            THEN("Endeffector at (0, 5, 2)"){
                Vector3d position(0.0, 5.0, 2.0);
                allclose(res.origin, position, 0.0, abstol);
            }
        }
        WHEN("Inverse kinematics: Target (0, 5, 2)"){
            q = VectorXd::Zero(chain.nr_joints);
            q[0] = pi/2.0;
            q[1] = pi/2.0;
            VectorXd q_init = VectorXd::Zero(chain.nr_joints);
            kin.joint_to_cartesian(q);
            Frame target = kin.f_end;
            Error_type error = kin.cartesian_to_joint(target, q_init);
            VectorXd res = kin.q_out;
            kin.joint_to_cartesian(res);
            Frame check = kin.f_end;
            THEN("Result of IK yields almost same point through FK"){
                allclose(check.origin, target.origin, 0.0, abstol);
            }
            THEN("Result of IK yields also same orientation"){
                allclose(check.orientation, target.orientation, 0.0, abstol);
            }
            THEN("No error"){
                REQUIRE(error == Error_type::no_error);
            }
        } 
    }
}

SCENARIO("Dynamics tests"){
    GIVEN("6-axis robot manipulator (none, wrist, ellbow, ellbow, wrist, ellbow, wrist). Links each 1m."){
        Vector3d axis_z, axis_y;
        axis_y << 0.0, 1.0, 0.0;
        axis_z << 0.0, 0.0, 1.0;
        Frame f = Frame();
        Joint joint_ellbow = Joint(0, f, axis_y, Joint_type::Rotational);
        Joint joint_wrist = Joint(0, f, axis_z, Joint_type::Rotational);
        Joint joint_none = Joint(0, f, axis_z, Joint_type::None);
        Vector3d length;
        length << 0.0, 0.0, 1.0;
        Frame tip = Frame(length);
        Inertia inertia = Inertia(1.0, length/2, Matrix3d::Identity());

        Link link_0 = Link(0, joint_none, tip, inertia);
        Link link_1 = Link(1, joint_wrist, tip, inertia);
        Link link_2 = Link(2, joint_ellbow, tip, inertia);
        Link link_3 = Link(3, joint_ellbow, tip, inertia);
        Link link_4 = Link(4, joint_wrist, tip, inertia);
        Link link_5 = Link(5, joint_ellbow, tip, inertia);
        Link link_6 = Link(6, joint_wrist, tip, inertia);
        
        Chain chain;
        chain.add_link(link_0);
        chain.add_link(link_1);
        chain.add_link(link_2);
        chain.add_link(link_3);
        chain.add_link(link_4);
        chain.add_link(link_5);
        chain.add_link(link_6);

        Dynamics dyn = Dynamics(chain);

        VectorXd q   = VectorXd::Zero(chain.nr_joints);
        VectorXd dq  = VectorXd::Zero(chain.nr_joints);
        VectorXd ddq = VectorXd::Zero(chain.nr_joints);
        double g = 9.81;
        Vector3d grav;
        grav << 0.0, 0.0, -g;

        WHEN("Inverse Dynamics: Static Home position"){
            q = VectorXd::Zero(chain.nr_joints);
            dyn.calculate_generalized_forces(q, grav);
            VectorXd res = dyn.joint_torques;
            THEN("No torques at home position"){
                REQUIRE(res == VectorXd::Zero(chain.nr_joints));
            }
        }

        WHEN("Inverse Dynamics: Static J5 at 90°"){
            q << 0.0, 0, 0, 0.0, pi/2.0, 0;
            dyn.calculate_generalized_forces(q, grav);
            VectorXd res = dyn.joint_torques;
            THEN("Equal torques in J2, J3 and J5"){
                VectorXd compare(chain.nr_joints);
                compare << 0.0, 2*g, 2*g, 0.0, 2*g, 0.0;
                REQUIRE(allclose(res, compare, 0.0, abstol));
            }
        }

        WHEN("Inverse Dynamics: Static J1,J4 and J6 at 90°"){
            q << pi/2, 0, 0, pi/2, 0, pi/2;
            dyn.calculate_generalized_forces(q, grav);
            VectorXd res = dyn.joint_torques;
            THEN("Zero torques in all joints"){
                VectorXd compare(chain.nr_joints);
                compare << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                REQUIRE(allclose(res, compare, 0.0, abstol));
            }
        }

        WHEN("Inverse Dynamics: Static J5 at 90° and constant velocities"){
            q << 0.0, 0, 0, 0.0, pi/2.0, 0;
            dq = VectorXd::Constant(chain.nr_joints, 4.2);
            dyn.calculate_generalized_forces(q, grav);
            VectorXd res = dyn.joint_torques;
            THEN("Constant velocities does not change anything compared to static case."){
                VectorXd compare(chain.nr_joints);
                compare << 0.0, 2*g, 2*g, 0.0, 2*g, 0.0;
                REQUIRE(allclose(res, compare, 0.0, abstol));
            }
        }

        WHEN("Inverse Dynamics: J6 accelerating"){
            q = VectorXd::Zero(chain.nr_joints);
            dq = VectorXd::Zero(chain.nr_joints);
            double acc = 1.337;
            ddq << 0, 0, 0, 0, 0, acc;
            dyn.calculate_generalized_forces(q, dq, ddq, grav);
            VectorXd res = dyn.joint_torques;
            THEN("Equal torques in all wrist joints"){
                VectorXd compare(chain.nr_joints);
                compare << acc, 0.0, 0.0, acc, 0.0, acc;
                REQUIRE(allclose(res, compare, 0.0, abstol));
            }
        }

        WHEN("Inverse Dynamics: J1 accelerating"){
            q = VectorXd::Zero(chain.nr_joints);
            dq = VectorXd::Zero(chain.nr_joints);
            double acc = 4.2;
            ddq << acc, 0, 0, 0, 0, 0;
            dyn.calculate_generalized_forces(q, dq, ddq, grav);
            VectorXd res = dyn.joint_torques;
            THEN("Torques in each wrist joint equal to inertia*acc of rest of chain"){
                VectorXd compare(chain.nr_joints);
                compare << 6*acc, 0.0, 0.0, 3*acc, 0.0, acc;
                REQUIRE(allclose(res, compare, 0.0, abstol));
            }
        }

    }
}

