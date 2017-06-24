#define CATCH_CONFIG_MAIN

#include "../include/kinematics.hpp"
#include "../include/dynamics.hpp"
#include "../lib/catch.hpp"

#include <Eigen/Dense>

constexpr auto pi = 3.141592653589793238462643383279502884L;

using namespace robo;

// Helper functions
void compare_vectors_approx(Eigen::Vector3d a, Eigen::Vector3d b, double eps=1e-15){
	for(int i=0; i<3; ++i){
            		REQUIRE(a[i] == Approx(b[i]).epsilon(eps));
            	}
}

SCENARIO("Joint tests"){
    GIVEN("y and z axis and a frame"){
        Eigen::Vector3d axis_z, axis_y;
		axis_y << 0.0, 1.0, 0.0;
		axis_z << 0.0, 0.0, 1.0;
		Frame f = Frame();
        SECTION("Joint creation"){
        	Joint joint_trans = Joint(0, f, axis_z, JointType::Translational);
        	Joint joint_ellbow = Joint(0, f, axis_y, JointType::Rotational);
			Joint joint_wrist = Joint(0, f, axis_z, JointType::Rotational);
			Joint joint_none = Joint(0, f, axis_z, JointType::None);

			REQUIRE(joint_trans.pose(0.0).origin == Eigen::Vector3d::Zero());
			REQUIRE(joint_ellbow.pose(0.0).origin == Eigen::Vector3d::Zero());
			REQUIRE(joint_wrist.pose(0.0).origin == Eigen::Vector3d::Zero());
			REQUIRE(joint_none.pose(0.0).origin == Eigen::Vector3d::Zero());

			REQUIRE(joint_trans.pose(1.0).origin == Eigen::Vector3d(0.0, 0.0, 1.0));
			REQUIRE(joint_ellbow.pose(1.0).origin == Eigen::Vector3d::Zero());
			REQUIRE(joint_wrist.pose(1.0).origin == Eigen::Vector3d::Zero());
			REQUIRE(joint_none.pose(1.0).origin == Eigen::Vector3d::Zero());
        }
    }
}


SCENARIO("Kinematics tests"){
    GIVEN("6-axis robot manipulator (none, wrist, ellbow, ellbow, wrist, ellbow, wrist). Links each 1m."){
        Eigen::Vector3d axis_z, axis_y;
		axis_y << 0.0, 1.0, 0.0;
		axis_z << 0.0, 0.0, 1.0;
		Frame f = Frame();
		Joint joint_ellbow = Joint(0, f, axis_y, JointType::Rotational);
		Joint joint_wrist = Joint(0, f, axis_z, JointType::Rotational);
		Joint joint_none = Joint(0, f, axis_z, JointType::None);
		Eigen::Vector3d length;
		length << 0.0, 0.0, 1.0;
		Frame tip = Frame(length);
		Inertia inertia = Inertia(1.0, length/2);

		Link link_0 = Link(0, joint_none, tip, inertia);
		Link link_1 = Link(1, joint_wrist, tip, inertia);
		Link link_2 = Link(2, joint_ellbow, tip, inertia);
		Link link_3 = Link(3, joint_ellbow, tip, inertia);
		Link link_4 = Link(4, joint_wrist, tip, inertia);
		Link link_5 = Link(5, joint_ellbow, tip, inertia);
		Link link_6 = Link(6, joint_wrist, tip, inertia);
		
		Chain chain;
		chain.addLink(link_0);
		chain.addLink(link_1);
		chain.addLink(link_2);
		chain.addLink(link_3);
		chain.addLink(link_4);
		chain.addLink(link_5);
		chain.addLink(link_6);

		Kinematics kin = Kinematics(chain);
 		Eigen::VectorXd q(chain.nr_joints);

        WHEN("Forward kinematics: Home position"){
            q = Eigen::VectorXd::Zero(chain.nr_joints);
            kin.joint_to_cartesian(q);
            Frame res = kin.f_end;
            THEN("Endeffector at (0, 0, 7)"){
                REQUIRE(res.origin == Eigen::Vector3d(0.0, 0.0, 7.0));
            }
            THEN("Endeffector Orientation Zero"){
            	REQUIRE(res.orientation == Eigen::Matrix3d::Identity());
            }
        }
        WHEN("Forward kinematics: Joint 2 at 90°"){
        	q = Eigen::VectorXd::Zero(chain.nr_joints);
        	q[1] = pi/2.0;
            kin.joint_to_cartesian(q);
            Frame res = kin.f_end;
            THEN("Endeffector at (5, 0, 2)"){
                REQUIRE(res.origin == Eigen::Vector3d(5.0, 0.0, 2.0));
            }
        }
        WHEN("Forward kinematics: Joint 1 & 2 at 90°"){
        	q = Eigen::VectorXd::Zero(chain.nr_joints);
        	q[0] = pi/2.0;
        	q[1] = pi/2.0;
            kin.joint_to_cartesian(q);
            Frame res = kin.f_end;
            THEN("Endeffector at (5, 0, 2)"){
            	Eigen::Vector3d position(0.0, 5.0, 2.0);
            	compare_vectors_approx(res.origin, position);
            }
        }
        WHEN("Inverse kinematics: Target (0, 5, 2)"){
            q = Eigen::VectorXd::Zero(chain.nr_joints);
            q[0] = pi/2.0;
            q[1] = pi/2.0;
            Eigen::VectorXd q_init = Eigen::VectorXd::Zero(chain.nr_joints);
            kin.joint_to_cartesian(q);
            Frame target = kin.f_end;
            int error = kin.cartesian_to_joint(target, q_init);
            Eigen::VectorXd res = kin.q_out;
            kin.joint_to_cartesian(res);
            Frame check = kin.f_end;
            THEN("No error"){
                REQUIRE(error == 1);
            }

            THEN("Result of IK  yields almost same point through FK"){
                compare_vectors_approx(check.origin, target.origin, 1e-5);
            }
        } 
    }
}

