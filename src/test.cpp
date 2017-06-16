#include "../include/chain.hpp"
#include "../include/link.hpp"
#include "../include/joint.hpp"
#include "../include/frame.hpp"
#include "../include/kinematics.hpp"

#include <Eigen/Dense>

#include <iostream>
#include <string>
#include <vector>
#include <chrono>

using namespace robo;
using namespace std;
using namespace std::chrono;

#define now() high_resolution_clock::now()
typedef high_resolution_clock::time_point TimePoint;

double my_rand(){
    int r = std::rand() % 314;
    return ((double)r)/100.0 - 1.57;
}

int main () {
    std::srand(std::time(0));

	Eigen::Vector3d axis_z, axis_y;

	axis_y << 0.0, 1.0, 0.0;
	axis_z << 0.0, 0.0, 1.0;
	
	Frame f = Frame();
	
	Joint joint_ellbow = Joint(0, f, axis_y, JointType::Rotational);
	Joint joint_wrist = Joint(0, f, axis_z, JointType::Rotational);
	Joint joint_none = Joint(0, f, axis_z, JointType::None);

	// test twist
	Twist twist(axis_y, axis_z);
	twist = twist*3.0;
	cout << "Twist Test:" << endl 
		 << "twist.linear=" << endl << twist.linear << endl 
		 << "twist.rotation=" << endl << twist.rotation << endl
		 << "Functions: " << endl
		 << rotate_twist(joint_ellbow.pose(0.5).orientation, twist).rotation << endl
		 << change_twist_reference(twist, axis_y).linear << endl <<endl;;

	cout << "Joint Test:" << endl 
		 << "Pose(0.5)=" << endl << joint_ellbow.pose(0.5).origin << endl 
		 << joint_ellbow.pose(0.5).orientation << endl 
		 << " Twist(0.5)=" << endl << joint_ellbow.twist(0.5).linear << endl 
		 << joint_ellbow.twist(0.5).rotation << endl << endl;;

	
	Eigen::Vector3d length;
	length << 0.0, 0.0, 0.5;
	Frame tip = Frame(length);
    Frame i_want_a_copy;
    i_want_a_copy = tip;
	
	Link link_0 = Link(0, joint_none, tip);
	Link link_1 = Link(1, joint_wrist, tip);
	Link link_2 = Link(2, joint_ellbow, tip);
	Link link_3 = Link(3, joint_ellbow, tip);
	Link link_4 = Link(4, joint_wrist, tip);
	Link link_5 = Link(5, joint_ellbow, tip);
	Link link_6 = Link(6, joint_wrist, tip);
	
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
 	q << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
 	Eigen::VectorXd dq(chain.nr_joints);
 	dq << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;


 	TimePoint tic = now();
 	kin.joint_to_cartesian(q);
 	TimePoint toc = now();
 	auto duration = duration_cast<microseconds>( toc - tic ).count();
 	cout << "Kinematics Test:" << endl 
 		 << "Solved forward kinematics in: " << duration << " Microseconds." << endl
 	     << "Frame at end of robot chain:" << endl << kin.f_end.origin << endl << kin.f_end.orientation << endl << endl
 	     << "As a homogeneous matrix:" << endl << kin.f_end.as_homogeneous_matrix() << endl << endl
 	     << "It's nautical_angles:" << endl << kin.f_end.nautical_angles() << endl;

    kin.calculate_jacobian(q);
    cout << "Calculated jacobian: " << endl << kin.jacobian << endl;

 	Frame f_target = kin.f_end;
 	Eigen::VectorXd q_init(chain.nr_joints);
    q_init << my_rand(), my_rand(), my_rand(), my_rand(), my_rand(), my_rand();
 	tic = now();
 	kin.joint_to_cartesian(q_init);
 	int error_code = kin.cartesian_to_joint(f_target, q_init);
 	toc = now();
 	duration = duration_cast<microseconds>( toc - tic ).count();
 	cout << "Solved inverse kinematics in: " << duration << " Microseconds." << endl;
 	cout << "Robot joint positions:" << endl << kin.q_out << endl;
 	cout << "With error code:" << endl << error_code << endl;
    kin.joint_to_cartesian(kin.q_out);
    cout << "Corresponding forward postion: " << endl << kin.f_end.origin << endl << kin.f_end.orientation << endl;
 } 
