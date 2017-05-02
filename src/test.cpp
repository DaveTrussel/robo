#include "../include/chain.hpp"
#include "../include/link.hpp"
#include "../include/joint.hpp"
#include "../include/frame.hpp"
#include "../include/forward_kinematics.hpp"

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

int main () {
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
 	
 	ForwardKinematics fk = ForwardKinematics(chain);
 	Eigen::VectorXd q(chain.nr_joints);
 	q << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
 	std::vector<Frame> f_out(chain.nr_links);
 	TimePoint tic = now();
 	fk.joint2cartesian(q, f_out);
 	TimePoint toc = now();
 	auto duration = duration_cast<microseconds>( toc - tic ).count();
 	cout << "Solved forward kinematics in: " << duration << " Microseconds." << endl;
 	cout << "Frame at end of robot chain:" << endl << f_out.at(chain.nr_links-1).origin << endl << f_out.at(chain.nr_links-1).orientation << endl;
 	cout << endl << "As a homogeneous matrix:" << endl << f_out.at(chain.nr_links-1).as_homogeneous_matrix() << endl;
 	cout << endl << "It's nautical_angles:" << endl << f_out.at(chain.nr_links-1).nautical_angles() << endl;
 } 
