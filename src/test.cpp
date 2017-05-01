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
	Eigen::Vector3d axis;
	axis << 0.0, 0.0, 1.0;
	Frame f = Frame();
	Joint joint_1 = Joint(1, f, axis, JointType::Rotational);
	Joint joint_2 = Joint(2, f, axis, JointType::Rotational); 
	Eigen::Vector3d length;
	length << 1.0, 1.0, 1.0;
	Frame tip = Frame(length);
	Link link_1 = Link(1, joint_1, tip);
	Link link_2 = Link(2, joint_2, tip);
	Chain chain;
	chain.addLink(link_1);
	chain.addLink(link_2);
	Frame turned = joint_1.pose(1.0);
	cout << "DEBUG turned frame joint: " << endl << turned.origin << endl << turned.orientation << endl;
	turned = link_1.pose(1.0);
	cout << "DEBUG turned frame link tip: " << endl << turned.origin << endl << turned.orientation << endl;
 	
 	ForwardKinematics fk = ForwardKinematics(chain);
 	cout << "I have forward kinematics." << endl;
 	Eigen::VectorXd q(chain.nr_joints);
 	q << 1.5, 0.5;
 	cout << "q: " << q << endl;
 	std::vector<Frame> f_out(chain.nr_links);
 	TimePoint tic = now();
 	fk.joint2cartesian(q, f_out);
 	TimePoint toc = now();
 	auto duration = duration_cast<microseconds>( toc - tic ).count();
 	cout << "Solved forward kinematics in: " << duration << " Microseconds." << endl;
 	cout << "Frame at end of robot chain:" << endl << f_out.at(chain.nr_links-1).origin << endl << f_out.at(chain.nr_links-1).orientation << endl;
 	cout << "And as homogeneous matrix:" << endl << f_out.at(chain.nr_links-1).as_homogeneous_matrix() << endl;
 } 
