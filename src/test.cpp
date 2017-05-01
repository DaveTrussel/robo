#include "../include/chain.hpp"
#include "../include/link.hpp"
#include "../include/joint.hpp"
#include "../include/frame.hpp"
#include "../include/forward_kinematics.hpp"

#include <Eigen/Dense>

#include <iostream>
#include <string>
#include <vector>

using namespace robo;
using namespace std;

int main () {
	Eigen::Vector3d axis;
	axis << 0.0, 0.0, 1.0;
	Frame f = Frame();
	Joint joint_1 = Joint(1, f, axis, JointType::Rotational); 
	Eigen::Vector3d length;
	length << 1.0, 1.0, 1.0;
	Frame tip = Frame(length);
	Link link_01 = Link(1, joint_1, tip);
	Chain chain;
	chain.addLink(link_01);
	Frame turned = joint_1.pose(1.0);
	cout << "DEBUG turned frame: " << endl << turned.origin << endl << turned.orientation << endl;
 	
 	ForwardKinematics fk = ForwardKinematics(chain);
 	Eigen::VectorXd q;
 	q << 1.5;
 	std::vector<Frame> f_out(1);
 	fk.joint2cartesian(q, f_out)
 } 
