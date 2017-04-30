#include "../include/chain.hpp"
#include "../include/link.hpp"
#include "../include/joint.hpp"
#include "../include/frame.hpp"
#include "../include/forward_kinematics.hpp"

#include <Eigen/Dense>


int main () {
	Eigen::Vecotr3d axis << 0.0, 0.0, 1.0;
	Frame f = Frame();
	Joint joint_1 = Joint(1, f, axis, JointType::Rotational); 
	Eigen::Vecotr3d length << 1.0, 1.0, 1.0;
	Frame tip = Frame(length);
	Link link_01 = Link(1, joint_1, tip);
 } 