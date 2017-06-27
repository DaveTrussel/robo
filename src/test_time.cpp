#define EIGEN_NO_MALLOC
// according to Eigen documentation this macro should not be used in "real-world code"
// however we really dont want any calls to malloc because we are concerned about real-time issues
// results in assertion failure when malloc is called by Eigen.

#include "../include/kinematics.hpp"
#include "../include/dynamics.hpp"

#include <Eigen/Dense>

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <algorithm>
#include <random>

using namespace robo;
using namespace std;
using namespace std::chrono;

constexpr int nr_runs = 1e5;
constexpr auto pi = 3.141592653589793238462643383279502884L;

// helper functions
#define now() high_resolution_clock::now()

VectorXd rand_joint_vector(int size, double min, double max){
	random_device rd;
	mt19937 gen(rd());
	uniform_real_distribution<> distri(min, max);
	VectorXd vec(size);
	for(int i=0; i<size; ++i){
		vec[i] = distri(gen);
	}
    return vec;
}

void print_timing_result(vector<double> timings){
	double sum = 0;
	double median = 0;
	size_t len = timings.size();
	sort(timings.begin(), timings.end());
	double max = timings.back();
	double min = timings.front();
	for (double &dt: timings) {
    	sum += dt;
  	}
  	double average = sum / len;
  	if(len % 2 == 0){
 		median = (timings[len/2] + timings[len/2-1]) / 2.0;
  	}
  	else{
  		median = timings[len/2];
  	}
  	cout << "Timing results (in microseconds) from " << len << " runs:"<< endl
  		 << "Average: " << average << endl
  		 << "Median: " << median << endl
  		 << "Min: " << min << endl
  		 << "Max: " << max << endl;
}


int main () {
    std::srand(std::time(0));

	Vector3d axis_z, axis_y;

	axis_y << 0.0, 1.0, 0.0;
	axis_z << 0.0, 0.0, 1.0;
	
	Frame f = Frame();

	double q_min = -160.0/pi;
	double q_max = 160.0/pi;

	Joint joint_ellbow = Joint(0, f, axis_y, JointType::Rotational, q_min, q_max);
	Joint joint_wrist = Joint(0, f, axis_z, JointType::Rotational, q_min, q_max);
	Joint joint_none = Joint(0, f, axis_z, JointType::None);

	Vector3d length;
	length << 0.0, 0.0, 0.5;
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
 	Dynamics dyn = Dynamics(chain);
 	VectorXd q(chain.nr_joints);
 	VectorXd dq(chain.nr_joints);
 	VectorXd ddq(chain.nr_joints);
 	VectorXd q_init(chain.nr_joints);

 	q = rand_joint_vector(chain.nr_joints, q_min, q_max);
 	dq = rand_joint_vector(chain.nr_joints, q_min, q_max);
 	ddq = rand_joint_vector(chain.nr_joints, q_min, q_max);
    q_init = rand_joint_vector(chain.nr_joints, q_min, q_max);
 	
 	auto tic = now();
 	kin.joint_to_cartesian(q);
 	auto toc = now();
 	auto duration = duration_cast<microseconds>( toc - tic ).count();

 	vector<double> timings;
 	timings.reserve(nr_runs);

 	// Time Forward Kinematics
 	cout << "==============================" << endl 
 		 << "Forward kinematics" 			 << endl
 		 << "==============================" << endl;
 	for(int i=0; i<nr_runs; ++i){
 		q = rand_joint_vector(chain.nr_joints, q_min, q_max);

 		tic = now();
	 	kin.joint_to_cartesian(q);
	 	toc = now();
	 	duration = duration_cast<microseconds>( toc - tic ).count();
	 	timings.push_back(duration);
 	}
 	print_timing_result(timings);
 	cout << endl;


 	// Time Inverse Kinematics
 	cout << "==============================" << endl 
 		 << "Inverse kinematics" 			 << endl
 		 << "==============================" << endl;
 	timings.clear();
 	timings.reserve(nr_runs);
 	vector<double> error_codes;
 	error_codes.reserve(nr_runs);
 	int error_code = 0;
 	for(int i=0; i<nr_runs; ++i){
 		q = rand_joint_vector(chain.nr_joints, q_min, q_max);
 		q_init = rand_joint_vector(chain.nr_joints, q_min, q_max);
 		kin.joint_to_cartesian(q);
 		Frame f_target = kin.f_end;
 		tic = now();
	 	error_code = kin.cartesian_to_joint(f_target, q_init);
	 	toc = now();
	 	duration = duration_cast<microseconds>( toc - tic ).count();
	 	timings.push_back(duration);
	 	error_codes.push_back(error_code);
 	}
 	int nr_no_error = count(error_codes.begin(), error_codes.end(), 1);
 	double successrate = 100.0 * double(nr_no_error)/nr_runs;
 	cout << "Successrate: " << successrate << "%" << endl;
 	print_timing_result(timings);
 	cout << endl;


 	// Time Inverse Dynamics
 	cout << "==============================" << endl 
 		 << "Inverse dynamics" 			 << endl
 		 << "==============================" << endl;
 	timings.clear();
 	timings.reserve(nr_runs);
 	for(int i=0; i<nr_runs; ++i){
 		q = rand_joint_vector(chain.nr_joints, q_min, q_max);
	 	dq = rand_joint_vector(chain.nr_joints, q_min, q_max);
	 	ddq = rand_joint_vector(chain.nr_joints, q_min, q_max);
 		tic = now();
	 	dyn.calculate_torques(q, dq, ddq);
	 	toc = now();
	 	duration = duration_cast<microseconds>( toc - tic ).count();
	 	timings.push_back(duration);
 	}
 	print_timing_result(timings);
 	cout << endl;
} 
