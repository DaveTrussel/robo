# Robo

[![Build Status](https://travis-ci.org/DaveTrussel/robo.svg?branch=master)](https://travis-ci.org/DaveTrussel/robo)

This C++ project is intended to provide a simple way of modelling robotic chain manipulators and be used for their control. It lets you calculte the forward and inverse kinematics and dynamics of the robot.

Built with the following compilers: g++5, g++6, clang++3.6, clang++3.9

License: GPLv3.


## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Dependencies

Depends on the Eigen3 header-only library (for now still in the repo)

On Ubuntu install the header files with:
```
sudo apt-get install libeigen3-dev
```

### Installing

This project uses the Cmake build system

```
mkdir build && cd build
cmake ..
make -j4
```

For now only the test applications are built. Depending on the later design choices providing a shared library or a ROS2 package will be considered.

## Running the tests

The unit tests are currently all in the test_unit executable.
The tests can be run from the build directory like this:

```
./test_unit
sudo chrt 98 ./test_time

// Examplary output from the timing tests (on a non-realtime OS)
==============================
Forward kinematics
==============================
Timing results (in microseconds) from 100000 runs:
Average: 0.9426
Median: 1
Min: 0
Max: 4101

==============================
Inverse kinematics
==============================
Successrate: 99.994%
Timing results (in microseconds) from 100000 runs:
Average: 73.4609
Median: 64
Min: 15
Max: 4593

==============================
Inverse dynamics
==============================
Timing results (in microseconds) from 100000 runs:
Average: 3.07337
Median: 3
Min: 2
Max: 189
```

### Basic Usage
Here is a basic example:

```cpp
#include "kinematics.hpp"
#include "dynamics.hpp"
using namespace robo;

// Define axes of rotation
Vector3d axis_z, axis_y;
axis_y << 0.0, 1.0, 0.0;
axis_z << 0.0, 0.0, 1.0;

// Creates a standard frame origin at 0 and standard x,y,z axes
Frame f = Frame();

// Create joint types (Joint_type::Translational would also be possible)
Joint joint_ellbow = Joint(0, f, axis_y, Joint_type::Rotational);
Joint joint_wrist = Joint(0, f, axis_z, Joint_type::Rotational);
Joint joint_none = Joint(0, f, axis_z, Joint_type::None);

// Simplified case with all same link lengths
Vector3d length;
length << 0.0, 0.0, 1.0;
Frame tip = Frame(length);

Inertia inertia = Inertia(1.0, length/2);

// Create links
Link link_0 = Link(0, joint_none, tip, inertia);
Link link_1 = Link(1, joint_wrist, tip, inertia);
Link link_2 = Link(2, joint_ellbow, tip, inertia);
Link link_3 = Link(3, joint_ellbow, tip, inertia);
Link link_4 = Link(4, joint_wrist, tip, inertia);
Link link_5 = Link(5, joint_ellbow, tip, inertia);
Link link_6 = Link(6, joint_wrist, tip, inertia);

// Create a chain out of links
Chain chain;
chain.add_link(link_0);
chain.add_link(link_1);
chain.add_link(link_2);
chain.add_link(link_3);
chain.add_link(link_4);
chain.add_link(link_5);
chain.add_link(link_6);
	
Kinematics kin = Kinematics(chain);
Dynamics dyn = Dynamics(chain);

VectorXd q(chain.nr_joints); // joint positions
VectorXd dq(chain.nr_joints); // joint velocities
VectorXd ddq(chain.nr_joints); // joint acclerations
VectorXd q_init(chain.nr_joints); // initial position for inverse kinematics
q 	<< 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
dq 	<< 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
ddq 	<< 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
q_init 	<< 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2;

// Solve forward kinematics
kin.joint_to_cartesian(q);
Frame result_fk = kin.f_end;

// Solve inverse kinematics
Frame f_target = result_fk;
int error_code = kin.cartesian_to_joint(f_target, q_init);
VectorXd result_ik = kin.q_out;

// Solve inverse dynamics
dyn.calculate_torques(q, dq, ddq);
VectorXd result_id = dyn.joint_torques;
```

## Implementation details
Internally rotations are represented as matrices.
### Forward kinematics
Recursive
### Inverse kinematics
Damped Weighted Levenberg-Marquart algorithm
### Inverse dynamics
Recursive Newton-Euler algorithm

## TODO
- Basic model (Frames, Joints, Links, Chain)  :white_check_mark:
- Forward Kinematics (joint coordinates to cartesian coordinates)  :white_check_mark:
- Inverse Kinematics (cartesian coordinates to joint coordinates) (Based on a damped Levenberg-Marquart algorithm. Inspired by this paper [here](http://mi.ams.eng.osaka-u.ac.jp/pub/2011/tro2011sugihara.pdf) which showed high success rate for the recommended LM method.) and [here](https://groups.csail.mit.edu/drl/journal_club/papers/033005/buss-2004.pdf) :white_check_mark:
- Inverse Dynamic Model (Positions, Velocities, Accelerations -> Torques) :white_check_mark:
- Own classes for twist (minimal velocity, acceleration representation) :white_check_mark: and Wrench (minimal force representation) :white_check_mark:
- Own classes for inertia :white_check_mark:
- Investigate possible ROS2 integration (also check if (soft/hard) real-time execution is possible through profiling i.e. low jitter. 
- Avoid dynamic memory allocation with malloc/new after initialization to avoid page faults and obtain deterministic execution paths. :white_check_mark:
- Add license :white_check_mark:
- Dynamic Model (Positions, Velocities, Torques -> Accelerations)
- Implementation of joint limits (is this compatible with IK?)
- Define a control concept based on kinematcs / dynamics
- Add path planing
- Add friction torques to joints and dynamics
- Add rotor inertia to joints and dynamics
- Avoid self-collision in IK (introduce sphere around each joint? cylinder around each link? do research on this) 

## Authors

* **David Trussel**  :monkey:

