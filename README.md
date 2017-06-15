# Robo

[![Build Status](https://travis-ci.org/DaveTrussel/robo.svg?branch=master)](https://travis-ci.org/DaveTrussel/robo)

Built with the following compilers: g++5, g++6, clang++3.6, clang++3.9

This project is intended to provide a simple interface to model robotic manipulators. 

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
```

For now only a test application is build. Maybe build a shared or static lib instead later into the project.

## Running the tests

Basic functionality is currently tested in the `testing` executable.


```
./testing
```

### Basic Usage
Here is a basic example (TODO add includes)

```cpp
using namespace robo;

// Define axes of rotation
Eigen::Vector3d axis_z, axis_y;
axis_y << 0.0, 1.0, 0.0;
axis_z << 0.0, 0.0, 1.0;

// Creates a standard frame origin at 0 and standard x,y,z axes
Frame f = Frame();

// Create joint types (JointType::Translational would also be possible)
Joint joint_ellbow = Joint(0, f, axis_y, JointType::Rotational);
Joint joint_wrist = Joint(0, f, axis_z, JointType::Rotational);
Joint joint_none = Joint(0, f, axis_z, JointType::None);

// Simplified case with all same link lengths
Eigen::Vector3d length;
length << 0.0, 0.0, 1.0;
Frame tip = Frame(length);

// Create links
Link link_0 = Link(0, joint_none, tip);
Link link_1 = Link(1, joint_wrist, tip);
Link link_2 = Link(2, joint_ellbow, tip);
Link link_3 = Link(3, joint_ellbow, tip);
Link link_4 = Link(4, joint_wrist, tip);
Link link_5 = Link(5, joint_ellbow, tip);
Link link_6 = Link(6, joint_wrist, tip);

// Create a chain out of links
Chain chain;
chain.addLink(link_0);
chain.addLink(link_1);
chain.addLink(link_2);
chain.addLink(link_3);
chain.addLink(link_4);
chain.addLink(link_5);
chain.addLink(link_6);
	
ForwardKinematics fk = ForwardKinematics(chain);

Eigen::VectorXd q(chain.nr_links);
q << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
std::vector<Frame> f_out(chain.nr_links);

TimePoint tic = now();
fk.joint2cartesian(q, f_out); // Solve forward  kinematics
TimePoint toc = now();

auto duration = duration_cast<microseconds>( toc - tic ).count();

cout << "Solved forward kinematics in: " << duration << " Microseconds." << endl;
cout << "Frame at end of robot chain:" << endl << f_out.at(chain.nr_links-1).origin << endl << f_out.at(chain.nr_links-1).orientation << endl;
cout << endl << "And as homogeneous matrix:" << endl << f_out.at(chain.nr_links-1).as_homogeneous_matrix() << endl;
```

## TODO
- Basic model (Frames, Joints, Links, Chain)  :white_check_mark:
- Forward Kinematics (joint coordinates to cartesian coordinates)  :white_check_mark:
- Inverse Kinematics (cartesian coordinates to joint coordinates) (First implementation done. However check this paper [here](http://mi.ams.eng.osaka-u.ac.jp/pub/2011/tro2011sugihara.pdf) for improvement in performance and success rate. Also implement joint limits.)
- Dynamic Model (Positions, Velocities, Torques -> Accelerations)
- Inverse Dynamic Model (Positions, Velocities, Accelerations -> Torques)
- Implement own classes for twist (minimal velocity, acceleration representation) and Wrench (minimal force representation)
- Investigate possible ROS2 integration (also check if (soft/hard) real-time execution is possible through profiling i.e. low jitter. Avoid dynamic memory allocation with malloc/new after initialization to avoid page faults and obtain deterministic execution paths.

## Authors

* **David Trussel**  :monkey:

