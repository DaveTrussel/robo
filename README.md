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

TODO for now only test application is build. Maybe build a shared or static lib instead.

## Running the tests

Basic functionality is currently tested in the `testing` executable.


```
./testing
```

### Basic Usage
TODO Explain how to use it

```
Give an example
```

## TODO
- Basic model (Frames, Joints, Links, Chain)
- Forward Kinematics (joint coordinates to cartesian coordinates)
- Inverse Kinematics (cartesian coordinates to joint coordinates)
- Dynamic Model (Positions, Velocities, Torques -> Accelerations)
- Inverse Dynamic Model (Positions, Velocities, Accelerations -> Torques)

## Authors

* **David Trussel**

