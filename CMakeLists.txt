cmake_minimum_required(VERSION 2.8.0)
project(robo)

include_directories(include lib/ /usr/include/eigen3)

file(GLOB SOURCES "src/*.cpp")

add_executable(testing ${SOURCES})

set_property(TARGET testing PROPERTY CXX_STANDARD 11)
set_property(TARGET testing PROPERTY CXX_STANDARD_REQUIRED ON)
# the line below is added because the above did not work on travis CI pipeline (but did work on Ubuntu 16.04 machine)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") # to do find proper CI solution
