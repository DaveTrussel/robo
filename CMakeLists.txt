cmake_minimum_required(VERSION 3.2.0)
project(robo)
 
#Bring the headers, such as Student.h into the project
include_directories(include lib/Eigen)
 
#However, the file(GLOB...) allows for wildcard additions:
file(GLOB SOURCES "src/*.cpp")
 
add_executable(testing ${SOURCES})

set_property(TARGET testing PROPERTY CXX_STANDARD 11)
set_property(TARGET testing PROPERTY CXX_STANDARD_REQUIRED ON)
