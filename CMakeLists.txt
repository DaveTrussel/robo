cmake_minimum_required(VERSION 2.8.0) # TODO increase to 3.1 if available on travis
project(robo)

IF ( NOT CMAKE_BUILD_TYPE )
  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build. Options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
  MESSAGE( "Setting build type to '${CMAKE_BUILD_TYPE}'" )
ELSE ( NOT CMAKE_BUILD_TYPE )
  MESSAGE( "Build type set to '${CMAKE_BUILD_TYPE}' by user." )
ENDIF ( NOT CMAKE_BUILD_TYPE )

include_directories(include/ lib/ /usr/include/eigen3)

file(GLOB HEADERS "include/*.hpp")
file(GLOB HEADERS_TEST "lib/catch.hpp")
set(SRC_LIB src/kinematics.cpp src/dynamics.cpp)
set(SRC_UNIT_TEST ${SRC_LIB} src/test_unit.cpp)
set(SRC_TIME_TEST ${SRC_LIB} src/test_time.cpp)

add_executable(test_unit ${SRC_UNIT_TEST} ${HEADERS} ${HEADERS_TEST})
add_executable(test_time ${SRC_TIME_TEST} ${HEADERS} ${HEADERS_TEST})

set_property(TARGET test_unit test_time PROPERTY CXX_STANDARD 14)
set_property(TARGET test_unit test_time PROPERTY CXX_STANDARD_REQUIRED ON)
# cmake>=3.1 not available on travis set std14 manually
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
