cmake_minimum_required(VERSION 2.8.0) # TODO increase to 3.1 if available on travis
project(robo)


IF ( NOT CMAKE_BUILD_TYPE )
  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build. Options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo." FORCE)
  MESSAGE( "Setting build type to '${CMAKE_BUILD_TYPE}'" )
ELSE ( NOT CMAKE_BUILD_TYPE )
  MESSAGE( "Build type is set to '${CMAKE_BUILD_TYPE}' ." )
ENDIF ( NOT CMAKE_BUILD_TYPE )

include_directories(include/ lib/ /usr/include/eigen3)

file(GLOB HEADERS "include/*.hpp")
file(GLOB HEADERS_TEST "lib/catch.hpp")
set(SRC_LIB src/kinematics.cpp src/dynamics.cpp)
set(SRC_TEST_UNIT ${SRC_LIB} src/test_unit.cpp)
set(SRC_TEST_TIME ${SRC_LIB} src/test_time.cpp)
set(SRC_TEST_KINEMATICS_SIMPLE ${SRC_LIB} src/test_kinematics_simple.cpp)


add_executable(test_unit ${SRC_TEST_UNIT} ${HEADERS} ${HEADERS_TEST})
add_executable(test_time ${SRC_TEST_TIME} ${HEADERS} ${HEADERS_TEST})
add_executable(test_kinematics_simple ${SRC_TEST_KINEMATICS_SIMPLE} ${HEADERS} ${HEADERS_TEST})


set_property(TARGET test_unit test_time test_kinematics_simple PROPERTY CXX_STANDARD 14)
set_property(TARGET test_unit test_time test_kinematics_simple PROPERTY CXX_STANDARD_REQUIRED ON)
# cmake>=3.1 not available on travis set std14 manually
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wextra" CACHE STRING "Compiler flags used by the Debug CMAKE_BUILD_TYPE." FORCE)
#set(CMAKE_CXX_FLAGS_RELEASE "-Ofast -DNDEBUG -Wall -Wextra -march=native -fopenmp" CACHE STRING "Compiler flags used by the Release CMAKE_BUILD_TYPE." FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -Wall -Wextra -march=native" CACHE STRING "Compiler flags used by the Release CMAKE_BUILD_TYPE." FORCE)
#set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-Ofast -DNDEBUG -g -Wall -Wextra -march=native -fopenmp" CACHE STRING "Compiler flags used by the RelWithDebInfo CMAKE_BUILD_TYPE." FORCE)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -DNDEBUG -g -Wall -Wextra -march=native" CACHE STRING "Compiler flags used by the RelWithDebInfo CMAKE_BUILD_TYPE." FORCE)
