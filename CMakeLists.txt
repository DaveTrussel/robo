cmake_minimum_required(VERSION 2.8.0) # TODO increase to 3.1 if available on travis
project(robo)

IF ( NOT CMAKE_BUILD_TYPE )
  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
  MESSAGE( "Setting build type to '${CMAKE_BUILD_TYPE}'" )
ELSE ( NOT CMAKE_BUILD_TYPE )
  MESSAGE( "Build type set to '${CMAKE_BUILD_TYPE}' by user." )
ENDIF ( NOT CMAKE_BUILD_TYPE )

include_directories(include/ lib/ /usr/include/eigen3)

file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "include/*.hpp")

add_executable(testing ${SOURCES} ${HEADERS})

set_property(TARGET testing PROPERTY CXX_STANDARD 11)
set_property(TARGET testing PROPERTY CXX_STANDARD_REQUIRED ON)
# cmake>=3.1 not available on travis set std11 manually
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
