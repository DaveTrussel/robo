cmake_minimum_required(VERSION 2.8.0) # TODO increase to 3.1 if available on travis
project(robo)

include_directories(include/ lib/ /usr/include/eigen3)

file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "include/*.hpp")

add_executable(testing ${SOURCES} ${HEADERS})

set_property(TARGET testing PROPERTY CXX_STANDARD 11)
set_property(TARGET testing PROPERTY CXX_STANDARD_REQUIRED ON)
# cmake>=3.1 not available on travis set std11 manually
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
