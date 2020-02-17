cmake_minimum_required(VERSION 3.5)
project(robo LANGUAGES CXX)

IF ( NOT CMAKE_BUILD_TYPE )
  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build. Options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo." FORCE)
  MESSAGE( "Setting build type to '${CMAKE_BUILD_TYPE}'" )
ELSE ( NOT CMAKE_BUILD_TYPE )
  MESSAGE( "Build type is set to '${CMAKE_BUILD_TYPE}' ." )
ENDIF ( NOT CMAKE_BUILD_TYPE )

option(NLOPT_CXX "enable cxx routines" ON)
option(NLOPT_FORTRAN "enable fortran tests" OFF)
option(BUILD_SHARED_LIBS "Build NLopt as a shared library" OFF)
option(NLOPT_PYTHON "build python bindings" OFF)
option(NLOPT_OCTAVE "build octave bindings" OFF)
option(NLOPT_MATLAB "build matlab bindings" OFF)
option(NLOPT_GUILE "build guile bindings" OFF)
option(NLOPT_SWIG "use SWIG to build bindings" OFF)
add_subdirectory(third_party/nlopt)

add_library(${PROJECT_NAME}
  src/robo/dynamics.cpp
  src/robo/kinematics_forward.cpp
  src/robo/kinematics_inverse.cpp
)

#src/robo/model_factory.cpp

target_include_directories(${PROJECT_NAME}
PUBLIC
  include/
PRIVATE
  src/
  third_party/eigen/
  ${nlopt_BINARY_DIR}/src/api
)

target_link_libraries(${PROJECT_NAME}
PRIVATE
  nlopt
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

add_executable(test_unit
  src/robo/test/unit.cpp
)

target_link_libraries(test_unit
  ${PROJECT_NAME}
)

target_include_directories(test_unit
PRIVATE
  src/
  third_party/
  third_party/eigen/
)

add_executable(test_time
  src/robo/test/time.cpp
)

target_include_directories(test_time
PRIVATE
  src/
  third_party/eigen/
)

target_link_libraries(test_time
  ${PROJECT_NAME}
)

add_executable(test_inverse_kinematics
  src/robo/test/inverse_kinematics.cpp
)

target_include_directories(test_inverse_kinematics
PRIVATE
  src/
  third_party/eigen/
)

target_link_libraries(test_inverse_kinematics
  ${PROJECT_NAME}
)
