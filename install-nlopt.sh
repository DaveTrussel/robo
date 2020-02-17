#! /bin/bash
#
# Build and install the NLopt optimization library
# Is statically linked so not needed on the target system
#

function privileged_execute {
  if [[ $(whoami) != "root" ]]
  then
    sudo $@
  else
    $@
  fi 
}

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"                                                                 
BUILD_DIR="$SCRIPTPATH/src/third_party/nlopt/build"                                                              

rm -rf $BUILD_DIR                                                              
mkdir $BUILD_DIR                                                               
cd $BUILD_DIR                                                                  
cmake -D NLOPT_CXX=ON -D BUILD_SHARED_LIBS=OFF -D NLOPT_PYTHON=OFF \
    -D NLOPT_OCTAVE=OFF -D NLOPT_GUILE=OFF -D NLOPT_SWIG=OFF \
    -D NLOPT_LINK_PYTHON=OFF -D CMAKE_C_COMPILER=clang \
    -D CMAKE_CXX_COMPILER=clang++ -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_C_FLAGS=-Wno-deprecated-declarations ..
                                                                                
make -j$(nproc)

privileged_execute make install
 
# clean up
cd $SCRIPTPATH
privileged_execute rm -r $BUILD_DIR
