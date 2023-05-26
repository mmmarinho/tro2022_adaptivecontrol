#!/bin/bash
####################################################
## Stand-alone example build script
## Murilo M. Marinho (www.murilomarinho.info)
####################################################

##########################
# LIST UP SUBMODULES HERE
##########################

# The packages will be built in this order, so
# use an order that makes sense with the dependencies
# of you example.
pkg_array=(
"submodules/dqrobotics/cpp"
"submodules/dqrobotics/cpp-interface-vrep"
"submodules/dqrobotics/cpp-interface-qpoases"
"submodules/qpOASES"
"." # This is the example itself, so don't forget it!
)

##########################
# BUILD ENVIROMENT SETUP
##########################

# Get the current directory
repo_root_dir=`pwd`
# For this shell, we add the local include folder
export CXXFLAGS=-isystem\ ${repo_root_dir}/include\ -L${repo_root_dir}/lib

####################################
# TRY TO GET THE NUMBER OF CORES
####################################

# This script works only on UBUNTU AFAIK
# and is a bit different from what you find on stackoverflow
# but also works for VMs
phycores=$(echo $sudoPW|lscpu | grep -m 1 "CPU(s)"|awk '{print $ 2;}') || true
echo "Found "${phycores}" physical cores."

##########################
# BUILD FUNCTION
##########################

BUILD(){
  rm -rf build
  mkdir build
  cd build
  cmake .. -DCMAKE_INSTALL_PREFIX=${repo_root_dir} -DCMAKE_BUILD_TYPE=Release
  if [ -z $variable ]
    then
      make -j${phycores}
    else
      make
  fi
  make install
}

######################################
# BUILD DEPENDENCIES & INSTALL LOCALLY
######################################

for pkg_relative_path in ${pkg_array[@]}; do
  echo "Building "${pkg_relative_path}"..."
  cd ${repo_root_dir}
  cd ${pkg_relative_path}
  BUILD
done

