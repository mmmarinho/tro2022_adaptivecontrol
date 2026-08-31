# (C) Copyright 2023-2026 Murilo Marques Marinho (www.murilomarinho.info)
#
# This file is part of adaptive_control_example.
#
# SPDX-License-Identifier: MIT
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

#!/bin/bash

####################################################
## Stand-alone example build script
## Murilo M. Marinho (www.murilomarinho.info)
####################################################
#
# This script, in brief.
#
# It expects the cmake's for each (sub)project to behave
# in a sane way. E.g.
#
# - Compiled examples go into bin/
# - Compiled libraries/shared libraries go into lib/
# - Installed headers go into include/
# - Dependencies are cmake projects in the submodules folder.
#   It does not matter if the submodule is a git submodule.
#
# Subprojects do not have to be modified to install them
# locally, this is done by this script.
#
##########################
# LIST UP SUBMODULES HERE
##########################

# The top-level CMakeLists.txt builds every dependency (dqrobotics,
# qpOASES) in-tree through add_subdirectory, so only the example itself
# needs to be configured/built here. The standalone build skips the
# pybind11 module (see -DBUILD_PYTHON_WRAPPER=OFF in BUILD below).
pkg_array=(
"." # This is the example itself, so don't forget it!
)

##########################
# BUILD ENVIRONMENT SETUP
##########################

# Get the current directory
repo_root_dir=`pwd`
# For this shell, we add the local include folder
export CXXFLAGS=-isystem\ ${repo_root_dir}/include\ -L${repo_root_dir}/lib

####################################
# TRY TO GET THE NUMBER OF CORES
####################################

# This script works only on Ubuntu, AFAIK, so we use `|| true` to ignore failures.
# It is a bit different from what you find on stackoverflow but also works for Ubuntu VMs
phycores=$(echo $sudoPW|lscpu | grep -m 1 "CPU(s)"|awk '{print $ 2;}') || true
echo "Found ""${phycores}"" physical cores."

##########################
# BUILD FUNCTION
##########################

# The only unusual thing here is the -DCMAKE_INSTALL_PREFIX that we set
# so everything is installed locally.
BUILD(){
  rm -rf build
  mkdir build
  cd build || return
  # The standalone C++ build does not need a Python interpreter, so the
  # pybind11 module (built by `pip install .` through setup.py) is skipped.
  cmake .. -DCMAKE_INSTALL_PREFIX="${repo_root_dir}" -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DBUILD_PYTHON_WRAPPER=OFF
  if [ -z "$variable" ]
    then
      make -j"${phycores}"
    else
      make
  fi
  make install
}

######################################
# BUILD DEPENDENCIES & INSTALL LOCALLY
######################################

for pkg_relative_path in "${pkg_array[@]}"; do
  echo "Building ""${pkg_relative_path}""..."
  cd "${repo_root_dir}" || return
  cd "${pkg_relative_path}" || return
  BUILD # The function defined above in this script
done

#############################################################
# FOR MACOS WE HAVE TO DEFINE EXPLICITLY WHERE THE DYLIBS ARE
#############################################################

# Figuring this one out was a little difficult and convoluted, so sadly I don't have
# a precise link to share. In brief, the compiled binaries expect the
# dylibs to be at a system-wide folder, e.g. /usr/local. We use the `install_name_tool`
# to re-link the libraries to use the local copies in lib/.

if [ "$(uname)" == "Darwin" ]; then
  cd "${repo_root_dir}" || return
  cd bin || return
  for executable_name in *; do
    [ -e "$executable_name" ] || continue
    for dylib_name in ../lib/*.dylib; do
      [ -e "$dylib_name" ] || continue
      dylib_base_name="$(basename -- $dylib_name)"
      echo "Fixing ""${dylib_base_name}"" for ""${executable_name}"" ..."
      install_name_tool -change @rpath/"${dylib_base_name}" @executable_path/../lib/"${dylib_base_name}" "${executable_name}"
    done
  done
fi
