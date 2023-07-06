#!/bin/bash

# apt packages
sudo apt install python3-dev python3-distutils libeigen3-dev

# qpOASES
git clone https://github.com/coin-or/qpOASES.git
cd qpOASES
sed -i -e 's/option(BUILD_SHARED_LIBS "If ON, build shared library instead of static" OFF)/option(BUILD_SHARED_LIBS "If ON, build shared library instead of static" ON)/g' CMakeLists.txt
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
cd ..
cd ..

# dqrobotics devel
sudo add-apt-repository ppa:dqrobotics-dev/development
sudo apt-get update
sudo apt-get install libdqrobotics*

# python stuff, at user level
python3 -m pip install --upgrade pip
python3 -m pip install wheel scipy quadprog

# dqrobotics python, compiled in the user's enviroment
git clone https://github.com/dqrobotics/python.git dqrobotics_python --recursive
python3 -m pip install ./dqrobotics_python 

# test dqrobotics python
cd dqrobotics_python/tests
python3 DQ_test.py
python3 DQ_Kinematics_test.py
python3 cpp_issues.py
python3 python_issues.py
cd ..
cd ..

# Install the example wrapper code
python3 -m pip install ./python_wrapper

# Test the example wrapper code
echo "
from dqrobotics import *
from dqrobotics.robot_modeling import DQ_SerialManipulator
from adaptive_control_example import *
print('adaptive_control_example import ok.')
" > adaptive_control_import_eval.py

cat adaptive_control_import_eval.py

python3 adaptive_control_import_eval.py
