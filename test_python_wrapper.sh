#!/bin/bash
sudo apt install pybind11-dev python3-dev python3-disutils

cd ~
mkdir tmp
cd tmp

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

python3 -m venv venv
source venv/bin/activate

python3 -m pip install --upgrade pip

python3 -m pip uninstall dqrobotics
git clone https://github.com/dqrobotics/python.git dqrobotics_python --recursive
python3 -m pip install wheel scipy quadprog
python3 -m pip install ./dqrobotics_python 

cd dqrobotics_python/tests
python3 DQ_test.py
python3 DQ_Kinematics_test.py
python3 cpp_issues.py
python3 python_issues.py
cd ..
cd ..

git clone https://github.com/mmmarinho/tro2022_adaptivecontrol.git --recursive
cd tro2022_adaptivecontrol
python3 -m pip install ./python_wrapper
cd ..

deactivate
source venv/bin/activate

echo "
from dqrobotics import *
from adaptive_control_example import *
" > adaptive_control_import_eval.py

python3 adaptive_control_import_eval.py