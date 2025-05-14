#!/bin/bash

# Requirements
if [ "$(uname)" == "Darwin" ]; then
    echo "Installing prerequisites with brew..."
    brew install eigen cppzmq boost
elif [ "$(expr substr "$(uname -s)" 1 5)" == "Linux" ]; then
    echo "Installing prerequisites with apt..."
    sudo apt install python3-dev libeigen3-dev libzmq3-dev libboost-all-dev
else
    echo "Unrecognized system."
fi

python3 -m pip config set global.break-system-packages true
python3 -m pip install setuptools wheel

python3 -m pip install dqrobotics --pre
python3 -m pip install ./python_wrapper

cat adaptive_control_import_eval.py
python3 adaptive_control_import_eval.py
