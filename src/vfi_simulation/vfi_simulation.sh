#!/usr/bin/bash
rosparam load params
rosrun simulations_cpp setpoint_simulation -measure_space 1 -use_adaptation false -proportional_gain 4. -vfi_gain 10.0 -vfi_weight 0.001 -damping 0.01 -parameter_source parameterserver
rosrun simulations_cpp setpoint_simulation -measure_space 1 -use_adaptation true -proportional_gain 4. -vfi_gain 10.0 -vfi_weight 0.001 -damping 0.01 -parameter_source parameterserver
