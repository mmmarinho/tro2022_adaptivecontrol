#pragma once
/**
(C) Copyright 2020-2023 Murilo Marques Marinho (www.murilomarinho.info)

This file is part of adaptive_control_example.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    adaptive_control_example is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with adaptive_control_example.  If not, see <http://www.gnu.org/licenses/>.

Author:
    Murilo M. Marinho (murilomarinho@ieee.org)

Contributors (aside from author):
    None
*/
#include<tuple>

#include<dqrobotics/DQ.h>
#include<dqrobotics/solvers/DQ_QPOASESSolver.h>

#include"robot_modeling/Example_SerialManipulatorEDH.h"
#include"robot_control/Example_MeasurementSpace.h"
#include"constraints_modeling/Example_VFI.h"

using namespace Eigen;
using namespace DQ_robotics;

enum class Example_AdaptiveControlStrategy
{
    NONE=0,
    TASK_ONLY,
    MEASUREMENT_ONLY,
    FULL
};

struct Example_SimulationArguments
{
    Example_MeasureSpace measure_space;
    double proportional_gain;
    double vfi_gain;
    double vfi_weight;
    double damping;
    bool use_adaptation;
};


std::tuple<VectorXd, VectorXd, VectorXd, VectorXd, DQ> compute_setpoint_control_signal(const Example_AdaptiveControlStrategy &control_strategy,
                                                                                         const VectorXd&q,
                                                                                         const Example_SerialManipulatorEDH& robot,
                                                                                         const DQ& x_d,
                                                                                         const DQ& y, std::vector<Example_VFI> &vfis,
                                                                                         const Example_SimulationArguments &cla);

DQ convert_pose_to_measure_space(const DQ& x, const Example_MeasureSpace& measure_space);
VectorXd smart_vec(const DQ& x, const Example_MeasureSpace& measure_space);
MatrixXd convert_pose_jacobian_to_measure_space(const MatrixXd& Jx, const DQ &x, const DQ &xd, const Example_MeasureSpace& measure_space);
MatrixXd get_complimentary_measure_space_jacobian(const MatrixXd& Jx, const DQ &x, const Example_MeasureSpace& measure_space);
int get_measure_space_dimension(const Example_MeasureSpace& measure_space);
