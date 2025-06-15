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
/**
 * Example code for:
 * M. M. Marinho and B. V. Adorno,
 * "Adaptive Constrained Kinematic Control Using Partial or Complete Task-Space Measurements,"
 * in IEEE Transactions on Robotics, vol. 38, no. 6, pp. 3498-3513, Dec. 2022,
 * doi: 10.1109/TRO.2022.3181047.
 */
#include<tuple>
#include <dqrobotics/robot_control/DQ_KinematicController.h>
#include<dqrobotics/DQ.h>
#include<dqrobotics/solvers/DQ_QPOASESSolver.h>

#include"example/Example_SerialManipulatorEDH.h"
#include"example/Example_MeasurementSpace.h"
#include"example/Example_VFI.h"

using namespace Eigen;
using namespace DQ_robotics;

enum class Example_AdaptiveControlStrategy
{
    NONE=0,
    TASK_ONLY,
    MEASUREMENT_ONLY,
    FULL
};

struct Example_SimulationParameters
{
    Example_MeasureSpace measure_space;
    double proportional_gain;
    double vfi_gain;
    double vfi_weight;
    double damping;
    double sampling_time_sec;
    double reference_timeout_sec;
};

//To the pure soul that will port this to DQ_robotics.
//DQ_robotics needs to be altered before inheritance can happen here.
//For instance
//- add a class similar to DQ_SerialManipulator that provides parameter-space Jacobians, e.g. Example_SerialManipulatorEDH.
//- add a class similar to DQ_KinematicController but for the parameter-space.
//- And many of the support classes in this example, such as Example_AdaptiveControlStrategy and Example_MeasureSpace.
class Example_AdaptiveController
{
private:
    const Example_SimulationParameters& simulation_arguments_;
    std::shared_ptr<Example_SerialManipulatorEDH> robot_;

    ControlObjective control_objective;
    DQ   primitive;       //  the control objective (in format of class ControObjective) with respect to the end-effector

    DQ_QPOASESSolver task_space_solver_;
    DQ_QPOASESSolver parameter_space_solver_;

    DQ _convert_pose_to_measure_space(const DQ& x, const Example_MeasureSpace& measure_space);

    VectorXd _smart_vec(const DQ& x, const Example_MeasureSpace& measure_space);
    MatrixXd _convert_pose_jacobian_to_measure_space(const MatrixXd& Jx, const DQ &x,  const DQ &xd, const Example_MeasureSpace& measure_space);
    MatrixXd _convert_pose_jacobian_to_control_objective(const MatrixXd& Jx, const DQ &x,  const DQ &xd, const ControlObjective& objective_space);
    MatrixXd _get_complimentary_measure_space_jacobian(const MatrixXd& Jx, const DQ &x, const Example_MeasureSpace& measure_space);
 public:
    void set_primitive_to_effector(const DQ& primitive_);
    void set_control_objective(const ControlObjective& control_objective_);

    Example_AdaptiveController()=delete;
    Example_AdaptiveController(Example_AdaptiveController&)=delete;
    Example_AdaptiveController(const std::shared_ptr<Example_SerialManipulatorEDH>& robot,
                               const Example_SimulationParameters &simulation_arguments);

    std::tuple<VectorXd, VectorXd, VectorXd, VectorXd, DQ> compute_setpoint_control_signal(const Example_AdaptiveControlStrategy &control_strategy,
                                                                                           const VectorXd& q,
                                                                                           const DQ& xd,
                                                                                           const DQ& y,
                                                                                           std::vector<Example_VFI> &vfis);
};



