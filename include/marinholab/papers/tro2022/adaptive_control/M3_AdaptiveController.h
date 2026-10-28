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
    Frederico Fernandes Afonso Silva (frederico.silva@manchester.ac.uk)
        - Add line control objective and associated methods
*/
/**
 * Example code for:
 * M. M. Marinho and B. V. Adorno,
 * "Adaptive Constrained Kinematic Control Using Partial or Complete Task-Space Measurements,"
 * in IEEE Transactions on Robotics, vol. 38, no. 6, pp. 3498-3513, Dec. 2022,
 * doi: 10.1109/TRO.2022.3181047.
 */
#include<tuple>

#include<dqrobotics/DQ.h>
#include<dqrobotics/solvers/DQ_QPOASESSolver.h>
#include<dqrobotics/robot_control/DQ_KinematicController.h>

#include"marinholab/papers/tro2022/adaptive_control/M3_SerialManipulatorEDH.h"
#include"marinholab/papers/tro2022/adaptive_control/M3_MeasurementSpace.h"
#include"marinholab/papers/tro2022/adaptive_control/M3_VFI.h"

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
    M3_MeasureSpace measure_space;
    double proportional_gain;
    double proportional_adaptation_gain;
    double vfi_gain;
    double vfi_weight;
    double damping;
    double sampling_time_sec;
    double reference_timeout_sec;
};

//To the pure soul that will port this to DQ_robotics.
//DQ_robotics needs to be altered before inheritance can happen here.
//For instance
//- add a class similar to DQ_SerialManipulator that provides parameter-space Jacobians, e.g. M3_SerialManipulatorEDH.
//- add a class similar to DQ_KinematicController but for the parameter-space.
//- And many of the support classes in this example, such as Example_AdaptiveControlStrategy and M3_MeasureSpace.
class M3_AdaptiveController
{
private:
    const Example_SimulationParameters& simulation_arguments_;
    std::shared_ptr<M3_SerialManipulatorEDH> robot_;

    ControlObjective control_objective_ = ControlObjective::Pose;
    DQ attached_primitive_;
    DQ target_primitive_;

    DQ_QPOASESSolver task_space_solver_;
    DQ_QPOASESSolver parameter_space_solver_;

    std::tuple<VectorXd,double> _closest_invariant_task_error(const DQ& x, const DQ& xd, const M3_MeasureSpace& measure_space);
    MatrixXd _get_task_jacobian(const VectorXd& q, const DQ& xd) const;
    MatrixXd _get_parameter_jacobian(const VectorXd& q, const VectorXd& x_tilde, const DQ& xd) const;

    DQ _convert_pose_to_measure_space(const DQ& x, const M3_MeasureSpace& measure_space);

    static VectorXd _smart_vec(const DQ& x, const M3_MeasureSpace& measure_space);
    static MatrixXd _convert_pose_jacobian_to_measure_space(const MatrixXd& Jx, const DQ &x, const DQ &xd, const M3_MeasureSpace& measure_space);
    static MatrixXd _get_complimentary_measure_space_jacobian(const MatrixXd& Jx, const DQ &x, const M3_MeasureSpace& measure_space);
 public:
    M3_AdaptiveController()=delete;
    M3_AdaptiveController(M3_AdaptiveController&)=delete;
    M3_AdaptiveController(const std::shared_ptr<M3_SerialManipulatorEDH>& robot,
                               const Example_SimulationParameters &simulation_arguments);

    ControlObjective get_control_objective() const;

    void set_control_objective(const ControlObjective& control_objective);

    void set_primitive_to_effector(const DQ& primitive);

    void set_target_primitive(const DQ& primitive);

    std::tuple<VectorXd, VectorXd, VectorXd, VectorXd, DQ> compute_setpoint_control_signal(const Example_AdaptiveControlStrategy &control_strategy,
                                                                                           const VectorXd& q,
                                                                                           const DQ& xd,
                                                                                           const DQ& y,
                                                                                           std::vector<M3_VFI> &vfis);
};



