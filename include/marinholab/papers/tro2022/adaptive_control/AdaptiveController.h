#pragma once
/**
 * (C) Copyright 2023-2026 Murilo Marques Marinho (www.murilomarinho.info)
 *
 * This file is part of adaptive_control_example.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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

#include"marinholab/papers/tro2022/adaptive_control/SerialManipulatorEDH.h"
#include"marinholab/papers/tro2022/adaptive_control/MeasurementSpace.h"
#include"marinholab/papers/tro2022/adaptive_control/VFI.h"


namespace marinholab::papers::tro2022::adaptive_control
{

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
    MeasureSpace measure_space;
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
//- add a class similar to DQ_SerialManipulator that provides parameter-space Jacobians, e.g. SerialManipulatorEDH.
//- add a class similar to DQ_KinematicController but for the parameter-space.
//- And many of the support classes in this example, such as Example_AdaptiveControlStrategy and MeasureSpace.
class AdaptiveController
{
private:
    const Example_SimulationParameters& simulation_arguments_;
    std::shared_ptr<SerialManipulatorEDH> robot_;

    DQ_QPOASESSolver task_space_solver_;
    DQ_QPOASESSolver parameter_space_solver_;

    DQ _convert_pose_to_measure_space(const DQ& x, const MeasureSpace& measure_space);

    static VectorXd _smart_vec(const DQ& x, const MeasureSpace& measure_space);
    static MatrixXd _convert_pose_jacobian_to_measure_space(const MatrixXd& Jx, const DQ &x, const DQ &xd, const MeasureSpace& measure_space);
    static MatrixXd _get_complimentary_measure_space_jacobian(const MatrixXd& Jx, const DQ &x, const MeasureSpace& measure_space);
 public:
    AdaptiveController()=delete;
    AdaptiveController(AdaptiveController&)=delete;
    AdaptiveController(const std::shared_ptr<SerialManipulatorEDH>& robot,
                               const Example_SimulationParameters &simulation_arguments);

    std::tuple<VectorXd, VectorXd, VectorXd, VectorXd, DQ> compute_setpoint_control_signal(const Example_AdaptiveControlStrategy &control_strategy,
                                                                                           const VectorXd& q,
                                                                                           const DQ& xd,
                                                                                           const DQ& y,
                                                                                           std::vector<VFI> &vfis);
};

}  // namespace marinholab::papers::tro2022::adaptive_control
