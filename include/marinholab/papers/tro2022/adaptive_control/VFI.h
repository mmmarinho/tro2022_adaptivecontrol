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

#include<memory>
#include<string>

#include<dqrobotics/DQ.h>
#include "marinholab/papers/tro2022/adaptive_control/SimulatorDummy.h"


namespace marinholab::papers::tro2022::adaptive_control
{

using namespace DQ_robotics;

enum class Primitive
{
    None=0,
    Point,
    Plane,
    Line
};

enum class VFI_Direction
{
    None=0,
    FORBIDDEN_ZONE,//-Jd*q \leq \eta\tilde{d}, \tilde{d}=d-d_safe
    SAFE_ZONE//Jd*q \leq \eta\tilde{d}, \tilde{d}=d_safe-d
};

enum class VFI_DistanceType
{
    None=0,
    EUCLIDEAN,
    EUCLIDEAN_SQUARED
};

/**
 * @brief The Example_VFI class is an abstraction of VFIs to make them more configurable and easier to use.
 *
 * Implements some of the VFIs initially discussed in [1], below.
 *
 * [1] M. M. Marinho, B. V. Adorno, K. Harada and M. Mitsuishi,
 * "Dynamic Active Constraints for Surgical Robots Using Vector-Field Inequalities,"
 * in IEEE Transactions on Robotics, vol. 35, no. 5, pp. 1166-1185, Oct. 2019, doi: 10.1109/TRO.2019.2920078.
 *
 * This implementation was made in the context of [2], below.
 *
 * [2] M. M. Marinho and B. V. Adorno,
 * "Adaptive Constrained Kinematic Control Using Partial or Complete Task-Space Measurements,"
 * in IEEE Transactions on Robotics, vol. 38, no. 6, pp. 3498-3513, Dec. 2022, doi: 10.1109/TRO.2022.3181047.
 *
 * and does not include all VFIs of [1].
 *
 * A more mature implementation is available at [3], below.
 *
 * [3] Design and Validation of a Multi-Arm Robotic Platform for Scientific Exploration
 * Murilo Marques Marinho, Juan José Quiroz-Omaña, Kanako Harada
 * https://arxiv.org/abs/2210.11877
 */
class VFI
{
    std::string workspace_entity_name_;
    std::string robot_entity_name_;
    Primitive type_;
    DQ value_;
    std::shared_ptr<SimulatorDummy> vi_;
    double safe_distance_;
    VFI_Direction vfi_direction_;
    const int joint_index_; //Needs to be correctly implemented in the future
    const DQ relative_displacement_to_joint_;
    const std::string cs_reference_name_;

    //New in this paper
    double last_estimated_distance_;
    double last_real_distance_;
public:
    VFI(const std::string& workspace_entity_name,
                    const std::string& robot_entity_name,
                    const Primitive& type,
                    const std::shared_ptr<SimulatorDummy>& vi,
                    const double& safe_distance,
                    const VFI_Direction& vfi_direction,
                    const int& joint_index,
                    const DQ& relative_displacement_to_joint,
                    const std::string& cs_reference_name="");

    void initialize();

    DQ get_value() const;

    void set_value(const DQ &value);

    MatrixXd get_distance_jacobian(const DQ& x, const MatrixXd& Jx) const;

    MatrixXd get_vfi_matrix(const DQ& x, const MatrixXd& Jx) const;

    double get_distance(const DQ& x) const;

    double get_distance_error(const DQ& x) const;

    double get_safe_distance() const;

    VFI_DistanceType get_distance_type() const;

    void set_last_real_distance(const DQ& y);

    double get_last_real_distance() const;

    void set_last_estimated_distance(const DQ& x_hat);

    double get_last_estimated_distance() const;

    std::string get_vfi_name() const;
};

}  // namespace marinholab::papers::tro2022::adaptive_control
