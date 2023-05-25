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

#include<memory>
#include<string>

#include<dqrobotics/DQ.h>
#include<dqrobotics/robot_modeling/DQ_Kinematics.h>
#include<dqrobotics/utils/DQ_Geometry.h>
#include<dqrobotics/interfaces/vrep/DQ_VrepInterface.h>


enum class Example_Primitive
{
    None=0,
    Point,
    Plane,
    Line
};

enum class Example_VFI_Direction
{
    None=0,
    FORBIDDEN_ZONE,//-Jd*q \leq \eta\tilde{d}, \tilde{d}=d-d_safe
    SAFE_ZONE//Jd*q \leq \eta\tilde{d}, \tilde{d}=d_safe-d
};

enum class Example_VFI_DistanceType
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
class Example_VFI
{
private:
    std::string workspace_entity_name_;
    std::string robot_entity_name_;
    Example_Primitive type_;
    DQ value_;
    std::shared_ptr<DQ_VrepInterface> vi_;
    double safe_distance_;
    Example_VFI_Direction vfi_direction_;
    const int joint_index_; //Needs to be correctly implemented in the future
    const DQ relative_displacement_to_joint_;
    const std::string cs_reference_name_;

    //New in this paper
    double last_estimated_distance_;
    double last_real_distance_;
public:
    Example_VFI(const std::string& workspace_entity_name,
                    const std::string& robot_entity_name,
                    const Example_Primitive& type,
                    const std::shared_ptr<DQ_VrepInterface>& vi,
                    const double& safe_distance,
                    const Example_VFI_Direction& vfi_direction,
                    const int& joint_index,
                    const DQ& relative_displacement_to_joint,
                    const std::string& cs_reference_name=VREP_OBJECTNAME_ABSOLUTE);

    void initialize();

    DQ get_value() const;

    void set_value(const DQ &value);;

    MatrixXd get_distance_jacobian(const DQ& x, const MatrixXd& Jx) const;

    MatrixXd get_vfi_matrix(const DQ& x, const MatrixXd& Jx) const;

    double get_distance(const DQ& x) const;

    double get_distance_error(const DQ& x) const;

    double get_safe_distance() const;

    Example_VFI_DistanceType get_distance_type() const;

    void set_last_real_distance(const DQ& y);

    double get_last_real_distance() const;

    void set_last_estimated_distance(const DQ& x_hat);

    double get_last_estimated_distance() const;

    std::string get_vfi_name() const;
};
