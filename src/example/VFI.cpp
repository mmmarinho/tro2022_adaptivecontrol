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


#include <dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <dqrobotics/utils/DQ_Geometry.h>
#include "marinholab/papers/tro2022/adaptive_control/VFI.h"


namespace marinholab::papers::tro2022::adaptive_control
{

VFI::VFI(const std::string &workspace_entity_name,
               const std::string& robot_entity_name,
               const Primitive &type,
               const std::shared_ptr<SimulatorDummy> &vi,
               const double &safe_distance,
               const VFI_Direction &vfi_direction,
               const int &joint_index,
               const DQ &relative_displacement_to_joint,
               const std::string &cs_reference_name):
    workspace_entity_name_(workspace_entity_name),
    robot_entity_name_(robot_entity_name),
    type_(type),
    vi_(vi),
    safe_distance_(safe_distance),
    vfi_direction_(vfi_direction),
    joint_index_(joint_index),
    relative_displacement_to_joint_(relative_displacement_to_joint),
    cs_reference_name_(cs_reference_name)
{
    // Do nothing
}

void VFI::initialize()
{
    //Reference pose is desired
    DQ x_ref(1);
    if (!cs_reference_name_.empty()) {
        x_ref = vi_->get_object_pose(cs_reference_name_);
    }

    switch(type_)
    {
    case Primitive::None:
        throw std::runtime_error("Expected valid type.");
    case Primitive::Point:
        throw std::runtime_error("Not implemented yet.");
    case Primitive::Plane:
    {
        const DQ x = conj(x_ref) * vi_->get_object_pose(workspace_entity_name_);
        const DQ r = rotation(x);
        const DQ n = Ad(r, k_);
        const DQ t = translation(x);
        set_value(n + E_*dot(t,n));
        return;
    }
    case Primitive::Line:
        const DQ x = conj(x_ref) * vi_->get_object_pose(workspace_entity_name_);
        const DQ r = rotation(x);
        const DQ l = Ad(r, k_);
        const DQ t = translation(x);
        set_value(l + E_*cross(t,l));
        return;
    }
}

DQ VFI::get_value() const
{
    return value_;
}

void VFI::set_value(const DQ &value)
{
    switch(type_)
    {
    case Primitive::None:
        throw std::runtime_error("Expected valid type.");
    case Primitive::Point:
        if(is_pure_quaternion(value))
        {
            value_ = value;
            return;
        }
        else
            throw std::runtime_error("Invalid point.");
    case Primitive::Plane:
        if(is_plane(value))
        {
            value_ = value;
            return;
        }
        else
            throw std::runtime_error("Invalid plane.");
    case Primitive::Line:
        if(is_line(value))
        {
            value_ = value;
            return;
        }
        else
            throw std::runtime_error("Invalid line.");
    }
}

MatrixXd VFI::get_distance_jacobian(const DQ &x, const MatrixXd &Jx) const
{
    //Consider the relative displacement
    const DQ& local_x = x*relative_displacement_to_joint_;
    const MatrixXd& local_Jx = haminus8(relative_displacement_to_joint_)*Jx;
    switch(type_)
    {
    case Primitive::None:
    {
        throw std::runtime_error("Expected valid type.");
    }
    case Primitive::Point:
    {
        throw std::runtime_error("Not implemented yet.");
    }
    case Primitive::Plane:
    {
        const MatrixXd Jt = DQ_Kinematics::translation_jacobian(local_Jx, local_x);
        const DQ t = translation(local_x);
        return DQ_Kinematics::point_to_plane_distance_jacobian(Jt, t, get_value());
    }
    case Primitive::Line:
    {
        const MatrixXd& Jt = DQ_Kinematics::translation_jacobian(local_Jx, local_x);
        const DQ& t = translation(local_x);
        return DQ_Kinematics::point_to_line_distance_jacobian(Jt, t, get_value());
    }
    }
    throw std::runtime_error("Unexpected end of method.");
}

MatrixXd VFI::get_vfi_matrix(const DQ &x, const MatrixXd &Jx) const
{
    switch(vfi_direction_)
    {
    case VFI_Direction::None:
    {
        throw std::runtime_error("Expected valid type");
    }
    case VFI_Direction::FORBIDDEN_ZONE:
    {
        //-Jd*q \leq \eta\tilde{d}, \tilde{d}=d-d_safe
        return -get_distance_jacobian(x, Jx);
    }
    case VFI_Direction::SAFE_ZONE:
    {
        //Jd*q \leq \eta\tilde{d}, \tilde{d}=d_safe-d
        return get_distance_jacobian(x, Jx);
    }
    }
    throw std::runtime_error("Unexpected end of method.");
}

double VFI::get_distance(const DQ &x) const
{
    //Consider the relative displacement
    const DQ& local_x = x*relative_displacement_to_joint_;
    switch(type_)
    {
    case Primitive::None:
    {
        throw std::runtime_error("Expected valid type.");
    }
    case Primitive::Point:
    {
        throw std::runtime_error("Not implemented yet.");
    }
    case Primitive::Plane:
    {
        const DQ& t = translation(local_x);
        return DQ_Geometry::point_to_plane_distance(t, get_value());
    }
    case Primitive::Line:
    {
        const DQ& t = translation(local_x);
        return DQ_Geometry::point_to_line_squared_distance(t, get_value());
    }
    }
    throw std::runtime_error("Unexpected end of method.");
}

double VFI::get_distance_error(const DQ &x) const
{
    switch(vfi_direction_)
    {
    case VFI_Direction::None:
        throw std::runtime_error("Expected valid type");
    case VFI_Direction::FORBIDDEN_ZONE:
    {
        //-Jd*q \leq \eta\tilde{d}, \tilde{d}=d-d_safe
        return (get_distance(x) - safe_distance_);
    }
    case VFI_Direction::SAFE_ZONE:
        //Jd*q \leq \eta\tilde{d}, \tilde{d}=d_safe-d
        return (safe_distance_ - get_distance(x));
    }
    throw std::runtime_error("Unexpected end of method.");
}

double VFI::get_safe_distance() const
{
    return safe_distance_;
}

VFI_DistanceType VFI::get_distance_type() const
{
    switch(type_)
    {
    case Primitive::None:
        throw std::runtime_error("Expected valid type.");
    case Primitive::Point:
    {
        return VFI_DistanceType::EUCLIDEAN_SQUARED;
    }
    case Primitive::Plane:
    {
        return VFI_DistanceType::EUCLIDEAN;
    }
    case Primitive::Line:
    {
        return VFI_DistanceType::EUCLIDEAN_SQUARED;
    }
    }
    throw std::runtime_error("Unexpected end of method.");
}

void VFI::set_last_real_distance(const DQ &y)
{
    last_real_distance_ = get_distance(y);
}

double VFI::get_last_real_distance() const
{
    return last_real_distance_;
}

void VFI::set_last_estimated_distance(const DQ &x_hat)
{
    last_estimated_distance_ = get_distance(x_hat);
}

double VFI::get_last_estimated_distance() const
{
    return last_estimated_distance_;
}

std::string VFI::get_vfi_name() const
{
    return workspace_entity_name_ + std::string("___") + robot_entity_name_;
}

}  // namespace marinholab::papers::tro2022::adaptive_control
