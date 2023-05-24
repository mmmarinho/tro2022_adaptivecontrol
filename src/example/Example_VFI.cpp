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
#include "example/Example_VFI.h"


Example_VFI::Example_VFI(const std::string &workspace_entity_name,
                                 const std::string& robot_entity_name,
                                 const Example_Primitive &type,
                                 const std::shared_ptr<DQ_VrepInterface> &vi,
                                 const double &safe_distance,
                                 const Example_VFI_Direction &vfi_direction,
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
    if(joint_index_ != 7)
        throw std::runtime_error("Not implemented yet for anything besides joint_index_ == 7");
}

void Example_VFI::initialize()
{
    switch(type_)
    {
    case Example_Primitive::None:
        throw std::runtime_error("Expected valid type.");
    case Example_Primitive::Point:
        throw std::runtime_error("Not implemented yet.");
    case Example_Primitive::Plane:
    {
        const DQ x = vi_->get_object_pose(workspace_entity_name_,cs_reference_name_);
        const DQ r = rotation(x);
        const DQ n = Ad(r, k_);
        const DQ t = translation(x);
        set_value(n + E_*dot(t,n));
        return;
    }
    case Example_Primitive::Line:
        const DQ x = vi_->get_object_pose(workspace_entity_name_,cs_reference_name_);
        const DQ r = rotation(x);
        const DQ l = Ad(r, k_);
        const DQ t = translation(x);
        set_value(l + E_*cross(t,l));
        return;
    }
}

DQ Example_VFI::get_value() const
{
    return value_;
}

void Example_VFI::set_value(const DQ &value)
{
    switch(type_)
    {
    case Example_Primitive::None:
        throw std::runtime_error("Expected valid type.");
    case Example_Primitive::Point:
        if(is_pure_quaternion(value))
        {
            value_ = value;
            return;
        }
        else
            throw std::runtime_error("Invalid point.");
    case Example_Primitive::Plane:
        if(is_plane(value))
        {
            value_ = value;
            return;
        }
        else
            throw std::runtime_error("Invalid plane.");
    case Example_Primitive::Line:
        if(is_line(value))
        {
            value_ = value;
            return;
        }
        else
            throw std::runtime_error("Invalid line.");
    }
}

MatrixXd Example_VFI::get_distance_jacobian(const DQ &x, const MatrixXd &Jx) const
{
    //Consider the relative displacement
    const DQ& local_x = x*relative_displacement_to_joint_;
    const MatrixXd& local_Jx = haminus8(relative_displacement_to_joint_)*Jx;
    switch(type_)
    {
    case Example_Primitive::None:
    {
        throw std::runtime_error("Expected valid type.");
    }
    case Example_Primitive::Point:
    {
        throw std::runtime_error("Not implemented yet.");
    }
    case Example_Primitive::Plane:
    {
        const MatrixXd Jt = DQ_Kinematics::translation_jacobian(local_Jx, local_x);
        const DQ t = translation(local_x);
        return DQ_Kinematics::point_to_plane_distance_jacobian(Jt, t, get_value());
    }
    case Example_Primitive::Line:
    {
        const MatrixXd& Jt = DQ_Kinematics::translation_jacobian(local_Jx, local_x);
        const DQ& t = translation(local_x);
        return DQ_Kinematics::point_to_line_distance_jacobian(Jt, t, get_value());
    }
    }
    throw std::runtime_error("Unexpected end of method.");
}

MatrixXd Example_VFI::get_vfi_matrix(const DQ &x, const MatrixXd &Jx) const
{
    switch(vfi_direction_)
    {
    case Example_VFI_Direction::None:
    {
        throw std::runtime_error("Expected valid type");
    }
    case Example_VFI_Direction::FORBIDDEN_ZONE:
    {
        //-Jd*q \leq \eta\tilde{d}, \tilde{d}=d-d_safe
        return -get_distance_jacobian(x, Jx);
    }
    case Example_VFI_Direction::SAFE_ZONE:
    {
        //Jd*q \leq \eta\tilde{d}, \tilde{d}=d_safe-d
        return get_distance_jacobian(x, Jx);
    }
    }
    throw std::runtime_error("Unexpected end of method.");
}

double Example_VFI::get_distance(const DQ &x) const
{
    //Consider the relative displacement
    const DQ& local_x = x*relative_displacement_to_joint_;
    switch(type_)
    {
    case Example_Primitive::None:
    {
        throw std::runtime_error("Expected valid type.");
    }
    case Example_Primitive::Point:
    {
        throw std::runtime_error("Not implemented yet.");
    }
    case Example_Primitive::Plane:
    {
        const DQ& t = translation(local_x);
        return DQ_Geometry::point_to_plane_distance(t, get_value());
    }
    case Example_Primitive::Line:
    {
        const DQ& t = translation(local_x);
        return DQ_Geometry::point_to_line_squared_distance(t, get_value());
    }
    }
    throw std::runtime_error("Unexpected end of method.");
}

double Example_VFI::get_distance_error(const DQ &x) const
{
    switch(vfi_direction_)
    {
    case Example_VFI_Direction::None:
        throw std::runtime_error("Expected valid type");
    case Example_VFI_Direction::FORBIDDEN_ZONE:
    {
        //-Jd*q \leq \eta\tilde{d}, \tilde{d}=d-d_safe
        return (get_distance(x) - safe_distance_);
    }
    case Example_VFI_Direction::SAFE_ZONE:
        //Jd*q \leq \eta\tilde{d}, \tilde{d}=d_safe-d
        return (safe_distance_ - get_distance(x));
    }
    throw std::runtime_error("Unexpected end of method.");
}

double Example_VFI::get_safe_distance() const
{
    return safe_distance_;
}

Example_VFI_DistanceType Example_VFI::get_distance_type() const
{
    switch(type_)
    {
    case Example_Primitive::None:
        throw std::runtime_error("Expected valid type.");
    case Example_Primitive::Point:
    {
        return Example_VFI_DistanceType::EUCLIDEAN_SQUARED;
    }
    case Example_Primitive::Plane:
    {
        return Example_VFI_DistanceType::EUCLIDEAN;
    }
    case Example_Primitive::Line:
    {
        return Example_VFI_DistanceType::EUCLIDEAN_SQUARED;
    }
    }
    throw std::runtime_error("Unexpected end of method.");
}

void Example_VFI::set_last_real_distance(const DQ &y)
{
    last_real_distance_ = get_distance(y);
}

double Example_VFI::get_last_real_distance() const
{
    return last_real_distance_;
}

void Example_VFI::set_last_estimated_distance(const DQ &x_hat)
{
    last_estimated_distance_ = get_distance(x_hat);
}

double Example_VFI::get_last_estimated_distance() const
{
    return last_estimated_distance_;
}

std::string Example_VFI::get_vfi_name() const
{
    return workspace_entity_name_ + std::string("___") + robot_entity_name_;
}
