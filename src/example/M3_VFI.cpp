/**
(C) Copyright 2020-2025 Murilo Marques Marinho (www.murilomarinho.info)

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
        - Add cylinder VFI
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
#include "marinholab/papers/tro2022/adaptive_control/M3_VFI.h"

M3_VFI::M3_VFI(const std::string &workspace_entity_name,
               const std::string& robot_entity_name,
               const M3_Primitive &type,
               const std::shared_ptr<DQ_CoppeliaSimInterface> &vi,
               const double &safe_distance,
               const M3_VFI_Direction &vfi_direction,
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
}

/**
 * @brief Overloaded constructor that accepts multiple workspace geometric primitives
 *        to define the VFI constraint.
 */
M3_VFI::M3_VFI(const M3_Primitive &type,
               const double &safe_distance,
               const M3_VFI_Direction &vfi_direction,
               std::shared_ptr<M3_VFI> line,
               std::shared_ptr<M3_VFI> start_point,
               std::shared_ptr<M3_VFI> end_point,
               const DQ &relative_displacement_to_joint,
               const std::string &cs_reference_name):
safe_distance_(safe_distance),
vfi_direction_(vfi_direction),
joint_index_(0), // placeholder initialization of const int joint_index_
relative_displacement_to_joint_(relative_displacement_to_joint),
cs_reference_name_(cs_reference_name)
{
    if (type == M3_Primitive::Cylinder)
        type_ = type;
    else
        throw std::runtime_error("Expected valid type. Currently only supports M3_Primitive::Cylinder.");

    // line->initialize();
    // start_point->initialize();
    // end_point->initialize();

    primitives_.push_back(line);
    primitives_.push_back(start_point);
    primitives_.push_back(end_point);
}

/**
 * @brief Checks whether a point is inside a line segment.
 * @param point_in_line A pure quaternion representing a point in the line.
 * @param line_segment_start_point A pure quaternion representing the starting point of
 *        the line segment.
 * @param line_segment_end_point A pure quaternion representing the ending point of
 *        the line segment.
 * @return A tuple in which the first boolean indicates whether the point is inside the
 *         line segment and the second boolean indicates whether the point is closest to
 *         the starting point of the line segment.
 */
std::tuple<bool, bool> M3_VFI::check_if_point_is_inside_line_segment(
    const DQ& point_in_line,
    const DQ& line_segment_start_point,
    const DQ& line_segment_end_point) const
{
    double D_p_start = DQ_Geometry::point_to_point_squared_distance(
        point_in_line,
        line_segment_start_point);
    double D_p_end = DQ_Geometry::point_to_point_squared_distance(
        point_in_line,
        line_segment_end_point);
    double segment_size = DQ_Geometry::point_to_point_squared_distance(
        line_segment_start_point,
        line_segment_end_point);

    bool is_inside = ((D_p_start < segment_size) & (D_p_end < segment_size));
    bool is_closest_to_starting_point;
    if (D_p_start < D_p_end)
        is_closest_to_starting_point = true;
    else
        is_closest_to_starting_point = false;

    return std::make_tuple(is_inside, is_closest_to_starting_point);
}

/**
 * @brief Initialize the VFI constraint.
 */
void M3_VFI::initialize()
{
    //Reference pose is desired
    DQ x_ref(1);
    if (!cs_reference_name_.empty()) {
        x_ref = vi_->get_object_pose(cs_reference_name_);
    }

    switch(type_)
    {
    case M3_Primitive::None:
        throw std::runtime_error("Expected valid type.");
    case M3_Primitive::Point:
    {
        const DQ x = conj(x_ref) * vi_->get_object_pose(workspace_entity_name_);
        set_value(translation(x));
        return;
    }
    case M3_Primitive::Plane:
    {
        const DQ x = conj(x_ref) * vi_->get_object_pose(workspace_entity_name_);
        const DQ r = rotation(x);
        const DQ n = Ad(r, k_);
        const DQ t = translation(x);
        set_value(n + E_*dot(t,n));
        return;
    }
    case M3_Primitive::Line:
    {
        const DQ x = conj(x_ref) * vi_->get_object_pose(workspace_entity_name_);
        const DQ r = rotation(x);
        const DQ l = Ad(r, k_);
        const DQ t = translation(x);
        set_value(l + E_*cross(t,l));
        return;
    }
    case M3_Primitive::Cylinder:
        std::vector<DQ> cylinder;

        primitives_.at(0)->initialize();
        primitives_.at(1)->initialize();
        primitives_.at(2)->initialize();
        return;
    }
}

DQ M3_VFI::get_value() const
{
    return value_;
}

void M3_VFI::set_value(const DQ &value)
{
    switch(type_)
    {
    case M3_Primitive::None:
        throw std::runtime_error("Expected valid type.");
    case M3_Primitive::Point:
        if(is_pure_quaternion(value))
        {
            value_ = value;
            return;
        }
        else
            throw std::runtime_error("Invalid point.");
    case M3_Primitive::Plane:
        if(is_plane(value))
        {
            value_ = value;
            return;
        }
        else
            throw std::runtime_error("Invalid plane.");
    case M3_Primitive::Line:
        if(is_line(value))
        {
            value_ = value;
            return;
        }
        else
            throw std::runtime_error("Invalid line.");
    case M3_Primitive::Cylinder:
        throw std::runtime_error("Expected valid type. set_value() is not implemented to M3_Primitive::Cylinder.");
    }
}

MatrixXd M3_VFI::get_distance_jacobian(const DQ &x, const MatrixXd &Jx) const
{
    //Consider the relative displacement
    const DQ& local_x = x*relative_displacement_to_joint_;
    const MatrixXd& local_Jx = haminus8(relative_displacement_to_joint_)*Jx;
    switch(type_)
    {
    case M3_Primitive::None:
    {
        throw std::runtime_error("Expected valid type.");
    }
    case M3_Primitive::Point:
    {
        const MatrixXd Jt = DQ_Kinematics::translation_jacobian(local_Jx, local_x);
        const DQ t = translation(local_x);
        return DQ_Kinematics::point_to_point_distance_jacobian(local_Jx, t, get_value());
    }
    case M3_Primitive::Plane:
    {
        const MatrixXd Jt = DQ_Kinematics::translation_jacobian(local_Jx, local_x);
        const DQ t = translation(local_x);
        return DQ_Kinematics::point_to_plane_distance_jacobian(Jt, t, get_value());
    }
    case M3_Primitive::Line:
    {
        const MatrixXd& Jt = DQ_Kinematics::translation_jacobian(local_Jx, local_x);
        const DQ& t = translation(local_x);
        return DQ_Kinematics::point_to_line_distance_jacobian(Jt, t, get_value());
    }
    case M3_Primitive::Cylinder:
        DQ point_in_line = DQ_Geometry::point_projected_in_line(local_x.translation(),
                                                                primitives_.at(0)->get_value());
        bool is_inside, is_closest_to_starting_point;
        std::tie(is_inside, is_closest_to_starting_point) =
            this->check_if_point_is_inside_line_segment(point_in_line,
                                                        primitives_.at(1)->get_value(),
                                                        primitives_.at(2)->get_value());

        if (is_inside){ // get point-to-line distance jacobian
            return primitives_.at(0)->get_distance_jacobian(x, Jx);
        }else{
            if (is_closest_to_starting_point){ // get point-to-point distance jacobian considering the cylinder's starting point
                return primitives_.at(1)->get_distance_jacobian(x, Jx);
            }else{ // get point-to-point distance jacobian considering the cylinder's ending point
                return primitives_.at(2)->get_distance_jacobian(x, Jx);
            }
        }
    }
    throw std::runtime_error("Unexpected end of method.");
}

MatrixXd M3_VFI::get_vfi_matrix(const DQ &x, const MatrixXd &Jx) const
{
    switch(vfi_direction_)
    {
    case M3_VFI_Direction::None:
    {
        throw std::runtime_error("Expected valid type");
    }
    case M3_VFI_Direction::FORBIDDEN_ZONE:
    {
        //-Jd*q \leq \eta\tilde{d}, \tilde{d}=d-d_safe
        return -get_distance_jacobian(x, Jx);
    }
    case M3_VFI_Direction::SAFE_ZONE:
    {
        //Jd*q \leq \eta\tilde{d}, \tilde{d}=d_safe-d
        return get_distance_jacobian(x, Jx);
    }
    }
    throw std::runtime_error("Unexpected end of method.");
}

double M3_VFI::get_distance(const DQ &x) const
{
    //Consider the relative displacement
    const DQ& local_x = x*relative_displacement_to_joint_;
    switch(type_)
    {
    case M3_Primitive::None:
    {
        throw std::runtime_error("Expected valid type.");
    }
    case M3_Primitive::Point:
    {
        const DQ& t = translation(local_x);
        return DQ_Geometry::point_to_point_squared_distance(t, get_value());
    }
    case M3_Primitive::Plane:
    {
        const DQ& t = translation(local_x);
        return DQ_Geometry::point_to_plane_distance(t, get_value());
    }
    case M3_Primitive::Line:
    {
        const DQ& t = translation(local_x);
        return DQ_Geometry::point_to_line_squared_distance(t, get_value());
    }
    case M3_Primitive::Cylinder:
        DQ point_in_line = DQ_Geometry::point_projected_in_line(local_x.translation(),
                                                                primitives_.at(0)->get_value());
        bool is_inside, is_closest_to_starting_point;
        std::tie(is_inside, is_closest_to_starting_point) =
            this->check_if_point_is_inside_line_segment(point_in_line,
                                                        primitives_.at(1)->get_value(),
                                                        primitives_.at(2)->get_value());

        if (is_inside){ // get point-to-line distance
            return primitives_.at(0)->get_distance(x);
        }else{
            if (is_closest_to_starting_point){ // get point-to-point distance considering the cylinder's starting point
                return primitives_.at(1)->get_distance(x);
            }else{ // get point-to-point distance considering the cylinder's ending point
                return primitives_.at(2)->get_distance(x);
            }
        }
    }
    throw std::runtime_error("Unexpected end of method.");
}

double M3_VFI::get_distance_error(const DQ &x) const
{
    switch(vfi_direction_)
    {
    case M3_VFI_Direction::None:
        throw std::runtime_error("Expected valid type");
    case M3_VFI_Direction::FORBIDDEN_ZONE:
    {
        //-Jd*q \leq \eta\tilde{d}, \tilde{d}=d-d_safe
        // if (type_ == M3_Primitive::Cylinder){
        //     std::cerr << "For the cylinder: distance = " << get_distance(x) << std::endl;
        //     std::cerr << "For the cylinder: safe_distance = " << safe_distance_ << std::endl;
        //     std::cerr << "For the cylinder: distance - safe_distance = " << get_distance(x) - safe_distance_ << std::endl;
        // }
        // if (type_ == M3_Primitive::Line){
        //     std::cerr << "For the line: distance = " << get_distance(x) << std::endl;
        //     std::cerr << "For the line: safe_distance = " << safe_distance_ << std::endl;
        //     std::cerr << "For the line: distance - safe_distance = " << get_distance(x) - safe_distance_ << std::endl;
        // }
        return (get_distance(x) - safe_distance_);
    }
    case M3_VFI_Direction::SAFE_ZONE:
        //Jd*q \leq \eta\tilde{d}, \tilde{d}=d_safe-d
        return (safe_distance_ - get_distance(x));
    }
    throw std::runtime_error("Unexpected end of method.");
}

double M3_VFI::get_safe_distance() const
{
    return safe_distance_;
}

M3_VFI_DistanceType M3_VFI::get_distance_type(const DQ &x) const
{
    switch(type_)
    {
    case M3_Primitive::None:
        throw std::runtime_error("Expected valid type.");
    case M3_Primitive::Point:
    {
        return M3_VFI_DistanceType::EUCLIDEAN_SQUARED;
    }
    case M3_Primitive::Plane:
    {
        return M3_VFI_DistanceType::EUCLIDEAN;
    }
    case M3_Primitive::Line:
    {
        return M3_VFI_DistanceType::EUCLIDEAN_SQUARED;
    }
    case M3_Primitive::Cylinder:
        //Consider the relative displacement
        const DQ& local_x = x*relative_displacement_to_joint_;
        DQ point_in_line = DQ_Geometry::point_projected_in_line(local_x.translation(),
                                                                primitives_.at(0)->get_value());
        bool is_inside, is_closest_to_starting_point;
        std::tie(is_inside, is_closest_to_starting_point) =
            this->check_if_point_is_inside_line_segment(point_in_line,
                                                        primitives_.at(1)->get_value(),
                                                        primitives_.at(2)->get_value());

        if (is_inside){ // get point-to-line distance type
            return primitives_.at(0)->get_distance_type(local_x);
        }else{ // get point-to-point distance type (it's the same whether closest to the cylinder's starting or ending point)
            return primitives_.at(1)->get_distance_type(local_x);
        }
    }
    throw std::runtime_error("Unexpected end of method.");
}

void M3_VFI::set_last_real_distance(const DQ &y)
{
    last_real_distance_ = get_distance(y);
}

double M3_VFI::get_last_real_distance() const
{
    return last_real_distance_;
}

void M3_VFI::set_last_estimated_distance(const DQ &x_hat)
{
    last_estimated_distance_ = get_distance(x_hat);
}

double M3_VFI::get_last_estimated_distance() const
{
    return last_estimated_distance_;
}

std::string M3_VFI::get_vfi_name() const
{
    return workspace_entity_name_ + std::string("___") + robot_entity_name_;
}
