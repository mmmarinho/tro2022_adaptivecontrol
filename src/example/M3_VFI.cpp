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
        - Add cylinder VFI and associated methods
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
#include <dqrobotics/internal/_dq_linesegment.h>
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
    // Do nothing
}

/**
 * @brief Overloaded constructor that accepts multiple workspace geometric primitives
 *        to define the VFI constraint.
 */
M3_VFI::M3_VFI(const M3_Primitive &type,
               const std::shared_ptr<DQ_CoppeliaSimInterface> &vi,
               const double &safe_distance,
               const M3_VFI_Direction &vfi_direction,
               std::shared_ptr<M3_VFI> line,
               std::shared_ptr<M3_VFI> start_point,
               std::shared_ptr<M3_VFI> end_point,
               const DQ &relative_displacement_to_joint,
               const std::string &cs_reference_name):
vi_(vi),
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

    primitives_.push_back(line);
    primitives_.push_back(start_point);
    primitives_.push_back(end_point);

    workspace_entity_name_ = line->robot_entity_name_ + " to cylinder";
    robot_entity_name_ = line->workspace_entity_name_;
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
        throw std::runtime_error("Does not support M3_Primitive::Cylinder type."
                                 "Use initialize_dynamic_geometric_primitives() instead!");
    }
}

/**
 * @brief Initialize dynamic VFI constraints. Currently only supports dynamic geometric primitives attached to a robot.
 * @param robot_ptr A std::shared_ptr to a DQ_SerialManipulator object representing the robot.
 * @param q A VectorXd representing the robot's configuration.
 */
void M3_VFI::initialize_dynamic_geometric_primitives(std::shared_ptr<DQ_SerialManipulator> robot_ptr,
                                                     const VectorXd &q)
{
    //Reference pose is desired
    DQ x_ref(1);
    if (!cs_reference_name_.empty()) {
        x_ref = vi_->get_object_pose(cs_reference_name_);
    }

    switch(type_)
    {
    case M3_Primitive::Cylinder:
    {
        // Calculate the relative pose to the cylinder's line
        DQ robot_dh_frame = robot_ptr->fkm(q, primitives_.at(0)->joint_index_);
        DQ x_primitive = vi_->get_object_pose(primitives_.at(0)->workspace_entity_name_);
        primitives_.at(0)->relative_displacement_to_primitive_ = conj(robot_dh_frame)*conj(x_ref)*x_primitive;

        // Calculate the relative pose to the cylinder's starting point
        robot_dh_frame = robot_ptr->fkm(q, primitives_.at(1)->joint_index_);
        x_primitive = vi_->get_object_pose(primitives_.at(1)->workspace_entity_name_);
        primitives_.at(1)->relative_displacement_to_primitive_ = conj(robot_dh_frame)*conj(x_ref)*x_primitive;


        // Calculate the relative pose to the cylinder's ending point
        robot_dh_frame = robot_ptr->fkm(q, primitives_.at(2)->joint_index_);
        x_primitive = vi_->get_object_pose(primitives_.at(2)->workspace_entity_name_);
        primitives_.at(2)->relative_displacement_to_primitive_ = conj(robot_dh_frame)*conj(x_ref)*x_primitive;

        return;
    }
    default:
    {
        throw std::runtime_error("Expected valid type. Currently only supports M3_Primitive::Cylinder.");
    }
    }
}

/**
 * @brief Update a dynamic geometric primitive. Currently only supports dynamic geometric primitives attached to a robot.
 * @param robot_ptr A std::shared_ptr to a DQ_SerialManipulator object representing the robot.
 * @param q A VectorXd representing the robot's configuration.
 */
void M3_VFI::update_dynamic_geometric_primitives(std::shared_ptr<DQ_SerialManipulator> robot_ptr,
                                                 const VectorXd &q)
{
    const DQ& local_x = (robot_ptr->fkm(q, joint_index_))*relative_displacement_to_primitive_;
    switch(type_)
    {
    case M3_Primitive::None:
        throw std::runtime_error("Expected valid type.");
    case M3_Primitive::Point:
    {
        set_value(translation(local_x));
        return;
    }
    case M3_Primitive::Plane:
    {
        const DQ r = rotation(local_x);
        const DQ n = Ad(r, k_);
        const DQ t = translation(local_x);
        set_value(n + E_*dot(t,n));
        return;
    }
    case M3_Primitive::Line:
    {
        const DQ r = rotation(local_x);
        const DQ l = Ad(r, k_);
        const DQ t = translation(local_x);
        set_value(l + E_*cross(t,l));
        return;
    }
    case M3_Primitive::Cylinder:
        primitives_.at(0)->update_dynamic_geometric_primitives(robot_ptr, q);
        primitives_.at(1)->update_dynamic_geometric_primitives(robot_ptr, q);
        primitives_.at(2)->update_dynamic_geometric_primitives(robot_ptr, q);


        // Make sure the points are properly projected into the line
        DQ start_point_in_line = DQ_Geometry::point_projected_in_line(primitives_.at(1)->get_value(),
                                                                      primitives_.at(0)->get_value());
        primitives_.at(1)->set_value(start_point_in_line);

        DQ end_point_in_line = DQ_Geometry::point_projected_in_line(primitives_.at(2)->get_value(),
                                                                    primitives_.at(0)->get_value());
        primitives_.at(2)->set_value(end_point_in_line);

        return;
    }
}

/**
 * @brief Directly update a cylinder geometric primitive.
 * @param line The cylinder's line.
 * @param start_point The cylinder's starting point.
 * @param end_point The cylinder's ending point.
 */
void M3_VFI::update_cylinder_vfi(const DQ& line, const DQ& start_point, const DQ& end_point)
{
    primitives_.at(0)->set_value(line);
    primitives_.at(1)->set_value(start_point);
    primitives_.at(2)->set_value(end_point);

    // Make sure the points are properly projected into the line
    DQ start_point_in_line = DQ_Geometry::point_projected_in_line(primitives_.at(1)->get_value(),
                                                                  primitives_.at(0)->get_value());
    primitives_.at(1)->set_value(start_point_in_line);

    DQ end_point_in_line = DQ_Geometry::point_projected_in_line(primitives_.at(2)->get_value(),
                                                                primitives_.at(0)->get_value());
    primitives_.at(2)->set_value(end_point_in_line);
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
        // Define a cylinder at the end-effector with its starting point equal to its ending point
        // to create a sphere. This allow us to use DQ_robotics::internal::LineSegment::closest_elements_between_line_segments()
        const DQ l = k_;
        const DQ l_eff = l + E_*(DQ_robotics::cross(local_x.translation(), l));

        auto ce = DQ_robotics::internal::LineSegment::closest_elements_between_line_segments(
            {l_eff, local_x.translation(), local_x.translation()},
            {primitives_.at(0)->get_value(), primitives_.at(1)->get_value(), primitives_.at(2)->get_value()});

        switch(std::get<1>(std::get<0>(ce)))
        {
        case DQ_robotics::internal::LineSegment::Element::Line: // get point-to-line distance jacobian
            return primitives_.at(0)->get_distance_jacobian(x, Jx);
        case DQ_robotics::internal::LineSegment::Element::P1: // get point-to-point distance jacobian considering the cylinder's starting point
            return primitives_.at(1)->get_distance_jacobian(x, Jx);
        case DQ_robotics::internal::LineSegment::Element::P2: // get point-to-point distance jacobian considering the cylinder's ending point
            return primitives_.at(2)->get_distance_jacobian(x, Jx);
        }
        throw std::runtime_error("Unexpected type in M3_VFI::get_distance_jacobian()");
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
        // Define a cylinder at the end-effector with its starting point equal to its ending point
        // to create a sphere. This allow us to use DQ_Geometry::line_segment_to_line_segment_squared_distance()
        const DQ l = k_;
        const DQ l_eff = l + E_*(DQ_robotics::cross(local_x.translation(), l));

        return DQ_Geometry::line_segment_to_line_segment_squared_distance(l_eff, // end-effector line
                                                                          local_x.translation(), // end-effecor position
                                                                          local_x.translation(), // end-effecor position
                                                                          primitives_.at(0)->get_value(),  // cylinder's line
                                                                          primitives_.at(1)->get_value(),  // cylinder's starting point
                                                                          primitives_.at(2)->get_value());  // cylinder's ending point
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


/**
 * @brief Get the distance type of the VFI (e.g., Euclidean, Euclidean squared, etc.).
 */
M3_VFI_DistanceType M3_VFI::get_distance_type() const
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
        return M3_VFI_DistanceType::EUCLIDEAN_SQUARED;
    }
    throw std::runtime_error("Unexpected end of method.");
}

/**
 * @brief Get the VFI type.
 * @return A M3_Primitive represeting the VFI type.
 */
M3_Primitive M3_VFI::get_type() const
{
    return type_;
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
