#pragma once
/**
(C) Copyright 2020-2023 Murilo Marques Marinho (www.murilomarinho.info)

This file is part of DQ adaptive_control_example.

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


enum class DQ_Primitive
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

struct VFI_output
{
    double real_distance;
    double estimated_distance;
    VFI_output():
        real_distance(0),
        estimated_distance(0)
    {
    }
};

class VFI_Information
{
private:
    std::string workspace_entity_name_;
    std::string robot_entity_name_;
    DQ_Primitive type_;
    DQ value_;
    std::shared_ptr<DQ_VrepInterface> vi_;
    double safe_distance_;
    VFI_Direction vfi_direction_;
    const int joint_index_; //Needs to be correctly implemented in the future
    const DQ relative_displacement_to_joint_;
    const std::string cs_reference_name_;

    //New in this paper
    double last_estimated_distance_;
    double last_real_distance_;
public:
    VFI_Information(const std::string& workspace_entity_name,
                    const std::string& robot_entity_name,
                    const DQ_Primitive& type,
                    const std::shared_ptr<DQ_VrepInterface>& vi,
                    const double& safe_distance,
                    const VFI_Direction& vfi_direction,
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

    VFI_DistanceType get_distance_type() const;

    void set_last_real_distance(const DQ& y);

    double get_last_real_distance() const;

    void set_last_estimated_distance(const DQ& x_hat);

    double get_last_estimated_distance() const;

    std::string get_vfi_name() const;
};
