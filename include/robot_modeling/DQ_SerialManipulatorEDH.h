/*
# Copyright (c) 2020-2023 Murilo Marques Marinho (www.murilomarinho.info)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     - Neither the name of the Mitsuishi Sugita Laboratory (NML) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################*/

#ifndef DQ_SerialManipulatorEDH_H
#define DQ_SerialManipulatorEDH_H

#include <vector>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>

namespace DQ_robotics
{

// Contains EDH parameters and EDH parameter types
namespace DQ_ParameterSpaceEDH
{

// EDH parameter types
enum class ParameterType
{
    theta=1,
    d,
    a,
    alpha,

    base_x,
    base_y,
    base_z,
    base_alpha,
    base_beta,
    base_gamma,

    eff_x,
    eff_y,
    eff_z,
    eff_alpha,
    eff_beta,
    eff_gamma,
};

// EDH parameter
struct Parameter
{
    int link_index_;
    DQ_ParameterSpaceEDH::ParameterType type_;
    double value_;
    double min_;
    double max_;

    Parameter(const int& link_index, const DQ_ParameterSpaceEDH::ParameterType& type, const double& value=0, const double& min=0, const double& max=0):
        link_index_(link_index),
        type_(type),
        value_(value),
        min_(min),
        max_(max)
    {
        //Do nothing
    }
};
}


class DQ_SerialManipulatorEDH : public DQ_SerialManipulator
{
protected:
    MatrixXd dh_matrix_;

    std::vector<DQ_ParameterSpaceEDH::Parameter> parameter_space_;
    std::vector<DQ_ParameterSpaceEDH::Parameter> base_parameters_;
    std::vector<DQ_ParameterSpaceEDH::Parameter> eff_parameters_;

    void _check_link_index(const int& link_index) const;
    void _check_parameter_index(const int&parameter_index) const;
    void _check_base_parameters(const std::vector<DQ_ParameterSpaceEDH::Parameter>& parameters);
    void _check_eff_parameters(const std::vector<DQ_ParameterSpaceEDH::Parameter>& parameters);

    DQ _dh2dq(const double& joint_value, const int& link_index) const;
    DQ _get_w(const int& link_index) const;
    DQ _get_param_w(const double &joint_value, const DQ_ParameterSpaceEDH::Parameter& parameter) const;
    DQ _get_base_param_w(const DQ_ParameterSpaceEDH::ParameterType &parameter_type) const;
    DQ _get_eff_param_w(const DQ_ParameterSpaceEDH::ParameterType &parameter_type) const;
    DQ_ParameterSpaceEDH::Parameter _get_parameter(const int& parameter_index) const;
    double _get_parameter_space_value(const DQ_ParameterSpaceEDH::Parameter& parameter) const;
    void _set_parameter_space_value(const DQ_ParameterSpaceEDH::Parameter& parameter, const double& value);

    VectorXd _parameter_pose_jacobian_col(const VectorXd& joint_values, const int& parameter_index, const int& to_ith_link) const;

public:

    // Possible joint types
    enum JOINT_TYPES
    {
        JOINT_ROTATIONAL=0,
        JOINT_PRISMATIC
    };

    DQ_SerialManipulatorEDH()=delete;
    DQ_SerialManipulatorEDH(const MatrixXd& dh_matrix);


    DQ get_base_frame() const;
    std::vector<DQ_ParameterSpaceEDH::Parameter> get_base_parameters() const;
    void set_base_frame(const std::vector<DQ_ParameterSpaceEDH::Parameter> &base_parameters);
    void set_base_frame(const DQ& base);

    DQ get_effector_frame() const;
    std::vector<DQ_ParameterSpaceEDH::Parameter> get_effector_parameters() const;
    void set_effector_frame(const std::vector<DQ_ParameterSpaceEDH::Parameter>& effector_parameters);
    void set_effector_frame(const DQ& effector);

    void set_parameter_space(const std::vector<DQ_ParameterSpaceEDH::Parameter>& parameter_space);
    bool is_parameter_space_set() const;
    int get_dim_parameter_space() const;

    VectorXd get_parameter_space_values() const;
    void set_parameter_space_values(const VectorXd& parameter_space_vector);
    void set_parameter_space_boundaries(const std::tuple<VectorXd, VectorXd>& boundaries);
    std::tuple<VectorXd,VectorXd> get_parameter_space_boundaries() const;
    std::vector<DQ_ParameterSpaceEDH::ParameterType> get_parameter_types() const;

    VectorXd get_link_types() const;
    double get_link_type(const int& link_index) const;

    VectorXd get_thetas() const;
    double get_theta(const int& link_index) const;
    void set_theta(const int& link_index, double const& value);

    VectorXd get_ds() const;
    double get_d(const int& link_index) const;
    void set_d(const int& link_index, double const& value);

    VectorXd get_as() const;
    double get_a(const int& link_index) const;
    void set_a(const int& link_index, double const& value);

    VectorXd get_alphas() const;
    double get_alpha(const int& link_index) const;
    void set_alpha(const int& link_index, double const& value);

    MatrixXd parameter_pose_jacobian(const VectorXd& joint_values, const int& to_ith_link) const;
    MatrixXd parameter_pose_jacobian(const VectorXd& joint_values) const;

    //Virtual methods from DQ_SerialManipulator
    DQ raw_fkm(const VectorXd& joint_values, const int& to_ith_link) const override;
    MatrixXd raw_pose_jacobian(const VectorXd& joint_values, const int& to_ith_link) const override;
    DQ fkm (const VectorXd& joint_values) const override;
    DQ fkm (const VectorXd& joint_values, const int& to_ith_link) const override;
    MatrixXd pose_jacobian(const VectorXd& joint_values, const int& to_ith_link) const override;
    MatrixXd pose_jacobian (const VectorXd& joint_values) const override;
    int get_dim_configuration_space() const override;

    //NOT IMPLEMENTED
    MatrixXd pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot, const int &to_ith_link) const override;
    MatrixXd raw_pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot, const int &to_ith_link) const override;
};

}//Namespace DQ_robotics

#endif//DQ_SerialManipulatorEDH_H_INCLUDED

