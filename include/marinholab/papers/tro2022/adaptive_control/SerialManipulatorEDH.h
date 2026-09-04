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

#include <vector>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>


namespace marinholab::papers::tro2022::adaptive_control
{

using namespace DQ_robotics;

// Contains EDH parameters and EDH parameter types
namespace ParameterSpaceEDH
{

// EDH parameter types
enum class Example_ParameterType
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

struct Example_Parameter
{
    int link_index_;
    ParameterSpaceEDH::Example_ParameterType type_;
    double value_;
    double min_;
    double max_;

    Example_Parameter(const int& link_index, const ParameterSpaceEDH::Example_ParameterType& type, const double& value=0, const double& min=0, const double& max=0):
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

class SerialManipulatorEDH : public DQ_SerialManipulator
{
protected:
    MatrixXd dh_matrix_;

    std::vector<ParameterSpaceEDH::Example_Parameter> parameter_space_;
    std::vector<ParameterSpaceEDH::Example_Parameter> base_parameters_;
    std::vector<ParameterSpaceEDH::Example_Parameter> eff_parameters_;

    void _check_link_index(const int& link_index) const;
    void _check_parameter_index(const int&parameter_index) const;
    void _check_base_parameters(const std::vector<ParameterSpaceEDH::Example_Parameter>& parameters);
    void _check_eff_parameters(const std::vector<ParameterSpaceEDH::Example_Parameter>& parameters);

    DQ _dh2dq(const double& joint_value, const int& link_index) const;
    DQ _get_w(const int& link_index) const;
    DQ _get_param_w(const double &joint_value, const ParameterSpaceEDH::Example_Parameter& parameter) const;
    DQ _get_base_param_w(const ParameterSpaceEDH::Example_ParameterType &parameter_type) const;
    DQ _get_eff_param_w(const ParameterSpaceEDH::Example_ParameterType &parameter_type) const;
    ParameterSpaceEDH::Example_Parameter _get_parameter(const int& parameter_index) const;
    double _get_parameter_space_value(const ParameterSpaceEDH::Example_Parameter& parameter) const;
    void _set_parameter_space_value(const ParameterSpaceEDH::Example_Parameter& parameter, const double& value);

    VectorXd _parameter_pose_jacobian_col(const VectorXd& joint_values, const int& parameter_index, const int& to_ith_link) const;

public:

    // Possible joint types
    enum JOINT_TYPES
    {
        JOINT_ROTATIONAL=0,
        JOINT_PRISMATIC
    };

    SerialManipulatorEDH()=delete;
    SerialManipulatorEDH(const MatrixXd& dh_matrix);

    DQ get_base_frame() const;
    std::vector<ParameterSpaceEDH::Example_Parameter> get_base_parameters() const;
    void set_base_frame(const DQ& base);
    void set_base_frame(const std::vector<ParameterSpaceEDH::Example_Parameter> &base_parameters);

    DQ get_effector_frame() const;
    std::vector<ParameterSpaceEDH::Example_Parameter> get_effector_parameters() const;
    void set_effector_frame(const std::vector<ParameterSpaceEDH::Example_Parameter>& effector_parameters);
    void set_effector_frame(const DQ& effector);

    void set_parameter_space(const std::vector<ParameterSpaceEDH::Example_Parameter>& parameter_space);
    bool is_parameter_space_set() const;
    int get_dim_parameter_space() const;

    VectorXd get_parameter_space_values() const;
    void set_parameter_space_values(const VectorXd& parameter_space_vector);
    void set_parameter_space_boundaries(const std::tuple<VectorXd, VectorXd>& boundaries);
    std::tuple<VectorXd,VectorXd> get_parameter_space_boundaries() const;
    std::vector<ParameterSpaceEDH::Example_ParameterType> get_parameter_types() const;

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
    std::vector<DQ_JointType> get_supported_joint_types() const override;

    //NOT IMPLEMENTED
    MatrixXd pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot, const int &to_ith_link) const override;
    MatrixXd raw_pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot, const int &to_ith_link) const override;
};

}  // namespace marinholab::papers::tro2022::adaptive_control
