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
#include <example/Example_SerialManipulatorEDH.h>

Example_SerialManipulatorEDH::Example_SerialManipulatorEDH(const MatrixXd& dh_matrix):
    DQ_SerialManipulator(dh_matrix.cols()),
    dh_matrix_(dh_matrix)
{

}

/**
 * @brief Example_SerialManipulatorEDH::get_base_frame Obtains the base of the robot, with
 * respect to the measurement system, as a DQ.
 *
 * As described in the paper, the base transformation is **NOT** described using DH parameters.
 * This serves to show that this work is not only for DH parameters.
 * The base transformation is described by a 3D translation, followed by a rotation about
 * the x-axis, the y-axis, and the z-axis. This choice was arbitrary and anything will do as long
 * as _get_base_param_w is updated to match it.
 *
 * In the paper, we left DQ algebra in the sidelines to not distract the reader and further increase
 * the length of the paper. However, for a possible implementation of this on DQ Robotics, it might
 * be better to rewrite this and _get_base_param_w to use the DQ directly. Seems to be somewhat
 * trivial as long as DQs are in the spotlight.
 *
 * M. M. Marinho and B. V. Adorno,
 * "Adaptive Constrained Kinematic Control Using Partial or Complete Task-Space Measurements,"
 * in IEEE Transactions on Robotics, vol. 38, no. 6, pp. 3498-3513, Dec. 2022,
 * doi: 10.1109/TRO.2022.3181047.
 *
 * @return a DQ representing the base of the robot.
 */
DQ Example_SerialManipulatorEDH::get_base_frame() const
{
    if(base_parameters_.size()==6)
    {
        const DQ t(0.,
                   base_parameters_.at(0).value_,
                   base_parameters_.at(1).value_,
                   base_parameters_.at(2).value_);

        const double& alpha = base_parameters_.at(3).value_;
        const DQ ra = cos(alpha/2.0) + i_*sin(alpha/2.0);

        const double& beta = base_parameters_.at(4).value_;
        const DQ rb = cos(beta/2.0) + j_*sin(beta/2.0);

        const double& gamma = base_parameters_.at(5).value_;
        const DQ rg = cos(gamma/2.0) + k_*sin(gamma/2.0);

        const DQ r = ra*rb*rg;

        return (r + 0.5*E_*t*r);
    }
    else
    {
        throw std::runtime_error("DQ_SerialManipulatorDH::get_base_frame()::Error::The frame must be initialized before usage.");
    }
}

std::vector<Example_ParameterSpaceEDH::Example_Parameter> Example_SerialManipulatorEDH::get_base_parameters() const
{
    if(base_parameters_.size()==6)
        return base_parameters_;
    else
        throw std::runtime_error("DQ_SerialManipulatorDH::get_base_parameters()::Error::The parameters must be initialized before usage.");
}

void Example_SerialManipulatorEDH::set_base_frame(const std::vector<Example_ParameterSpaceEDH::Example_Parameter>& base_parameters)
{
    _check_base_parameters(base_parameters);
    base_parameters_ = base_parameters;
}

/**
 * @brief Example_SerialManipulatorEDH::set_base_frame Sets the base frame using base parameters.
 *
 * @param base a DQ to be broken into parameters.
 *
 * @see get_base_frame for a detailed explanation on the parameters.
 */
void Example_SerialManipulatorEDH::set_base_frame(const DQ &base)
{
    //The vector constructor of Eigen3 changes the order of the quaternion with scalar last.
    //The constructor using doubles has the "correct" order with scalar first
    //confusing but that is life.
    Vector3d base_euler = Quaterniond(base.q[0],base.q[1],base.q[2],base.q[3]).toRotationMatrix().eulerAngles(0, 1, 2);

    Vector3d base_t = vec3(translation(base));
    std::vector<Example_ParameterSpaceEDH::Example_Parameter> base_parameters = {
        Example_ParameterSpaceEDH::Example_Parameter(-1, Example_ParameterSpaceEDH::Example_ParameterType::base_x,     base_t(0)),
        Example_ParameterSpaceEDH::Example_Parameter(-1, Example_ParameterSpaceEDH::Example_ParameterType::base_y,     base_t(1)),
        Example_ParameterSpaceEDH::Example_Parameter(-1, Example_ParameterSpaceEDH::Example_ParameterType::base_z,     base_t(2)),
        Example_ParameterSpaceEDH::Example_Parameter(-1, Example_ParameterSpaceEDH::Example_ParameterType::base_alpha, base_euler(0)),
        Example_ParameterSpaceEDH::Example_Parameter(-1, Example_ParameterSpaceEDH::Example_ParameterType::base_beta,  base_euler(1)),
        Example_ParameterSpaceEDH::Example_Parameter(-1, Example_ParameterSpaceEDH::Example_ParameterType::base_gamma, base_euler(2)),
    };
    set_base_frame(base_parameters);

    //Verify if the rotation was correctly reconstructed
    if((vec4(rotation(base))-vec4(rotation(get_base_frame()))).cwiseAbs().maxCoeff() > DQ_robotics::DQ_threshold)
    {
        throw std::runtime_error("DQ_SerialManipulatorRDH::set_base_frame()::Error::Rotation quaternion could not be correctly reconstructed (a!=b). \n"
                                 "a= " + rotation(base).to_string() + " b=" + rotation(get_base_frame()).to_string() + "\n" +
                                 + "error=" + std::to_string((vec4(rotation(base))-vec4(rotation(get_base_frame()))).cwiseAbs().maxCoeff()) );
    }
}

/**
 * @brief Example_SerialManipulatorEDH::get_effector_frame Obtains the effector of the robot, with
 * respect to the measurement system, as a DQ.
 *
 * @return a DQ representing the base of the robot.
 *
 * @see get_base_frame for a detailed explanation on the parameters.
 */
DQ Example_SerialManipulatorEDH::get_effector_frame() const
{
    if(eff_parameters_.size()==6)
    {
        const DQ t(0.,
                   eff_parameters_.at(0).value_,
                   eff_parameters_.at(1).value_,
                   eff_parameters_.at(2).value_);

        const double& alpha = eff_parameters_.at(3).value_;
        const DQ ra = cos(alpha/2.0) + i_*sin(alpha/2.0);

        const double& beta = eff_parameters_.at(4).value_;
        const DQ rb = cos(beta/2.0) + j_*sin(beta/2.0);

        const double& gamma = eff_parameters_.at(5).value_;
        const DQ rg = cos(gamma/2.0) + k_*sin(gamma/2.0);

        const DQ r = ra*rb*rg;

        return (r + 0.5*E_*t*r);
    }
    else
    {
        throw std::runtime_error("DQ_SerialManipulatorDH::get_effector_frame()::Error::The frame must be initialized before usage.");
    }
}

std::vector<Example_ParameterSpaceEDH::Example_Parameter> Example_SerialManipulatorEDH::get_effector_parameters() const
{
    if(eff_parameters_.size() == 6)
        return eff_parameters_;
    else
        throw std::runtime_error("DQ_SerialManipulatorDH::get_effector_parameters()::Error::The parameters must be initialized before usage.");
}

void Example_SerialManipulatorEDH::set_effector_frame(const std::vector<Example_ParameterSpaceEDH::Example_Parameter> &effector_parameters)
{
    _check_eff_parameters(effector_parameters);
    eff_parameters_ = effector_parameters;
}

/**
 * @brief Example_SerialManipulatorEDH::set_effector_frame Sets the effector of the robot, with
 * respect to the measurement system, using a DQ.
 *
 * @param effector a DQ representing the effector of the robot.
 *
 * @see get_base_frame for a detailed explanation on the parameters.
 */
void Example_SerialManipulatorEDH::set_effector_frame(const DQ &effector)
{
    //The vector constructor of Eigen3 changes the order of the quaternion with scalar last.
    //The constructor using doubles has the "correct" order with scalar first
    //confusing but that is life.
    Vector3d eff_euler = Quaterniond(effector.q[0],effector.q[1],effector.q[2],effector.q[3]).toRotationMatrix().eulerAngles(0, 1, 2);
    Vector3d eff_t = vec3(translation(effector));

    std::vector<Example_ParameterSpaceEDH::Example_Parameter> eff_parameters = {
        Example_ParameterSpaceEDH::Example_Parameter(6, Example_ParameterSpaceEDH::Example_ParameterType::eff_x,     eff_t(0)),
        Example_ParameterSpaceEDH::Example_Parameter(6, Example_ParameterSpaceEDH::Example_ParameterType::eff_y,     eff_t(1)),
        Example_ParameterSpaceEDH::Example_Parameter(6, Example_ParameterSpaceEDH::Example_ParameterType::eff_z,     eff_t(2)),
        Example_ParameterSpaceEDH::Example_Parameter(6, Example_ParameterSpaceEDH::Example_ParameterType::eff_alpha, eff_euler(0)),
        Example_ParameterSpaceEDH::Example_Parameter(6, Example_ParameterSpaceEDH::Example_ParameterType::eff_beta,  eff_euler(1)),
        Example_ParameterSpaceEDH::Example_Parameter(6, Example_ParameterSpaceEDH::Example_ParameterType::eff_gamma, eff_euler(2)),
    };
    set_effector_frame(eff_parameters);

    //Verify if the rotation was correctly reconstructed
    if((vec4(rotation(effector))-vec4(rotation(get_effector_frame()))).cwiseAbs().maxCoeff() > DQ_robotics::DQ_threshold)
    {
        throw std::runtime_error("DQ_SerialManipulatorRDH::set_effector_frame()::Error::Rotation quaternion could not be correctly reconstructed (a!=b). \n"
                                 "a= " + rotation(effector).to_string() + " b=" + rotation(get_effector_frame()).to_string() + "\n" +
                                 + "error=" + std::to_string((vec4(rotation(effector))-vec4(rotation(get_effector_frame()))).cwiseAbs().maxCoeff()) );
    }
}

void Example_SerialManipulatorEDH::set_parameter_space(const std::vector<Example_ParameterSpaceEDH::Example_Parameter> &parameter_space)
{
    parameter_space_ = parameter_space;
    VectorXd parameter_space_values = VectorXd(parameter_space.size());
    for(int i=0;i<parameter_space.size();i++)
    {
        parameter_space_values(i) = parameter_space[i].value_;
    }
    set_parameter_space_values(parameter_space_values);
}

int Example_SerialManipulatorEDH::get_dim_parameter_space() const
{
    return parameter_space_.size();
}

bool Example_SerialManipulatorEDH::is_parameter_space_set() const
{
    return (get_dim_parameter_space()>0);
}

Example_ParameterSpaceEDH::Example_Parameter Example_SerialManipulatorEDH::_get_parameter(const int &parameter_index) const
{
    return parameter_space_[parameter_index];
}

double Example_SerialManipulatorEDH::_get_parameter_space_value(const Example_ParameterSpaceEDH::Example_Parameter &parameter) const
{
    switch (parameter.type_)
    {
    case Example_ParameterSpaceEDH::Example_ParameterType::theta:
        return get_theta(parameter.link_index_);
    case Example_ParameterSpaceEDH::Example_ParameterType::d:
        return get_d(parameter.link_index_);
    case Example_ParameterSpaceEDH::Example_ParameterType::a:
        return get_a(parameter.link_index_);
    case Example_ParameterSpaceEDH::Example_ParameterType::alpha:
        return get_alpha(parameter.link_index_);

    case Example_ParameterSpaceEDH::Example_ParameterType::base_x:
        return base_parameters_.at(0).value_;
    case Example_ParameterSpaceEDH::Example_ParameterType::base_y:
        return base_parameters_.at(1).value_;
    case Example_ParameterSpaceEDH::Example_ParameterType::base_z:
        return base_parameters_.at(2).value_;
    case Example_ParameterSpaceEDH::Example_ParameterType::base_alpha:
        return base_parameters_.at(3).value_;
    case Example_ParameterSpaceEDH::Example_ParameterType::base_beta:
        return base_parameters_.at(4).value_;
    case Example_ParameterSpaceEDH::Example_ParameterType::base_gamma:
        return base_parameters_.at(5).value_;

    case Example_ParameterSpaceEDH::Example_ParameterType::eff_x:
        return eff_parameters_.at(0).value_;
    case Example_ParameterSpaceEDH::Example_ParameterType::eff_y:
        return eff_parameters_.at(1).value_;
    case Example_ParameterSpaceEDH::Example_ParameterType::eff_z:
        return eff_parameters_.at(2).value_;
    case Example_ParameterSpaceEDH::Example_ParameterType::eff_alpha:
        return eff_parameters_.at(3).value_;
    case Example_ParameterSpaceEDH::Example_ParameterType::eff_beta:
        return eff_parameters_.at(4).value_;
    case Example_ParameterSpaceEDH::Example_ParameterType::eff_gamma:
        return eff_parameters_.at(5).value_;
    }
    throw std::runtime_error("Invalid parameter type in _get_parameter_space_value");
}

void Example_SerialManipulatorEDH::_set_parameter_space_value(const Example_ParameterSpaceEDH::Example_Parameter &parameter, const double &value)
{
    switch (parameter.type_)
    {
    case Example_ParameterSpaceEDH::Example_ParameterType::theta:
        set_theta(parameter.link_index_, value);
        return;
    case Example_ParameterSpaceEDH::Example_ParameterType::d:
        set_d(parameter.link_index_, value);
        return;
    case Example_ParameterSpaceEDH::Example_ParameterType::a:
        set_a(parameter.link_index_, value);
        return;
    case Example_ParameterSpaceEDH::Example_ParameterType::alpha:
        set_alpha(parameter.link_index_, value);
        return;

    case Example_ParameterSpaceEDH::Example_ParameterType::base_x:
        base_parameters_[0].value_ = value;
        return;
    case Example_ParameterSpaceEDH::Example_ParameterType::base_y:
        base_parameters_[1].value_ = value;
        return;
    case Example_ParameterSpaceEDH::Example_ParameterType::base_z:
        base_parameters_[2].value_ = value;
        return;
    case Example_ParameterSpaceEDH::Example_ParameterType::base_alpha:
        base_parameters_[3].value_ = value;
        return;
    case Example_ParameterSpaceEDH::Example_ParameterType::base_beta:
        base_parameters_[4].value_ = value;
        return;
    case Example_ParameterSpaceEDH::Example_ParameterType::base_gamma:
        base_parameters_[5].value_ = value;
        return;

    case Example_ParameterSpaceEDH::Example_ParameterType::eff_x:
        eff_parameters_[0].value_ = value;
        return;
    case Example_ParameterSpaceEDH::Example_ParameterType::eff_y:
        eff_parameters_[1].value_ = value;
        return;
    case Example_ParameterSpaceEDH::Example_ParameterType::eff_z:
        eff_parameters_[2].value_ = value;
        return;
    case Example_ParameterSpaceEDH::Example_ParameterType::eff_alpha:
        eff_parameters_[3].value_ = value;
        return;
    case Example_ParameterSpaceEDH::Example_ParameterType::eff_beta:
        eff_parameters_[4].value_ = value;
        return;
    case Example_ParameterSpaceEDH::Example_ParameterType::eff_gamma:
        eff_parameters_[5].value_ = value;
        return;
    }
    throw std::runtime_error("Invalid parameter type in _set_parameter_space_value");
}

VectorXd Example_SerialManipulatorEDH::get_parameter_space_values() const
{
    VectorXd parameter_space_vector(get_dim_parameter_space());

    for(int parameter_index=0; parameter_index < get_dim_parameter_space(); parameter_index++)
    {
        const Example_ParameterSpaceEDH::Example_Parameter& parameter = parameter_space_[parameter_index];
        parameter_space_vector(parameter_index) = _get_parameter_space_value(parameter);
    }

    return parameter_space_vector;
}

void Example_SerialManipulatorEDH::set_parameter_space_values(const VectorXd &parameter_space_vector)
{
    if(parameter_space_vector.size() != get_dim_parameter_space())
        throw std::runtime_error("parameter_space_vector incompatible with dim_parameter_space in set_parameter_space_vector");

    for(int parameter_index=0; parameter_index < get_dim_parameter_space(); parameter_index++)
    {
        const Example_ParameterSpaceEDH::Example_Parameter& parameter = parameter_space_[parameter_index];
        switch (parameter.type_)
        {
        case Example_ParameterSpaceEDH::Example_ParameterType::theta:
            set_theta(parameter.link_index_, parameter_space_vector(parameter_index));
            break;
        case Example_ParameterSpaceEDH::Example_ParameterType::d:
            set_d(parameter.link_index_, parameter_space_vector(parameter_index));
            break;
        case Example_ParameterSpaceEDH::Example_ParameterType::a:
            set_a(parameter.link_index_, parameter_space_vector(parameter_index));
            break;
        case Example_ParameterSpaceEDH::Example_ParameterType::alpha:
            set_alpha(parameter.link_index_, parameter_space_vector(parameter_index));
            break;

        // Base Parameters
        case Example_ParameterSpaceEDH::Example_ParameterType::base_x:
            base_parameters_[0].value_ = parameter_space_vector(parameter_index);
            break;
        case Example_ParameterSpaceEDH::Example_ParameterType::base_y:
            base_parameters_[1].value_ = parameter_space_vector(parameter_index);
            break;
        case Example_ParameterSpaceEDH::Example_ParameterType::base_z:
            base_parameters_[2].value_ = parameter_space_vector(parameter_index);
            break;
        case Example_ParameterSpaceEDH::Example_ParameterType::base_alpha:
            base_parameters_[3].value_ = parameter_space_vector(parameter_index);
            break;
        case Example_ParameterSpaceEDH::Example_ParameterType::base_beta:
            base_parameters_[4].value_ = parameter_space_vector(parameter_index);
            break;
        case Example_ParameterSpaceEDH::Example_ParameterType::base_gamma:
            base_parameters_[5].value_ = parameter_space_vector(parameter_index);
            break;

        // End Effector Parameters
        case Example_ParameterSpaceEDH::Example_ParameterType::eff_x:
            eff_parameters_[0].value_ = parameter_space_vector(parameter_index);
            break;
        case Example_ParameterSpaceEDH::Example_ParameterType::eff_y:
            eff_parameters_[1].value_ = parameter_space_vector(parameter_index);
            break;
        case Example_ParameterSpaceEDH::Example_ParameterType::eff_z:
            eff_parameters_[2].value_ = parameter_space_vector(parameter_index);
            break;
        case Example_ParameterSpaceEDH::Example_ParameterType::eff_alpha:
            eff_parameters_[3].value_ = parameter_space_vector(parameter_index);
            break;
        case Example_ParameterSpaceEDH::Example_ParameterType::eff_beta:
            eff_parameters_[4].value_ = parameter_space_vector(parameter_index);
            break;
        case Example_ParameterSpaceEDH::Example_ParameterType::eff_gamma:
            eff_parameters_[5].value_ = parameter_space_vector(parameter_index);
            break;
        }
    }
}

void Example_SerialManipulatorEDH::set_parameter_space_boundaries(const std::tuple<VectorXd, VectorXd>& boundaries)
{
    for(int parameter_counter=0;parameter_counter<get_dim_parameter_space();parameter_counter++)
    {
        Example_ParameterSpaceEDH::Example_Parameter& parameter = parameter_space_[parameter_counter];
        parameter.min_ = std::get<0>(boundaries)(parameter_counter);
        parameter.max_ = std::get<1>(boundaries)(parameter_counter);
    }
}

std::tuple<VectorXd,VectorXd> Example_SerialManipulatorEDH::get_parameter_space_boundaries() const
{
    VectorXd parameters_min(get_dim_parameter_space());
    VectorXd parameters_max(get_dim_parameter_space());
    for(int parameter_counter=0;parameter_counter<get_dim_parameter_space();parameter_counter++)
    {
        const Example_ParameterSpaceEDH::Example_Parameter& parameter = parameter_space_[parameter_counter];
        parameters_min(parameter_counter) = parameter.min_;
        parameters_max(parameter_counter) = parameter.max_;
    }

    return {parameters_min, parameters_max};
}

std::vector<Example_ParameterSpaceEDH::Example_ParameterType> Example_SerialManipulatorEDH::get_parameter_types() const
{
    std::vector<Example_ParameterSpaceEDH::Example_ParameterType> parameter_types;
    for(auto parameter : parameter_space_)
    {
        parameter_types.push_back(parameter.type_);
    }
    return parameter_types;
}


void Example_SerialManipulatorEDH::_check_link_index(const int &link_index) const
{
    if(link_index >= get_dim_configuration_space() || link_index < 0)
    {
        throw std::runtime_error(std::string("Tried to access link index ") + std::to_string(link_index) + std::string(" which is unnavailable."));
    }
}

void Example_SerialManipulatorEDH::_check_parameter_index(const int &parameter_index) const
{
    if(parameter_index >= get_dim_parameter_space() || parameter_index < 0)
    {
        throw std::runtime_error(std::string("Tried to access parameter_index ") + std::to_string(parameter_index) + std::string(" which is unnavailable."));
    }
}

void Example_SerialManipulatorEDH::_check_base_parameters(const std::vector<Example_ParameterSpaceEDH::Example_Parameter> &parameters)
{
    if(parameters.size()!=6)
        std::runtime_error("Incorrect number of parameters");

    if(parameters.at(0).type_ != Example_ParameterSpaceEDH::Example_ParameterType::base_x)
        std::runtime_error("First parameter should be of type base_x");
    if(parameters.at(1).type_ != Example_ParameterSpaceEDH::Example_ParameterType::base_y)
        std::runtime_error("Second parameter should be of type base_y");
    if(parameters.at(2).type_ != Example_ParameterSpaceEDH::Example_ParameterType::base_z)
        std::runtime_error("Third parameter should be of type base_z");
    if(parameters.at(3).type_ != Example_ParameterSpaceEDH::Example_ParameterType::base_alpha)
        std::runtime_error("Forth parameter should be of type base_alpha");
    if(parameters.at(4).type_ != Example_ParameterSpaceEDH::Example_ParameterType::base_beta)
        std::runtime_error("Fifth parameter should be of type base_beta");
    if(parameters.at(5).type_ != Example_ParameterSpaceEDH::Example_ParameterType::base_gamma)
        std::runtime_error("Sixth parameter should be of type base_gamma");
}

void Example_SerialManipulatorEDH::_check_eff_parameters(const std::vector<Example_ParameterSpaceEDH::Example_Parameter> &parameters)
{
    if(parameters.size()!=6)
        std::runtime_error("Incorrect number of parameters");

    if(parameters.at(0).type_ != Example_ParameterSpaceEDH::Example_ParameterType::eff_x)
        std::runtime_error("First parameter should be of type eff_x");
    if(parameters.at(1).type_ != Example_ParameterSpaceEDH::Example_ParameterType::eff_y)
        std::runtime_error("Second parameter should be of type eff_y");
    if(parameters.at(2).type_ != Example_ParameterSpaceEDH::Example_ParameterType::eff_z)
        std::runtime_error("Third parameter should be of type eff_z");
    if(parameters.at(3).type_ != Example_ParameterSpaceEDH::Example_ParameterType::eff_alpha)
        std::runtime_error("Forth parameter should be of type eff_alpha");
    if(parameters.at(4).type_ != Example_ParameterSpaceEDH::Example_ParameterType::eff_beta)
        std::runtime_error("Fifth parameter should be of type eff_beta");
    if(parameters.at(5).type_ != Example_ParameterSpaceEDH::Example_ParameterType::eff_gamma)
        std::runtime_error("Sixth parameter should be of type eff_gamma");
}

/**
 * @brief Example_SerialManipulatorEDH::_dh2dq
 *
 * At the time of first writing this class, it was the sandbox on many modifications in
 * DQ_SerialManipulatorDH, but now probably many of these changes became part of the main code.
 * This one might possibly be removed without loss.
 *
 * I'd recommend doing this after the next 23.04 (or later) release.
 *
 * @param joint_value
 * @param link_index
 * @return
 */
DQ Example_SerialManipulatorEDH::_dh2dq(const double &joint_value, const int &link_index) const
{
    double half_theta = get_theta(link_index)/2.0;
    double d = get_d(link_index);
    const double a = get_a(link_index);
    const double half_alpha = get_alpha(link_index)/2.0;
    const int joint_type = get_link_type(link_index);

    // Add the effect of the joint value
    switch(joint_type)
    {
    case JOINT_ROTATIONAL:
        half_theta = half_theta + (joint_value/2.0);
        break;
    case JOINT_PRISMATIC:
        d = d + joint_value;
        break;
    default:
        throw std::runtime_error("Unknown joint type");
    }

    // Pre-calculate cosines and sines
    const double sine_of_half_theta = sin(half_theta);
    const double cosine_of_half_theta = cos(half_theta);
    const double sine_of_half_alpha = sin(half_alpha);
    const double cosine_of_half_alpha = cos(half_alpha);

    // Return the optimized standard dh2dq calculation
    return DQ(
                cosine_of_half_alpha*cosine_of_half_theta,
                sine_of_half_alpha*cosine_of_half_theta,
                sine_of_half_alpha*sine_of_half_theta,
                cosine_of_half_alpha*sine_of_half_theta,
                -(a*sine_of_half_alpha*cosine_of_half_theta) /2.0 - (d*cosine_of_half_alpha*sine_of_half_theta)/2.0,
                (a*cosine_of_half_alpha*cosine_of_half_theta)/2.0 - (d*sine_of_half_alpha*sine_of_half_theta  )/2.0,
                (a*cosine_of_half_alpha*sine_of_half_theta)  /2.0 + (d*sine_of_half_alpha*cosine_of_half_theta)/2.0,
                (d*cosine_of_half_alpha*cosine_of_half_theta)/2.0 - (a*sine_of_half_alpha*sine_of_half_theta  )/2.0
                );
}

/**
 * @brief Example_SerialManipulatorEDH::_get_w
 *
 * At the time of first writing this class, it was the sandbox on many modifications in
 * DQ_SerialManipulatorDH, but now probably many of these changes became part of the main code.
 * This one might possibly be removed without loss.
 *
 * I'd recommend doing this after the next 23.04 (or later) release.
 *
 * @param link_index
 * @return
 */
DQ Example_SerialManipulatorEDH::_get_w(const int &link_index) const
{
    const int link_type = int(get_link_type(link_index));
    switch (link_type)
    {
    case JOINT_ROTATIONAL:
        return k_;
    case JOINT_PRISMATIC:
        return E_*k_;
    default:
        throw std::runtime_error("Unknown joint type");
    }
}

/**
 * @brief Example_SerialManipulatorEDH::_get_param_w An equivalent of _get_w for each parameter of the DH convention.
 * @param joint_value the current joint value.
 * @param parameter a Example_ParameterSpaceEDH::Example_Parameter containing all information of the parameter.
 * @return a DQ representing the parameter's w.
 */
DQ Example_SerialManipulatorEDH::_get_param_w(const double& joint_value, const Example_ParameterSpaceEDH::Example_Parameter &parameter) const
{
    double theta = get_theta(parameter.link_index_);
    double d = get_d(parameter.link_index_);

    // Add the effect of the joint value
    switch(int(get_link_type(parameter.link_index_)))
    {
    case JOINT_ROTATIONAL:
        theta = theta + joint_value;
        break;
    case JOINT_PRISMATIC:
        d = d + joint_value;
        break;
    default:
        throw std::runtime_error("Invalid joint type in _get_param_w");

    }

    // Get w depending on the parameters type
    switch(parameter.type_)
    {
    case Example_ParameterSpaceEDH::Example_ParameterType::theta:
        return k_;
    case Example_ParameterSpaceEDH::Example_ParameterType::d:
        return E_*k_;
    case Example_ParameterSpaceEDH::Example_ParameterType::a:
        return E_*(cos(theta)*i_ + sin(theta)*j_);
    case Example_ParameterSpaceEDH::Example_ParameterType::alpha:
        return (cos(theta)*i_ + sin(theta)*j_) + E_*(-d*sin(theta)*i_ + d*cos(theta)*j_);
    default:
        throw std::runtime_error("Invalid parameter type in _get_param_w");
    }
}

/**
 * @brief Example_SerialManipulatorEDH::_get_base_param_w An equivalent of _get_w for each parameter of the base transformation.
 * @param parameter_type
 * @return
 */
DQ Example_SerialManipulatorEDH::_get_base_param_w(const Example_ParameterSpaceEDH::Example_ParameterType &parameter_type) const
{
    const double& x = base_parameters_.at(0).value_;
    const double& y = base_parameters_.at(1).value_;
    const double& z = base_parameters_.at(2).value_;
    const double& alpha = base_parameters_.at(3).value_;
    const double& beta = base_parameters_.at(4).value_;

    switch(parameter_type)
    {
    case Example_ParameterSpaceEDH::Example_ParameterType::base_x:
        return E_*i_;
    case Example_ParameterSpaceEDH::Example_ParameterType::base_y:
        return E_*j_;
    case Example_ParameterSpaceEDH::Example_ParameterType::base_z:
        return E_*k_;
    case Example_ParameterSpaceEDH::Example_ParameterType::base_alpha:
        return i_ + E_*( z*j_ - y*k_ );
    case Example_ParameterSpaceEDH::Example_ParameterType::base_beta:
        return  j_*cos(alpha) +
                k_*sin(alpha) +
                E_*i_*(y*sin(alpha) - z*cos(alpha)) +
                E_*j_*(-x*sin(alpha)) +
                E_*k_*(x*cos(alpha));
    case Example_ParameterSpaceEDH::Example_ParameterType::base_gamma:
        return  i_*sin(beta) +
                j_*(-cos(beta)*sin(alpha)) +
                k_*(cos(alpha)*cos(beta)) +
                E_*i_*(cos(beta)*(y*cos(alpha) + z*sin(alpha))) +
                E_*j_*(z*sin(beta) - x*cos(alpha)*cos(beta)) +
                E_*k_*(-y*sin(beta) - x*cos(beta)*sin(alpha));
    default:
        throw std::runtime_error("Invalid parameter type in _get_base_param_w");
    }
}

/**
 * @brief Example_SerialManipulatorEDH::_get_eff_param_w An equivalent of _get_w for each parameter of the effector transformation.
 *
 * This implementation is slightly faster than _get_base_param_w, but I'm leaving both to illustrate two possible implementations.
 * Note that _get_base_param_w and _get_eff_param_w should return precisely the same values, as they do the same calculation.
 *
 * @param parameter_type
 * @return
 */
DQ Example_SerialManipulatorEDH::_get_eff_param_w(const Example_ParameterSpaceEDH::Example_ParameterType &parameter_type) const
{
    const double& x = eff_parameters_.at(0).value_;
    const double& y = eff_parameters_.at(1).value_;
    const double& z = eff_parameters_.at(2).value_;
    const double& alpha = eff_parameters_.at(3).value_;
    const double& beta = eff_parameters_.at(4).value_;

    switch(parameter_type)
    {
    case Example_ParameterSpaceEDH::Example_ParameterType::eff_x:
        return E_*i_;
    case Example_ParameterSpaceEDH::Example_ParameterType::eff_y:
        return E_*j_;
    case Example_ParameterSpaceEDH::Example_ParameterType::eff_z:
        return E_*k_;
    case Example_ParameterSpaceEDH::Example_ParameterType::eff_alpha:
        return DQ( 0., 1., 0., 0., 0., 0., z, -y);
    case Example_ParameterSpaceEDH::Example_ParameterType::eff_beta:
        return DQ( 0., 0., cos(alpha), sin(alpha), 0., y*sin(alpha) - z*cos(alpha), -x*sin(alpha), x*cos(alpha));
    case Example_ParameterSpaceEDH::Example_ParameterType::eff_gamma:
        return DQ( 0.,
                   sin(beta),
                   -cos(beta)*sin(alpha),
                   cos(alpha)*cos(beta),
                   0,
                   cos(beta)*(y*cos(alpha) + z*sin(alpha)),
                   z*sin(beta) - x*cos(alpha)*cos(beta),
                   - y*sin(beta) - x*cos(beta)*sin(alpha));
    default:
        throw std::runtime_error("Invalid parameter type in _get_eff_param_w");
    }
}

/**
 * @brief Example_SerialManipulatorEDH::raw_fkm
 *
 * At the time of first writing this class, it was the sandbox on many modifications in
 * DQ_SerialManipulatorDH, but now probably many of these changes became part of the main code.
 * This one might possibly be removed without loss.
 *
 * I'd recommend doing this after the next 23.04 (or later) release.
 *
 * @param joint_values
 * @param to_ith_link
 * @return
 */
DQ Example_SerialManipulatorEDH::raw_fkm(const VectorXd &joint_values, const int &to_ith_link) const
{
    _check_link_index(to_ith_link);

    DQ x(1);
    for (int link_index = 0; link_index <= to_ith_link; link_index++)
    {
        x = x * _dh2dq(joint_values(link_index), link_index);
    }
    return x;
}

/**
 * @brief Example_SerialManipulatorEDH::fkm
 *
 * At the time of first writing this class, it was the sandbox on many modifications in
 * DQ_SerialManipulatorDH, but now probably many of these changes became part of the main code.
 * This one might possibly be removed without loss.
 *
 * I'd recommend doing this after the next 23.04 (or later) release.
 *
 * @param joint_values
 * @param to_ith_link
 * @return
 */
DQ Example_SerialManipulatorEDH::fkm(const VectorXd& joint_values, const int& to_ith_link) const
{
    _check_q_vec(joint_values);
    _check_link_index(to_ith_link);

    DQ x = get_base_frame() * ( raw_fkm(joint_values, to_ith_link) ); //Take the base into account

    if(to_ith_link == get_dim_configuration_space() - 1)
        x = x * get_effector_frame(); //Take into account the end effector

    return x;
}

/**
 * @brief Example_SerialManipulatorEDH::fkm
 *
 * At the time of first writing this class, it was the sandbox on many modifications in
 * DQ_SerialManipulatorDH, but now probably many of these changes became part of the main code.
 * This one might possibly be removed without loss.
 *
 * I'd recommend doing this after the next 23.04 (or later) release.
 *
 * @param joint_values
 * @return
 */
DQ Example_SerialManipulatorEDH::fkm(const VectorXd &joint_values) const
{
    return fkm(joint_values, get_dim_configuration_space()-1);
}

/**
 * @brief Example_SerialManipulatorEDH::pose_jacobian
 *
 * At the time of first writing this class, it was the sandbox on many modifications in
 * DQ_SerialManipulatorDH, but now probably many of these changes became part of the main code.
 * This one might possibly be removed without loss.
 *
 * I'd recommend doing this after the next 23.04 (or later) release.
 *
 * @param joint_values
 * @param to_ith_link
 * @return
 */
MatrixXd Example_SerialManipulatorEDH::pose_jacobian(const VectorXd &joint_values, const int &to_ith_link) const
{
    _check_q_vec(joint_values);
    _check_link_index(to_ith_link);

    MatrixXd J = raw_pose_jacobian(joint_values,to_ith_link);

    if(to_ith_link==get_dim_configuration_space()-1)
    {
        J = hamiplus8(get_base_frame())*haminus8(get_effector_frame())*J;
    }
    else
    {
        J = hamiplus8(get_base_frame())*J;
    }

    return J;
}

/**
 * @brief Example_SerialManipulatorEDH::pose_jacobian
 *
 * At the time of first writing this class, it was the sandbox on many modifications in
 * DQ_SerialManipulatorDH, but now probably many of these changes became part of the main code.
 * This one might possibly be removed without loss.
 *
 * I'd recommend doing this after the next 23.04 (or later) release.
 *
 * @param joint_values
 * @return
 */
MatrixXd Example_SerialManipulatorEDH::pose_jacobian(const VectorXd &joint_values) const
{
    return pose_jacobian(joint_values, get_dim_configuration_space()-1);
}


/**
 * @brief Example_SerialManipulatorEDH::raw_pose_jacobian
 *
 * At the time of first writing this class, it was the sandbox on many modifications in
 * DQ_SerialManipulatorDH, but now probably many of these changes became part of the main code.
 * This one might possibly be removed without loss.
 *
 * I'd recommend doing this after the next 23.04 (or later) release.
 *
 * @param joint_values
 * @param to_ith_link
 * @return
 */
MatrixXd Example_SerialManipulatorEDH::raw_pose_jacobian(const VectorXd &joint_values, const int &to_ith_link) const
{
    _check_q_vec(joint_values);

    MatrixXd J = MatrixXd::Zero(8,to_ith_link+1);
    const DQ x_effector = raw_fkm(joint_values,to_ith_link);

    DQ x(1);

    for(int i=0;i<=to_ith_link;i++)
    {
        DQ w = _get_w(i);
        DQ z = 0.5*Ad(x,w);
        x = x*_dh2dq(joint_values(i),i);
        J.col(i) = vec8(z * x_effector);
    }
    return J;
}

/**
 * @brief Example_SerialManipulatorEDH::_parameter_pose_jacobian_col obtains the column of the pose Jacobian obtained from a parameter.
 *
 * The parameter_pose_jacobian, differently from the pose_jacobian related to the joint values, does not have an obvious order for the columns.
 * In addition, depending on the robot we might want to adapt more or less parameters, so this implementation helps making it highly
 * customizable.
 *
 * @param joint_values the vector of joint values.
 * @param parameter_index the index of the parameters.
 * @param to_ith_link up to which link of the robot should the parameter Jacobian column be calculated.
 * @return the parameter pose Jacobian column.
 */
VectorXd Example_SerialManipulatorEDH::_parameter_pose_jacobian_col(const VectorXd& joint_values, const int& parameter_index, const int& to_ith_link) const
{
    _check_parameter_index(parameter_index);

    const Example_ParameterSpaceEDH::Example_Parameter& parameter = _get_parameter(parameter_index);

    DQ x_effector = fkm(joint_values, to_ith_link);
    DQ x;
    DQ w;
    if(parameter.link_index_ == -1)
    {
        x = DQ(1);
        w = _get_base_param_w(parameter.type_);
    }
    else if(parameter.link_index_ == 0)
    {
        x = get_base_frame();
        w = _get_param_w(joint_values(parameter.link_index_), parameter);
    }
    else if(parameter.link_index_ == get_dim_configuration_space())
    {
        x = get_base_frame()*raw_fkm(joint_values, get_dim_configuration_space() - 1);
        w = _get_eff_param_w(parameter.type_);
    }
    else
    {
        x = get_base_frame()*raw_fkm(joint_values, parameter.link_index_ - 1);
        w = _get_param_w(joint_values(parameter.link_index_), parameter);
    }
    DQ z  = 0.5 * Ad(x,w);
    DQ jp = z * x_effector;

    return vec8(jp);
}

/**
 * @brief Example_SerialManipulatorEDH::parameter_pose_jacobian obtains the the pose Jacobian obtained from the parameters.
 *
 * @see _parameter_pose_jacobian_col.
 *
 * @param joint_values
 * @param to_ith_link
 * @return
 */
MatrixXd Example_SerialManipulatorEDH::parameter_pose_jacobian(const VectorXd &joint_values, const int &to_ith_link) const
{
    _check_q_vec(joint_values);
    _check_link_index(to_ith_link);

    const int dim_parameter_space = get_dim_parameter_space();

    MatrixXd Jp = MatrixXd::Zero(8,dim_parameter_space);
    for(int parameter_index = 0; parameter_index < dim_parameter_space; parameter_index++)
    {
        Jp.col(parameter_index)=_parameter_pose_jacobian_col(joint_values,parameter_index,to_ith_link);
    }
    return Jp;
}

MatrixXd Example_SerialManipulatorEDH::parameter_pose_jacobian(const VectorXd& joint_values) const
{
    return parameter_pose_jacobian(joint_values, get_dim_configuration_space()-1);
}

VectorXd Example_SerialManipulatorEDH::get_thetas() const
{
    return dh_matrix_.row(0);
}

double Example_SerialManipulatorEDH::get_theta(const int &link_index) const
{
    _check_link_index(link_index);
    return dh_matrix_(0,link_index);
}

void Example_SerialManipulatorEDH::set_theta(const int &link_index, const double &value)
{
    _check_link_index(link_index);
    dh_matrix_(0,link_index) = value;
}

VectorXd Example_SerialManipulatorEDH::get_ds() const
{
    return dh_matrix_.row(1);
}

double Example_SerialManipulatorEDH::get_d(const int &link_index) const
{
    _check_link_index(link_index);
    return dh_matrix_(1,link_index);
}

void Example_SerialManipulatorEDH::set_d(const int &link_index, const double &value)
{
    _check_link_index(link_index);
    dh_matrix_(1,link_index) = value;
}

VectorXd Example_SerialManipulatorEDH::get_as() const
{
    return dh_matrix_.row(2);
}

double Example_SerialManipulatorEDH::get_a(const int &link_index) const
{
    _check_link_index(link_index);
    return dh_matrix_(2,link_index);
}

void Example_SerialManipulatorEDH::set_a(const int &link_index, const double &value)
{
    _check_link_index(link_index);
    dh_matrix_(2,link_index) = value;
}

VectorXd Example_SerialManipulatorEDH::get_alphas() const
{
    return dh_matrix_.row(3);
}

double Example_SerialManipulatorEDH::get_alpha(const int &link_index) const
{
    _check_link_index(link_index);
    return dh_matrix_(3,link_index);
}

void Example_SerialManipulatorEDH::set_alpha(const int &link_index, const double &value)
{
    _check_link_index(link_index);
    dh_matrix_(3,link_index) = value;
}

VectorXd Example_SerialManipulatorEDH::get_link_types() const
{
    return dh_matrix_.row(4);
}

double Example_SerialManipulatorEDH::get_link_type(const int &link_index) const
{
    _check_link_index(link_index);
    return dh_matrix_(4,link_index);
}

int Example_SerialManipulatorEDH::get_dim_configuration_space() const
{
    return dh_matrix_.cols();
}

MatrixXd Example_SerialManipulatorEDH::pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot, const int &to_ith_link) const
{
    throw std::runtime_error("NOT IMPLEMENTED");
}

MatrixXd Example_SerialManipulatorEDH::raw_pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot, const int &to_ith_link) const
{
    throw std::runtime_error("NOT IMPLEMENTED");
}



