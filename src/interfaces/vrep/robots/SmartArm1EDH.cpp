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
#include<dqrobotics/utils/DQ_Constants.h>
#include<interfaces/vrep/robots/SmartArm1EDH.h>
#include<dqrobotics/utils/DQ_Math.h>

namespace DQ_robotics
{

SmartArm1EDH::SmartArm1EDH(const std::string& robot_name, const std::shared_ptr<DQ_VrepInterface>& vrep_interface_sptr):
    DQ_SerialVrepRobot("VS050",6,robot_name,vrep_interface_sptr)
{
    base_frame_name_ = "VS050_reference_frame";
}


DQ_SerialManipulatorEDH SmartArm1EDH::raw_kinematics()
{
    MatrixXd VS050_dh_matrix(5,6);

    VS050_dh_matrix <<
            -pi,  pi/2.,     -pi/2.,      0.,       pi,    0.,
          0.345,     0.,         0.,   0.255,       0.,    0.07,
             0.,  0.250,       0.01,      0.,       0.,    0,
          pi/2.,     0.,     -pi/2.,   pi/2.,    pi/2.,    0,
             0.,     0.,         0.,      0.,       0.,    0;

    VectorXd lower_joint_limits(6); lower_joint_limits << -170.,-100.,-60.,-265.,-119.,-355.; lower_joint_limits = deg2rad(lower_joint_limits);
    VectorXd upper_joint_limits(6); upper_joint_limits <<  170., 100.,124., 265., 89.9, 355.; upper_joint_limits = deg2rad(upper_joint_limits);

    DQ_SerialManipulatorEDH dq_serialmanipulator_edh(VS050_dh_matrix);
    dq_serialmanipulator_edh.set_lower_q_limit(lower_joint_limits);
    dq_serialmanipulator_edh.set_upper_q_limit(upper_joint_limits);
    return dq_serialmanipulator_edh;
}

DQ SmartArm1EDH::get_base_frame()
{
    return _get_interface_sptr()->get_object_pose(base_frame_name_);
}

void SmartArm1EDH::set_base_frame(const DQ& base_frame, const std::string& reference_frame_name)
{
    _get_interface_sptr()->set_object_pose(base_frame_name_,base_frame, reference_frame_name);
}

}
