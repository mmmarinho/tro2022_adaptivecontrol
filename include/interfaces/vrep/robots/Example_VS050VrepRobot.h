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

#include <vector>

#include <dqrobotics/interfaces/vrep/DQ_SerialVrepRobot.h>

#include "robot_modeling/Example_SerialManipulatorEDH.h"

using namespace DQ_robotics;

class Example_VS050VrepRobot: public DQ_SerialVrepRobot
{
public:
    Example_VS050VrepRobot(const std::string& robot_name,
                 const std::shared_ptr<DQ_VrepInterface>& vrep_interface_sptr);

    static Example_SerialManipulatorEDH raw_kinematics();
    DQ get_base_frame();
    void set_base_frame(const DQ& base_frame, const std::string& reference_frame_name=VREP_OBJECTNAME_ABSOLUTE);
};



