#pragma once

/**
(C) Copyright 2020 DQ Robotics Developers
This file is part of DQ Robotics.
    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.
Contributors:
- Murilo M. Marinho        (murilo@nml.t.u-tokyo.ac.jp)
*/

#include <vector>
#include <dqrobotics/interfaces/vrep/DQ_SerialVrepRobot.h>
#include <robot_modeling/DQ_SerialManipulatorEDH.h>

namespace DQ_robotics
{
class SmartArm1EDH: public DQ_SerialVrepRobot
{
public:
    SmartArm1EDH(const std::string& robot_name,
                              const std::shared_ptr<DQ_VrepInterface>& vrep_interface_sptr);

    static DQ_SerialManipulatorEDH raw_kinematics();
    DQ get_base_frame_from_vrep();
    void send_base_frame_to_vrep(const DQ& base_frame, const std::string& reference_frame=VREP_OBJECTNAME_ABSOLUTE);
};
}


