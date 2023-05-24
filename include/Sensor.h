#pragma once
/**
(C) Copyright 2029-2023 Murilo Marques Marinho (www.murilomarinho.info)

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
#include <dqrobotics/DQ.h>

#include <robot_control/DQ_MeasurementSpace.h>

using namespace DQ_robotics;

class Sensor
{
protected:
    DQ_MeasureSpace measure_space_;
    bool initialized_;
public:
    Sensor()=delete;
    Sensor(const DQ_MeasureSpace& measure_space);

    virtual VectorXd get_measurement_space_vector()=0;
    virtual DQ get_measurement()=0;
    virtual void initialize()=0;

    virtual bool is_initialized() const;
};
