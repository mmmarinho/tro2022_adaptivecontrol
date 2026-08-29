/**
(C) Copyright 2020-2026 Murilo Marques Marinho (www.murilomarinho.info)

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

#include "marinholab/papers/tro2022/adaptive_control/M3_Simulator.h"

#include <stdexcept>

void M3_Simulator::draw_scene()
{
    // Headless default: nothing to draw. Backends (e.g. the Python-side
    // M3_PyPlotSimulator) override this to render the scene.
}

DQ M3_Simulator::get_object_pose(const std::string& object_name) const
{
    const auto it = object_poses_.find(object_name);
    if(it == object_poses_.end())
        throw std::runtime_error("M3_Simulator: object \"" + object_name + "\" not found in the scene.");
    return it->second;
}

DQ M3_Simulator::get_object_pose_or_default(const std::string& object_name, const DQ& default_pose) const
{
    const auto it = object_poses_.find(object_name);
    if(it == object_poses_.end())
        return default_pose;
    return it->second;
}

void M3_Simulator::set_object_pose(const std::string& object_name, const DQ& pose)
{
    object_poses_[object_name] = pose;
}

bool M3_Simulator::has_object(const std::string& object_name) const
{
    return object_poses_.find(object_name) != object_poses_.end();
}

std::vector<std::string> M3_Simulator::get_object_names() const
{
    std::vector<std::string> names;
    names.reserve(object_poses_.size());
    for(const auto& kv : object_poses_)
        names.push_back(kv.first);
    return names;
}

const VectorXd& M3_Simulator::get_configuration_space_positions() const
{
    return q_;
}

void M3_Simulator::set_configuration_space_positions(const VectorXd& q)
{
    q_ = q;
}

void M3_Simulator::start_simulation()
{
    running_ = true;
}

void M3_Simulator::stop_simulation()
{
    running_ = false;
}

bool M3_Simulator::is_running() const
{
    return running_;
}
