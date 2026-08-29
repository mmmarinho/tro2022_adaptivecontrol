#pragma once
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

#include <string>
#include <unordered_map>
#include <vector>

#include <dqrobotics/DQ.h>
#include <dqrobotics/utils/DQ_Geometry.h>
#include <dqrobotics/utils/DQ_Math.h>

using namespace DQ_robotics;

/**
 * @brief M3_Simulator is the base class of a robot-scene simulator.
 *
 * It stores the state of a named set of scene objects (each given by a DQ
 * pose) plus the robot configuration (joint-space), and tracks whether the
 * simulation has been started.
 *
 * The interface is intentionally minimal and backend-agnostic: a concrete
 * backend provides the scene data and may override the virtual @ref draw_scene
 * for visualization. The headless, in-memory backend is M3_SimulatorDummy; a
 * visualization backend (based on dqrobotics_extensions.pyplot) is provided on
 * the Python side (M3_PyPlotSimulator).
 *
 * This class is exposed to Python together with a pybind11 trampoline so that
 * it can be subclassed from Python (a Python subclass overrides draw_scene).
 * It deliberately has no pybind11/Python-API dependency so that the pure C++
 * (headless) build stays free of any Python runtime.
 */
class M3_Simulator
{
    //Object poses, keyed by object name
    std::unordered_map<std::string, DQ> object_poses_;

    //Robot configuration (joint-space)
    VectorXd q_ = VectorXd::Zero(6);

    //Simulation state (the loop always runs in real time)
    bool running_ = false;

public:
    M3_Simulator() = default;

    // A polymorphic base class needs a virtual destructor so that a
    // std::shared_ptr<M3_Simulator> can safely destroy a derived object.
    virtual ~M3_Simulator() = default;

    /**
     * @brief draw_scene Draws the scene (robots, obstacles, targets). This is
     * the visualization hook that a backend (e.g. the Python-side
     * M3_PyPlotSimulator) is expected to override; it is exposed through a
     * pybind11 trampoline so a Python subclass can override it. The default
     * (headless) implementation does nothing.
     *
     * The method takes no arguments on purpose, so this C++ base class stays
     * free of any Python dependency (the pure headless build links no Python
     * runtime). A visualization backend renders onto the current plot Axes
     * (or a stored one), matching the dqrobotics_extensions.pyplot convention
     * of defaulting to matplotlib's current axes.
     */
    virtual void draw_scene();

    //*************** Object poses (the "scene") ***************

    /**
     * @brief get_object_pose Returns the pose of @p object_name.
     * @param object_name the name of the object.
     * @throws std::runtime_error if the object does not exist in the scene.
     */
    DQ get_object_pose(const std::string& object_name) const;

    /**
     * @brief get_object_pose_or_default Returns the pose of @p object_name, or
     * @p default_pose if the object does not exist.
     */
    DQ get_object_pose_or_default(const std::string& object_name, const DQ& default_pose) const;

    /**
     * @brief set_object_pose Sets the pose of @p object_name (creating it if new).
     */
    void set_object_pose(const std::string& object_name, const DQ& pose);

    /// Whether an object with @p object_name exists in the scene.
    bool has_object(const std::string& object_name) const;

    /// The names of all objects currently in the scene (order not guaranteed).
    std::vector<std::string> get_object_names() const;

    //*************** Robot configuration ***************

    /// The current robot configuration (joint-space).
    const VectorXd& get_configuration_space_positions() const;

    /// Sets the robot configuration (joint-space).
    void set_configuration_space_positions(const VectorXd& q);

    //*************** Simulation control ***************

    /// Marks the simulation as started (no real time-stepping is performed).
    void start_simulation();

    /// Marks the simulation as stopped.
    void stop_simulation();

    /// Whether the simulation is currently started.
    bool is_running() const;
};
