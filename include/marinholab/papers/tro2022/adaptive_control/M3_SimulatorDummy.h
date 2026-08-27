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

#include "marinholab/papers/tro2022/adaptive_control/M3_SerialManipulatorEDH.h"

using namespace DQ_robotics;

/**
 * @brief M3_SimulatorDummy is an in-memory stand-in for a real robot simulator
 * that stores named object poses and robot configurations.
 *
 * It exists so that the adaptive control example can be built and run
 * **headless** (no simulator, no network) for dry testing.
 *
 * The interface is intentionally minimal:
 *  - named object poses (DQ), the same concept the simulator exposed
 *    (e.g. "xd0", "xd1", "tool_sphere_1", "cube_40x40_wall_1", "x_hat");
 *  - robot configuration (joint-space) storage;
 *  - simulation start/stop bookkeeping (the loop runs in real time).
 *
 * It also ships a **nominal TRO2022 reference scene** (see load_reference_scene).
 * Note: this is a *reconstruction* of the original reference scene for dry
 * testing. The box, targets, and initial configuration are approximate, and
 * the joint limits may be adjusted per experiment.
 *
 * A future real-simulator backend can implement the same interface;
 * M3_SimulatorDummy itself is that backend for the headless case.
 */
class M3_SimulatorDummy
{
    //Object poses, keyed by object name (as used in the original scene)
    std::unordered_map<std::string, DQ> object_poses_;

    //Robot configuration (joint-space)
    VectorXd q_ = VectorXd::Zero(6);

    //Simulation state (the loop always runs in real time)
    bool running_ = false;

public:
    /// Default constructor: empty scene, zero configuration.
    M3_SimulatorDummy() = default;

    //*************** Object poses (the "scene") ***************

    /**
     * @brief get_object_pose Returns the pose of @p object_name.
     * @param object_name the name of the object (as in the original scene).
     * @throws std::runtime_error if the object does not exist in the scene.
     */
    DQ get_object_pose(const std::string& object_name) const;

    /**
     * @brief set_object_pose Sets the pose of @p object_name (creating it if new).
     * @param object_name the name of the object.
     * @param pose the pose to set.
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

    //*************** Scene presets (dry testing) ***************

    /**
     * @brief load_reference_scene Loads the nominal TRO2022 reference scene.
     *
     * This reconstructs, in memory, the scene of the TRO2022 paper's example:
     * a 6-DoF VS050-style robot (ideal base/effector identity), a 40 cm cube
     * workspace with 4 walls and 2 tubes, and the two task targets xd0 (safe
     * approach) and xd1 (unreachable target).
     *
     * Note: this is a *reconstruction* of the original reference scene for
     * dry testing. The box, targets, and initial configuration are
     * approximate and may be adjusted per experiment.
     *
     * It also sets the initial robot configuration to a plausible pose and
     * reconstructs the end-effector pose reference "x_hat" and the tool
     * reference spheres (tool_sphere_1..6) from the VS050 kinematics at that
     * configuration (in the original scene they were physical objects
     * attached to the robot).
     */
    void load_reference_scene();

    /**
     * @brief vs050_raw_kinematics Returns the kinematics of the VS050 robot of
     * the TRO2022 example (modified Denavit-Hartmann parameters and joint
     * limits), with identity base and effector frames.
     */
    static M3_SerialManipulatorEDH vs050_raw_kinematics();
};
