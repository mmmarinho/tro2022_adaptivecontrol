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
#include "marinholab/papers/tro2022/adaptive_control/M3_Simulator.h"

using namespace DQ_robotics;

/**
 * @brief M3_SimulatorDummy is an in-memory stand-in for a real robot simulator
 * that stores named object poses and robot configurations.
 *
 * It inherits from M3_Simulator (which owns the scene-object store, the
 * configuration and the start/stop bookkeeping) so that the example can be
 * built and run **headless** (no simulator, no network) for dry testing.
 *
 * It also ships a **nominal TRO2022 reference scene** (see load_reference_scene).
 * Note: this is a *reconstruction* of the original reference scene for dry
 * testing. The box, targets, and initial configuration are approximate, and
 * the joint limits may be adjusted per experiment.
 *
 * It is a headless backend of M3_Simulator: it does not override draw_scene,
 * so the (default) headless behavior (nothing drawn) applies. A visualization
 * backend instead loads a scene (e.g. from a .yaml file) and overrides
 * draw_scene, e.g. the Python-side M3_PyPlotSimulator.
 */
class M3_SimulatorDummy : public M3_Simulator
{
public:
    /// Default constructor: empty scene, zero configuration.
    M3_SimulatorDummy() = default;

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
