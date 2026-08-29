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

/**
 * Example code for:
 * M. M. Marinho and B. V. Adorno,
 * "Adaptive Constrained Kinematic Control Using Partial or Complete Task-Space Measurements,"
 * in IEEE Transactions on Robotics, vol. 38, no. 6, pp. 3498-3513, Dec. 2022,
 * doi: 10.1109/TRO.2022.3181047.
 */

#include "marinholab/papers/tro2022/adaptive_control/M3_SimulatorDummy.h"

#include <cmath>
#include <stdexcept>

void M3_SimulatorDummy::load_reference_scene()
{
    // The poses below are built with the dual-quaternion algebra written
    // explicitly (no helper functions), so the construction is visible in place:
    //   * a rotation of angle phi about the unit vector v is the quaternion
    //         r = cos(phi/2) + v*sin(phi/2)
    //   * a translation t is written in the DQ basis {i_, j_, k_}
    //         t = tx*i_ + ty*j_ + tz*k_
    //   * a pose (unit dual quaternion) combines the two as
    //         x = r + 0.5*E_*t*r
    // (E_ is the dual unit; for a unit DQ, translation(x) == t.)
    const DQ i_ = DQ::i;
    const DQ j_ = DQ::j;
    const DQ k_ = DQ::k;

    // Half-size of the 40 cm box workspace (in x and y; open in z).
    const double h = 0.2;

    // Robot base frame (world frame). The ideal robot uses the identity base:
    // r = 1 (no rotation), t = 0 (no translation)  ->  x = 1.
    const DQ identity = DQ(1.);
    set_object_pose("VS050_reference_frame", identity);

    // Four vertical walls of the box. Each wall object's pose encodes the plane:
    //   normal = the object's k-axis,  plane passes through the object's position.
    // The normals point inward so that the FORBIDDEN_ZONE VFIs keep the robot
    // inside the box. A wall's orientation is the rotation r that turns its k-axis
    // into the desired inward normal; t is where the plane passes through.

    // Wall at x=+h, inward normal -i: rotate k -> -i, i.e. -90 deg about j.
    const DQ r_wall1 = std::cos(-pi / 4.0) + j_ * std::sin(-pi / 4.0);
    const DQ t_wall1 = h * i_;
    set_object_pose("cube_40x40_wall_1", r_wall1 + 0.5 * E_ * t_wall1 * r_wall1);

    // Wall at x=-h, inward normal +i: rotate k -> +i, i.e. +90 deg about j.
    const DQ r_wall2 = std::cos(pi / 4.0) + j_ * std::sin(pi / 4.0);
    const DQ t_wall2 = -h * i_;
    set_object_pose("cube_40x40_wall_2", r_wall2 + 0.5 * E_ * t_wall2 * r_wall2);

    // Wall at y=+h, inward normal -j: rotate k -> -j, i.e. +90 deg about i.
    const DQ r_wall3 = std::cos(pi / 4.0) + i_ * std::sin(pi / 4.0);
    const DQ t_wall3 = h * j_;
    set_object_pose("cube_40x40_wall_3", r_wall3 + 0.5 * E_ * t_wall3 * r_wall3);

    // Wall at y=-h, inward normal +j: rotate k -> +j, i.e. -90 deg about i.
    const DQ r_wall4 = std::cos(-pi / 4.0) + i_ * std::sin(-pi / 4.0);
    const DQ t_wall4 = -h * j_;
    set_object_pose("cube_40x40_wall_4", r_wall4 + 0.5 * E_ * t_wall4 * r_wall4);

    // Two vertical tubes (lines) along z, inside the box. The tube object's
    // k-axis is the tube axis (no rotation, r = 1); the line passes through t.
    const DQ t_tube1 = 0.12 * j_;
    set_object_pose("cube_40x40_tube_1", identity + 0.5 * E_ * t_tube1 * identity);
    const DQ t_tube2 = -0.12 * j_;
    set_object_pose("cube_40x40_tube_2", identity + 0.5 * E_ * t_tube2 * identity);

    // Task targets.
    // xd0: a safe approach pose inside the box, in front of the opening (r = 1, t = 0).
    set_object_pose("xd0", identity);
    // xd1: the final target, chosen (as in the original example) as a pose the
    // robot cannot reach; it serves to show that even then the robot does not
    // collide with the environment.
    const DQ t_xd1 = 0.55 * k_;
    set_object_pose("xd1", identity + 0.5 * E_ * t_xd1 * identity);

    // A plausible initial robot configuration. It is a folded posture that keeps
    // every link/tool point inside the nominal box workspace, with enough
    // clearance that the example's parameter randomization (step [5]) can find a
    // non-penetrating "wrong but plausible" estimate. (This reconstruction stands
    // in for the initial configuration read from the live simulator.)
    VectorXd q_init(6);
    q_init << 2.32698, -0.427311, 0.681903, 1.4088, -0.759787, 0.879927;
    set_configuration_space_positions(q_init);

    // End-effector pose reference "x_hat" and the tool reference spheres
    // (tool_sphere_1..6). In the original scene these were physical objects
    // attached to the robot; here they are reconstructed as the six link-frame
    // poses of the VS050 at q_init. The N-th sphere sits on the N-th link
    // (DQ link index N-1, so the last sphere coincides with the end effector).
    // Their pose relative to "x_hat" (the end-effector) is what the VFI uses as
    // the protected point on the arm.
    const M3_SerialManipulatorEDH robot = vs050_raw_kinematics();
    const DQ x_hat_ee = robot.fkm(q_init);
    set_object_pose("x_hat", x_hat_ee);
    for(int link_index = 0; link_index <= 5; link_index++)
    {
        const DQ link_pose = robot.fkm(q_init, link_index);
        set_object_pose("tool_sphere_" + std::to_string(link_index + 1), link_pose);
    }
}

M3_SerialManipulatorEDH M3_SimulatorDummy::vs050_raw_kinematics()
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

    M3_SerialManipulatorEDH robot(VS050_dh_matrix);
    robot.set_lower_q_limit(lower_joint_limits);
    robot.set_upper_q_limit(upper_joint_limits);
    robot.set_base_frame(DQ(1.));
    robot.set_effector_frame(DQ(1.));
    return robot;
}
