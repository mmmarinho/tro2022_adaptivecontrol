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

#include <stdexcept>

namespace
{
// Builds a unit dual quaternion for a pose from a unit rotation quaternion @p P
// and a translation vector @p t. This mirrors DQ::unitDQ:
//     h = P + E_*(0.5 * p * P),   where p = (0, tx, ty, tz) is the position quaternion.
// (For a unit DQ, translation(h) == t by construction.)
DQ make_pose(const DQ& P, const Vector3d& t)
{
    const DQ p(0., t(0), t(1), t(2)); // position quaternion (0,tx,ty,tz), dual part 0
    return P + E_*(0.5 * p * P);
}

// Rotation quaternion for an angle (rad) about a unit axis.
DQ make_rotation(const Vector3d& axis, const double& angle)
{
    const double s = std::sin(angle / 2.0);
    return DQ(std::cos(angle / 2.0), s * axis(0), s * axis(1), s * axis(2), 0., 0., 0., 0.);
}
}

DQ M3_SimulatorDummy::get_object_pose(const std::string& object_name) const
{
    const auto it = object_poses_.find(object_name);
    if(it == object_poses_.end())
        throw std::runtime_error("M3_SimulatorDummy: object \"" + object_name + "\" not found in the scene.");
    return it->second;
}

void M3_SimulatorDummy::set_object_pose(const std::string& object_name, const DQ& pose)
{
    object_poses_[object_name] = pose;
}

bool M3_SimulatorDummy::has_object(const std::string& object_name) const
{
    return object_poses_.find(object_name) != object_poses_.end();
}

std::vector<std::string> M3_SimulatorDummy::get_object_names() const
{
    std::vector<std::string> names;
    names.reserve(object_poses_.size());
    for(const auto& kv : object_poses_)
        names.push_back(kv.first);
    return names;
}

const VectorXd& M3_SimulatorDummy::get_configuration_space_positions() const
{
    return q_;
}

void M3_SimulatorDummy::set_configuration_space_positions(const VectorXd& q)
{
    q_ = q;
}

void M3_SimulatorDummy::start_simulation()
{
    running_ = true;
}

void M3_SimulatorDummy::stop_simulation()
{
    running_ = false;
}

bool M3_SimulatorDummy::is_running() const
{
    return running_;
}

void M3_SimulatorDummy::load_reference_scene()
{
    const DQ identity(1., 0., 0., 0., 0., 0., 0., 0.);
    const Vector3d x_axis(1., 0., 0.);
    const Vector3d y_axis(0., 1., 0.);

    // Half-size of the 40 cm box workspace (in x and y; open in z).
    const double h = 0.2;

    // Robot base frame (world frame). The ideal robot uses the identity base.
    set_object_pose("VS050_reference_frame", identity);

    // Four vertical walls of the box. Each wall object's pose encodes the plane:
    //   normal = the object's k-axis,  plane passes through the object's position.
    // The normals point inward so that the FORBIDDEN_ZONE VFIs keep the robot
    // inside the box.
    // Wall at x=+h, inward normal -i  (rotate k -> -i: -90 deg about y)
    set_object_pose("cube_40x40_wall_1",
                    make_pose(make_rotation(y_axis, -pi / 2.0), Vector3d(h, 0., 0.)));
    // Wall at x=-h, inward normal +i  (rotate k -> +i: +90 deg about y)
    set_object_pose("cube_40x40_wall_2",
                    make_pose(make_rotation(y_axis, pi / 2.0), Vector3d(-h, 0., 0.)));
    // Wall at y=+h, inward normal -j  (rotate k -> -j: +90 deg about x)
    set_object_pose("cube_40x40_wall_3",
                    make_pose(make_rotation(x_axis, pi / 2.0), Vector3d(0., h, 0.)));
    // Wall at y=-h, inward normal +j  (rotate k -> +j: -90 deg about x)
    set_object_pose("cube_40x40_wall_4",
                    make_pose(make_rotation(x_axis, -pi / 2.0), Vector3d(0., -h, 0.)));

    // Two vertical tubes (lines) along z, inside the box. The tube object's
    // k-axis is the tube axis (no rotation); the line passes through the
    // object's position.
    set_object_pose("cube_40x40_tube_1", make_pose(identity, Vector3d(0., 0.12, 0.)));
    set_object_pose("cube_40x40_tube_2", make_pose(identity, Vector3d(0., -0.12, 0.)));

    // Task targets.
    // xd0: a safe approach pose inside the box, in front of the opening.
    set_object_pose("xd0", make_pose(identity, Vector3d(0.0, 0.0, 0.0)));
    // xd1: the final target, chosen (as in the original example) as a pose the
    // robot cannot reach; it serves to show that even then the robot does not
    // collide with the environment.
    set_object_pose("xd1", make_pose(identity, Vector3d(0.0, 0.0, 0.55)));

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
