/*
# Copyright (c) 2020-2023 Murilo Marques Marinho (www.murilomarinho.info)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     - Neither the name of the Mitsuishi Sugita Laboratory (NML) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################*/
#include <memory>
#include <string>
#include <deque>
#include <random>
#include <thread>

#include <dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/utils/DQ_Math.h>
#include <dqrobotics/utils/DQ_Geometry.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>

#include <sas_core/sas_clock.hpp>

#include "robot_control/DQ_AdaptiveController.h"
#include "constraints_modeling/VFI_Information.h"
#include "interfaces/vrep/robots/SmartArm1EDH.h"
#include "utils.h"

#include <signal.h>
#include <atomic>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}

void set_parameter_space_boundaries(DQ_SerialManipulatorEDH& robot);


int main(int argc, char** argv)
{
    //try{
        signal(SIGINT, sig_int_handler);

        /************************************************************************
         * Simulation Arguments
         * *********************************************************************/
        SimulationArguments simulation_arguments;
        simulation_arguments.measure_space = DQ_MeasureSpace::Pose;
        simulation_arguments.proportional_gain = 20.0;
        simulation_arguments.vfi_gain = 0.5;
        simulation_arguments.vfi_weight = 0.25;
        simulation_arguments.damping = 0.001;
        simulation_arguments.use_adaptation = true;

        /************************************************************************
         * Connect to CoppeliaSim
         * *********************************************************************/

        auto vi = std::make_shared<DQ_VrepInterface>();
        if(!vi->connect("127.0.0.1",19997, 100, 100))
        {
            vi->disconnect_all();
            throw std::runtime_error("Failed to connect to CoppeliaSim.");
        }
        vi->start_video_recording();
        vi->stop_simulation();

        SmartArm1EDH real_robot_in_vrep("VS050", vi);

        /************************************************************************
         * Initial joint values
         * *********************************************************************/

        VectorXd q_init(real_robot_in_vrep.get_configuration_space_positions());
        VectorXd q(q_init);

        /************************************************************************
         * Real base (the one in the simulation)
         * *********************************************************************/

        DQ real_base_frame(real_robot_in_vrep.get_base_frame());

        /************************************************************************
         * Real effector (from the CAD model)
         * *********************************************************************/

        const DQ& r = cos(-pi/4.0) + i_*sin(-pi/4.0);
        const DQ& effector_frame = r + 0.5 * E_ * k_ * 0.15688 * r;

        /************************************************************************
         * Real robot (the one in the simulation)
         * *********************************************************************/

        DQ_SerialManipulatorEDH real_robot(SmartArm1EDH::raw_kinematics());
        real_robot.set_base_frame(real_base_frame);
        real_robot.set_effector_frame(effector_frame);
        set_parameter_space_boundaries(real_robot);

        //real_robot_in_vrep.set_base_frame(real_base_frame);

        /************************************************************************
         * Estimated robot (the one in our kinematic model)
         * *********************************************************************/

        DQ_SerialManipulatorEDH estimated_robot(SmartArm1EDH::raw_kinematics());
        estimated_robot.set_base_frame(real_base_frame);
        estimated_robot.set_effector_frame(effector_frame);
        set_parameter_space_boundaries(estimated_robot);

        auto parameter_boundaries = estimated_robot.get_parameter_space_boundaries();
        //Velocity limits (simplified, this also helps to keep the real robot from moving too fast)
        const double& ROBOT_JOINT_VELOCITY_LIMIT = 0.01;
        estimated_robot.set_upper_q_dot_limit(ROBOT_JOINT_VELOCITY_LIMIT*VectorXd::Ones(estimated_robot.get_dim_configuration_space()));
        estimated_robot.set_lower_q_dot_limit(-ROBOT_JOINT_VELOCITY_LIMIT*VectorXd::Ones(estimated_robot.get_dim_configuration_space()));

        /************************************************************************
         * Initialize xd
         * *********************************************************************/

        std::cout << "[3] Initializing xd..." << std::endl;

        const DQ& xd_1 = vi->get_object_pose("xd1");
        const DQ& xd_0 = vi->get_object_pose("xd0");
        std::vector<DQ> xds = {
            xd_0, //Safe approach prior DQ (Going)
            xd_1, //Target DQ
        };

        /************************************************************************
         * Initialize VFIs
         * *********************************************************************/

        std::cout << "[4] Initializing VFIs..." << std::endl;

        std::vector<VFI_Information> vfis;
        std::vector<std::tuple<DQ,double,std::string>> vfi_reference_dqs = {
            {vi->get_object_pose("tool_sphere_1","x_hat"),0.04,"tool_sphere_1"},
            {vi->get_object_pose("tool_sphere_2","x_hat"),0.015,"tool_sphere_2"},
            {vi->get_object_pose("tool_sphere_3","x_hat"),0.015,"tool_sphere_3"},
            {vi->get_object_pose("tool_sphere_4","x_hat"),0.015,"tool_sphere_4"},
            {vi->get_object_pose("tool_sphere_5","x_hat"),0.015,"tool_sphere_5"},
            {vi->get_object_pose("tool_sphere_6","x_hat"),0.075,"tool_sphere_6"}
        };

        const double wall_distance = 0.02;
        const double tube_distance = 0.02;

        for(const std::tuple<DQ,double,std::string>& tp : vfi_reference_dqs)
        {
            vfis.push_back(VFI_Information("cube_40x40_wall_1",
                                           std::get<2>(tp),
                                           DQ_Primitive::Plane,
                                           vi,
                                           std::get<double>(tp)+wall_distance,
                                           VFI_Direction::FORBIDDEN_ZONE,
                                           7,
                                           std::get<DQ>(tp))
                           );
            vfis.push_back(VFI_Information("cube_40x40_wall_2",
                                           std::get<2>(tp),
                                           DQ_Primitive::Plane,
                                           vi,
                                           std::get<double>(tp)+wall_distance,
                                           VFI_Direction::FORBIDDEN_ZONE,
                                           7,
                                           std::get<DQ>(tp))
                           );
            vfis.push_back(VFI_Information("cube_40x40_wall_3",
                                           std::get<2>(tp),
                                           DQ_Primitive::Plane,
                                           vi,
                                           std::get<double>(tp)+wall_distance,
                                           VFI_Direction::FORBIDDEN_ZONE,
                                           7,
                                           std::get<DQ>(tp))
                           );
            vfis.push_back(VFI_Information("cube_40x40_wall_4",
                                           std::get<2>(tp),
                                           DQ_Primitive::Plane,
                                           vi,
                                           std::get<double>(tp)+wall_distance,
                                           VFI_Direction::FORBIDDEN_ZONE,
                                           7,
                                           std::get<DQ>(tp))
                           );
            vfis.push_back(VFI_Information("cube_40x40_tube_1",
                                           std::get<2>(tp),
                                           DQ_Primitive::Line,
                                           vi,
                                           powf64x(std::get<double>(tp)+tube_distance, 2.),
                                           VFI_Direction::FORBIDDEN_ZONE,
                                           7,
                                           std::get<DQ>(tp))
                           );

            vfis.push_back(VFI_Information("cube_40x40_tube_2",
                                           std::get<2>(tp),
                                           DQ_Primitive::Line,
                                           vi,
                                           powf64x(std::get<double>(tp)+tube_distance, 2.),
                                           VFI_Direction::FORBIDDEN_ZONE,
                                           7,
                                           std::get<DQ>(tp))
                           );
        }

        for(VFI_Information& vfi: vfis)
        {
            vfi.initialize();
        }

        /************************************************************************
         * Get a suitable initial estimate of the parameters
         * *********************************************************************/

        std::cout << "[5] Trying to find suitable initial parameters..." << std::endl;

        VectorXd a_hat(estimated_robot.get_parameter_space_values());
        bool found_suitable_parameters = false;
        sas::Clock find_suitable_parameters_clock(0.002);
        find_suitable_parameters_clock.init();
        const double& FIND_SUITABLE_PARAMETERS_TIMEOUT = 10.0;
        int finding_suitable_parameters_counter = 0;
        while(not found_suitable_parameters && not kill_this_process)
        {
            if(find_suitable_parameters_clock.get_elapsed_time_sec() > FIND_SUITABLE_PARAMETERS_TIMEOUT)
                throw std::runtime_error("Timeout in finding suitable initial parameters");
            std::cout << "  Finding suitable parameters {" << find_suitable_parameters_clock.get_elapsed_time_sec() <<
                         "," << finding_suitable_parameters_counter << "}." << std::endl;
            finding_suitable_parameters_counter++;

            real_robot_in_vrep.set_base_frame(estimated_robot.get_base_frame());

            const DQ x_hat = estimated_robot.fkm(q);

            found_suitable_parameters = true;
            for(const VFI_Information& vfi: vfis)
            {
                const double& distance_error = vfi.get_distance_error(x_hat);
                switch(vfi.get_distance_type())
                {
                case VFI_DistanceType::None:
                    throw std::runtime_error("Expected valid type.");
                case VFI_DistanceType::EUCLIDEAN:
                    if(distance_error < -0.001)
                    {
                        found_suitable_parameters = false;
                        break;
                    }
                case VFI_DistanceType::EUCLIDEAN_SQUARED:
                    if(distance_error < -0.00001)
                    {
                        found_suitable_parameters = false;
                        break;
                    }
                }
                if(!found_suitable_parameters)
                    break;
            }
            if(found_suitable_parameters)
                break;

            //Try another set of random parameters
            const VectorXd One_p = VectorXd::Ones(estimated_robot.get_dim_parameter_space());
            const VectorXd noise_weights = (VectorXd::Random(estimated_robot.get_dim_parameter_space())+One_p)*0.5;
            const VectorXd noise_affected_parameters = noise_weights.cwiseProduct(std::get<0>(parameter_boundaries)) + (One_p - noise_weights).cwiseProduct(std::get<1>(parameter_boundaries));
            estimated_robot.set_parameter_space_values(noise_affected_parameters);
            a_hat = estimated_robot.get_parameter_space_values();
        }
        if(!found_suitable_parameters)
        {
            throw std::runtime_error("Unable to find suitable parameters.");
        }

        /************************************************************************
         * Adaptive Control Loop
         * *********************************************************************/

        std::cout << "[6] Initializing Adaptive Control..." << std::endl;

        //Other states are only needed for real sensors that have noise, lost measurements etc.
        //Things that don't happen with a simulated sensor
        std::cout << "Starting with DQ_AdaptiveControlStrategy::FULL" << std::endl;
        DQ_AdaptiveControlStrategy control_strategy(DQ_AdaptiveControlStrategy::FULL);

        const double& sampling_time_sec = 0.02;
        sas::Clock clock(sampling_time_sec);
        clock.init();

        vi->start_simulation();

        /************************************************************************
         * For all xds
         * *********************************************************************/
        for(int xd_counter = 0; xd_counter < xds.size(); xd_counter++)
        {
            const DQ& xd = xds[xd_counter];

            while(!kill_this_process)
            {
                const DQ& x_hat = estimated_robot.fkm(q);
                vi->set_object_pose("x_hat",  x_hat);


                DQ y = real_robot.fkm(q);
                vi->set_object_pose("x", y);

                // Parameter adaptation law
                auto [uq, ua, x_tilde, y_tilde, y_partial] = compute_setpoint_control_signal(control_strategy,
                                                                                                q,
                                                                                                estimated_robot,
                                                                                                xd,
                                                                                                y,
                                                                                                vfis,
                                                                                                simulation_arguments);


                if(simulation_arguments.use_adaptation)
                {
                    a_hat += ua*sampling_time_sec;
                    estimated_robot.set_parameter_space_values(a_hat);
                }

                q += uq*sampling_time_sec;
                real_robot_in_vrep.set_configuration_space_positions(q);

                clock.update_and_sleep();

                if(clock.get_elapsed_time_sec() > 60.0)
                {
                    std::cout << "xd timeout" << std::endl;
                    std::cout << "  x_tilde norm " << x_tilde.norm() << std::endl;
                    std::cout << "  translation error norm " << (translation(x_hat)-translation(xd)).norm() << std::endl;
                    std::cout << "  y_tilde_norm " << y_tilde.norm() << std::endl;
                    if(is_unit(y))
                        std::cout << "  measurement translation error norm " << (translation(x_hat)-translation(y)).norm() << std::endl;
                    else
                        std::cout << "  measurement translation error norm: y not unit" << std::endl;
                    break;
                }

            }
        }

        vi->stop_simulation();
    //}
    //catch(const std::exception& e)
   // {
   //     std::cout << e.what() << std::endl;
   // }

    return 0;
}

void set_parameter_space_boundaries(DQ_SerialManipulatorEDH& robot)
{
    std::vector<DQ_ParameterSpaceEDH::Parameter> bp = robot.get_base_parameters();
    std::vector<DQ_ParameterSpaceEDH::Parameter> ep = robot.get_effector_parameters();

    const DQ_SerialManipulatorEDH& r = robot;
    std::vector<DQ_ParameterSpaceEDH::Parameter> parameter_space;

    parameter_space    =
    {
        DQ_ParameterSpaceEDH::Parameter(-1, DQ_ParameterSpaceEDH::ParameterType::base_x,     bp[0].value_,     bp[0].value_-0.1,           bp[0].value_+0.1),
        DQ_ParameterSpaceEDH::Parameter(-1, DQ_ParameterSpaceEDH::ParameterType::base_y,     bp[1].value_,     bp[1].value_-0.1,           bp[1].value_+0.1),
        DQ_ParameterSpaceEDH::Parameter(-1, DQ_ParameterSpaceEDH::ParameterType::base_z,     bp[2].value_,     bp[2].value_-0.1,           bp[2].value_+0.1),
        DQ_ParameterSpaceEDH::Parameter(-1, DQ_ParameterSpaceEDH::ParameterType::base_alpha, bp[3].value_,     bp[3].value_-deg2rad(20),   bp[3].value_+deg2rad(20)),
        DQ_ParameterSpaceEDH::Parameter(-1, DQ_ParameterSpaceEDH::ParameterType::base_beta,  bp[4].value_,     bp[4].value_-deg2rad(20),   bp[4].value_+deg2rad(20)),
        DQ_ParameterSpaceEDH::Parameter(-1, DQ_ParameterSpaceEDH::ParameterType::base_gamma, bp[5].value_,     bp[5].value_-deg2rad(20),   bp[5].value_+deg2rad(20)),

        DQ_ParameterSpaceEDH::Parameter(0, DQ_ParameterSpaceEDH::ParameterType::theta,       r.get_theta(0),         r.get_theta(0)-deg2rad(1),    r.get_theta(0)+deg2rad(1)),
        DQ_ParameterSpaceEDH::Parameter(0, DQ_ParameterSpaceEDH::ParameterType::d,           r.get_d(0),             r.get_d(0)-0.001,             r.get_d(0)+0.001),
        DQ_ParameterSpaceEDH::Parameter(0, DQ_ParameterSpaceEDH::ParameterType::a,           r.get_a(0),             r.get_a(0)-0.001,             r.get_a(0)+0.001),
        DQ_ParameterSpaceEDH::Parameter(0, DQ_ParameterSpaceEDH::ParameterType::alpha,       r.get_alpha(0),         r.get_alpha(0)-deg2rad(1),    r.get_alpha(0)+deg2rad(1)),

        DQ_ParameterSpaceEDH::Parameter(1, DQ_ParameterSpaceEDH::ParameterType::theta,       r.get_theta(1),         r.get_theta(1)-deg2rad(1),    r.get_theta(1)+deg2rad(1)),
        DQ_ParameterSpaceEDH::Parameter(1, DQ_ParameterSpaceEDH::ParameterType::d,           r.get_d(1),             r.get_d(1)-0.001,             r.get_d(1)+0.001),
        DQ_ParameterSpaceEDH::Parameter(1, DQ_ParameterSpaceEDH::ParameterType::a,           r.get_a(1),             r.get_a(1)-0.001,             r.get_a(1)+0.001),
        DQ_ParameterSpaceEDH::Parameter(1, DQ_ParameterSpaceEDH::ParameterType::alpha,       r.get_alpha(1),         r.get_alpha(1)-deg2rad(1),    r.get_alpha(1)+deg2rad(1)),

        DQ_ParameterSpaceEDH::Parameter(2, DQ_ParameterSpaceEDH::ParameterType::theta,       r.get_theta(2),         r.get_theta(2)-deg2rad(1),    r.get_theta(2)+deg2rad(1)),
        DQ_ParameterSpaceEDH::Parameter(2, DQ_ParameterSpaceEDH::ParameterType::d,           r.get_d(2),             r.get_d(2)-0.001,             r.get_d(2)+0.001),
        DQ_ParameterSpaceEDH::Parameter(2, DQ_ParameterSpaceEDH::ParameterType::a,           r.get_a(2),             r.get_a(2)-0.001,             r.get_a(2)+0.001),
        DQ_ParameterSpaceEDH::Parameter(2, DQ_ParameterSpaceEDH::ParameterType::alpha,       r.get_alpha(2),         r.get_alpha(2)-deg2rad(1),    r.get_alpha(2)+deg2rad(1)),

        DQ_ParameterSpaceEDH::Parameter(3, DQ_ParameterSpaceEDH::ParameterType::theta,       r.get_theta(3),         r.get_theta(3)-deg2rad(1),    r.get_theta(3)+deg2rad(1)),
        DQ_ParameterSpaceEDH::Parameter(3, DQ_ParameterSpaceEDH::ParameterType::d,           r.get_d(3),             r.get_d(3)-0.001,             r.get_d(3)+0.001),
        DQ_ParameterSpaceEDH::Parameter(3, DQ_ParameterSpaceEDH::ParameterType::a,           r.get_a(3),             r.get_a(3)-0.001,             r.get_a(3)+0.001),
        DQ_ParameterSpaceEDH::Parameter(3, DQ_ParameterSpaceEDH::ParameterType::alpha,       r.get_alpha(3),         r.get_alpha(3)-deg2rad(1),    r.get_alpha(3)+deg2rad(1)),

        DQ_ParameterSpaceEDH::Parameter(4, DQ_ParameterSpaceEDH::ParameterType::theta,       r.get_theta(4),         r.get_theta(4)-deg2rad(1),    r.get_theta(4)+deg2rad(1)),
        DQ_ParameterSpaceEDH::Parameter(4, DQ_ParameterSpaceEDH::ParameterType::d,           r.get_d(4),             r.get_d(4)-0.001,             r.get_d(4)+0.001),
        DQ_ParameterSpaceEDH::Parameter(4, DQ_ParameterSpaceEDH::ParameterType::a,           r.get_a(4),             r.get_a(4)-0.001,             r.get_a(4)+0.001),
        DQ_ParameterSpaceEDH::Parameter(4, DQ_ParameterSpaceEDH::ParameterType::alpha,       r.get_alpha(4),         r.get_alpha(4)-deg2rad(1),    r.get_alpha(4)+deg2rad(1)),

        DQ_ParameterSpaceEDH::Parameter(5, DQ_ParameterSpaceEDH::ParameterType::theta,       r.get_theta(5),         r.get_theta(5)-deg2rad(1),    r.get_theta(5)+deg2rad(1)),
        DQ_ParameterSpaceEDH::Parameter(5, DQ_ParameterSpaceEDH::ParameterType::d,           r.get_d(5),             r.get_d(5)-0.001,             r.get_d(5)+0.001),
        DQ_ParameterSpaceEDH::Parameter(5, DQ_ParameterSpaceEDH::ParameterType::a,           r.get_a(5),             r.get_a(5)-0.001,             r.get_a(5)+0.001),
        DQ_ParameterSpaceEDH::Parameter(5, DQ_ParameterSpaceEDH::ParameterType::alpha,       r.get_alpha(5),         r.get_alpha(5)-deg2rad(1),    r.get_alpha(5)+deg2rad(1)),

        DQ_ParameterSpaceEDH::Parameter(6, DQ_ParameterSpaceEDH::ParameterType::eff_x,     ep[0].value_,     ep[0].value_-0.01,           ep[0].value_+0.01),
        DQ_ParameterSpaceEDH::Parameter(6, DQ_ParameterSpaceEDH::ParameterType::eff_y,     ep[1].value_,     ep[1].value_-0.01,           ep[1].value_+0.01),
        DQ_ParameterSpaceEDH::Parameter(6, DQ_ParameterSpaceEDH::ParameterType::eff_z,     ep[2].value_,     ep[2].value_-0.01,           ep[2].value_+0.01),
        DQ_ParameterSpaceEDH::Parameter(6, DQ_ParameterSpaceEDH::ParameterType::eff_alpha, ep[3].value_,     ep[3].value_-deg2rad(5),   ep[3].value_+deg2rad(5)),
        DQ_ParameterSpaceEDH::Parameter(6, DQ_ParameterSpaceEDH::ParameterType::eff_beta,  ep[4].value_,     ep[4].value_-deg2rad(5),   ep[4].value_+deg2rad(5)),
        DQ_ParameterSpaceEDH::Parameter(6, DQ_ParameterSpaceEDH::ParameterType::eff_gamma, ep[5].value_,     ep[5].value_-deg2rad(5),   ep[5].value_+deg2rad(5)),
    };

    robot.set_parameter_space(parameter_space);
}


