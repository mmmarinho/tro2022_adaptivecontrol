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

void initialize_robot(DQ_SerialManipulatorEDH& robot);


int main(int argc, char** argv)
{
    try{
        signal(SIGINT, sig_int_handler);

        /************************************************************************
         * INITIALIZATION
         * *********************************************************************/
        SimulationArguments simulation_arguments;
        simulation_arguments.measure_space = DQ_MeasureSpace::Pose;
        simulation_arguments.proportional_gain = 20.0;
        simulation_arguments.vfi_gain = 0.5;
        simulation_arguments.vfi_weight = 0.25;
        simulation_arguments.damping = 0.001;
        simulation_arguments.use_adaptation = true;

        auto vi = std::make_shared<DQ_VrepInterface>();
        if(!vi->connect(19997, 100, 100))
        {
            vi->disconnect_all();
            throw std::runtime_error("Failed to connect to CoppeliaSim.");
        }
        vi->start_video_recording();
        vi->start_simulation();

        SmartArm1EDH vrep_robot("VS050", vi);


        //Real robot
        //if(simulation_arguments.use_real_robot)
        //{
        //    throw std::runtime_error("NOT IMPLEMENTED IN THIS VERSION");
        //}

        VectorXd q_init(vrep_robot.get_configuration_space_positions());
        VectorXd q(q_init);

        /************************************************************************
         * Estimated robot (the one in our kinematic model)
         * *********************************************************************/

        DQ_SerialManipulatorEDH estimated_robot = SmartArm1EDH::raw_kinematics();
        //Base frame is measured roughly using a tape measure
        const DQ& base_frame_t = 0.50*i_ + 0.6*j_ + 0.61*k_;
        const DQ& base_frame_r = (cos(pi/4.0) + i_*sin(pi/4.0))*(cos(pi/2.0) + k_*sin(pi/2.0));
        const DQ& base_frame = (1. + 0.5*E_*base_frame_t)*base_frame_r;
        estimated_robot.set_base_frame(base_frame);
        vrep_robot.send_base_frame_to_vrep(base_frame, "aruco_reference");
        //The end-effector measurements are obtained from the CAD model
        const DQ& r = cos(-pi/4.0) + i_*sin(-pi/4.0);
        const DQ& effector_frame = r + 0.5 * E_ * k_ * 0.15688 * r;
        estimated_robot.set_effector_frame(effector_frame);
        initialize_robot(estimated_robot);
        std::tuple<VectorXd, VectorXd> parameter_boundaries = estimated_robot.get_parameter_space_boundaries();
        //Velocity limits
        const double& ROBOT_JOINT_VELOCITY_LIMIT = 0.01;
        estimated_robot.set_upper_q_dot_limit(ROBOT_JOINT_VELOCITY_LIMIT*VectorXd::Ones(estimated_robot.get_dim_configuration_space()));
        estimated_robot.set_lower_q_dot_limit(-ROBOT_JOINT_VELOCITY_LIMIT*VectorXd::Ones(estimated_robot.get_dim_configuration_space()));

        /************************************************************************
         * Real robot (the one in the simulation)
         * *********************************************************************/

        std::unique_ptr<DQ_SerialManipulatorEDH> real_robot;
        //if(!simulation_arguments.use_real_robot)
        //{
        real_robot = std::make_unique<DQ_SerialManipulatorEDH>(SmartArm1EDH::raw_kinematics());
        //real_robot->set_base_frame(vrep_robot.get_base_frame_from_vrep());
        real_robot->set_base_frame(base_frame);
        real_robot->set_effector_frame(effector_frame);
        initialize_robot(*real_robot);
        //}


        /************************************************************************
         * Measurement
         * *********************************************************************/

        DQ xd_measurement = vi->get_object_pose("aruco_referece");

        /************************************************************************
         * Initialize xd
         * *********************************************************************/

        std::cout << "[3] Initializing xd..." << std::endl;

        const DQ& xd_1 = xd_measurement*(1 + 0.5*E_*(-0.02*k_));
        const DQ& xd_0 = xd_1*(1 + 0.5*E_*(0.15*j_));
        std::vector<DQ> xds = {
            xd_0, //Safe approach prior DQ (Going)
            xd_1, //Target DQ
        };
        vi->set_object_pose("xd0",xds[0],"aruco_reference");
        vi->set_object_pose("xd1",xds[1],"aruco_reference");

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
                                           std::get<DQ>(tp),
                                           "aruco_reference"));
            vfis.push_back(VFI_Information("cube_40x40_wall_2",
                                           std::get<2>(tp),
                                           DQ_Primitive::Plane,
                                           vi,
                                           std::get<double>(tp)+wall_distance,
                                           VFI_Direction::FORBIDDEN_ZONE,
                                           7,
                                           std::get<DQ>(tp),
                                           "aruco_reference"));
            vfis.push_back(VFI_Information("cube_40x40_wall_3",
                                           std::get<2>(tp),
                                           DQ_Primitive::Plane,
                                           vi,
                                           std::get<double>(tp)+wall_distance,
                                           VFI_Direction::FORBIDDEN_ZONE,
                                           7,
                                           std::get<DQ>(tp),
                                           "aruco_reference"));
            vfis.push_back(VFI_Information("cube_40x40_wall_4",
                                           std::get<2>(tp),
                                           DQ_Primitive::Plane,
                                           vi,
                                           std::get<double>(tp)+wall_distance,
                                           VFI_Direction::FORBIDDEN_ZONE,
                                           7,
                                           std::get<DQ>(tp),
                                           "aruco_reference"));
            vfis.push_back(VFI_Information("cube_40x40_tube_1",
                                           std::get<2>(tp),
                                           DQ_Primitive::Line,
                                           vi,
                                           powf64x(std::get<double>(tp)+tube_distance, 2.),
                                           VFI_Direction::FORBIDDEN_ZONE,
                                           7,
                                           std::get<DQ>(tp),
                                           "aruco_reference"));

            vfis.push_back(VFI_Information("cube_40x40_tube_2",
                                           std::get<2>(tp),
                                           DQ_Primitive::Line,
                                           vi,
                                           powf64x(std::get<double>(tp)+tube_distance, 2.),
                                           VFI_Direction::FORBIDDEN_ZONE,
                                           7,
                                           std::get<DQ>(tp),
                                           "aruco_reference"));
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
        //if(simulation_arguments;.get_parameters_from_parameter_server)
        //{
        //    throw std::runtime_error("NOT IMPLEMENTED");
        //}
        //else
        //{
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

            vrep_robot.send_base_frame_to_vrep(estimated_robot.get_base_frame(), "aruco_reference");

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
        if(found_suitable_parameters)
        {
            //node_handle.setParam("a_hat_init", rosilo::vectorxd_to_std_vector_double(a_hat));
        }
        //}

        /************************************************************************
         * Adaptive Control Loop
         * *********************************************************************/

        //Simple state-machine
        //state <- MEASUREMENT_ONLY
        //while measurement_error > threshold
        //  measurement_error <- get_measurement_error()
        //  {~, ua} = adaptive_control(state)
        //  update_parameters(ua)

        //state <- FULL
        //while task_error > threshold
        //  measurement <- get_measurement()
        //  if state == FULL && measurement == INVALID
        //    state <- TASK_ONLY
        //  if state == TASK_ONLY && measurement == VALID
        //    state <- FULL
        //  {uq, ua} = adaptive_control(state)
        //  update_q(uq)
        //  update_parameters(ua)


        std::cout << "[6] Initializing Adaptive Control..." << std::endl;

        const double& MEASUREMENT_ONLY_TIME = 10.0; // 10 seconds
        const double& FULL_ADAPTATION_TIME = 60.0*5.0; // 5 minutes
        const double& ARUCO_IGNORE_RANGE = 0.08; // 8 cm
        bool aruco_ignore_switch = false;

        //Other states are only needed for real sensors that have noise, lost measurements etc.
        //Things that don't happen with a simulated sensor
        std::cout << "Starting with DQ_AdaptiveControlStrategy::FULL" << std::endl;
        DQ_AdaptiveControlStrategy control_strategy(DQ_AdaptiveControlStrategy::FULL);

        const double& sampling_time_sec = 0.02;
        sas::Clock clock(sampling_time_sec);
        clock.init();

        /************************************************************************
         * For all xds
         * *********************************************************************/
        for(int xd_counter = 0; xd_counter < xds.size(); xd_counter++)
        {
            const DQ& xd = xds[xd_counter];

            while(!kill_this_process)
            {
                //if(simulation_arguments;.use_real_robot)
                //{
                //    throw std::runtime_error("NOT IMPLEMENTED");
                //}

                const DQ& x_hat = estimated_robot.fkm(q);
                vi->set_object_pose("x_hat",  x_hat, "aruco_reference");

                //DQ y(0);
                //if(not simulation_arguments;.use_real_robot)
                //{
                DQ y = real_robot->fkm(q);
                vi->set_object_pose("x", y, "aruco_reference");
                //}
                //else
                //{
                // Update our current robot estimate on VREP
                //vrep_robot.send_base_frame_to_vrep(estimated_robot.get_base_frame(), "aruco_reference");

                // try
                // {
                // Get robot end effector sensor unfiltered measurement
                //y = vi->get_object_pose("aruco_reference");
                //vi->set_object_pose("x", y, "aruco_reference");

                //If the measurement of "x" is too close to our measurement for xd_1,
                //we forcefully ignore it because ARUCO isn't reliable in that range.
                //if(vec3(translation(y) - translation(xd_1)).norm() < ARUCO_IGNORE_RANGE)
                //{
                //    if(!aruco_ignore_switch)
                //    {
                //        std::cout << "Aruco ignore switch activated" << std::endl;
                //        aruco_ignore_switch = true;
                //    }
                //    y = 0.;
                //    throw std::runtime_error("Ignoring y. Too close to xd_1 for a reliable measurement.");
                //}

                //If we are able to obtain a measurement and we are in State 2.b)
                //State 2.b) TASK_ONLY -> State 2.a) FULL
                //if(control_strategy == DQ_AdaptiveControlStrategy::TASK_ONLY && !aruco_ignore_switch)
                //{
                //State 2.a) FULL
                //    control_strategy = DQ_AdaptiveControlStrategy::FULL;
                //    std::cout << "Switched to DQ_AdaptiveControlStrategy::FULL (2)" << std::endl;
                //}

                //}
                //catch(const std::exception& e)
                //{
                //std::cout << "INVALID READING FROM SENSOR " << std::endl;
                //If we are unable to obtain a measurement and we are in State 2.a)
                //State 2.a) FULL -> State 2.b) TASK_ONLY
                //if(control_strategy == DQ_AdaptiveControlStrategy::FULL)
                //{
                //State 2.b) TASK_ONLY
                //   control_strategy = DQ_AdaptiveControlStrategy::TASK_ONLY;
                //    std::cout << "Switched to DQ_AdaptiveControlStrategy::TASK_ONLY (1)" << std::endl;
                //}
                //}

                //}

                // Parameter adaptation law
                VectorXd uq;
                VectorXd ua;
                VectorXd x_tilde;
                VectorXd y_tilde;
                DQ y_partial;
                std::tie(uq, ua, x_tilde, y_tilde, y_partial) = compute_setpoint_control_signal(control_strategy,
                                                                                                q,
                                                                                                estimated_robot,
                                                                                                xd,
                                                                                                y,
                                                                                                vfis,
                                                                                                simulation_arguments);

                if(control_strategy == DQ_AdaptiveControlStrategy::MEASUREMENT_ONLY && clock.get_elapsed_time_sec() > MEASUREMENT_ONLY_TIME)
                {
                    //State 2.a) FULL
                    control_strategy = DQ_AdaptiveControlStrategy::FULL;
                    std::cout << "Switched to DQ_AdaptiveControlStrategy::FULL (1)" << std::endl;
                }

                // Robot state:
                //datalogger.log("q", q);
                //datalogger.log("q_min", estimated_robot.get_lower_q_limit());
                //datalogger.log("q_dot_min", estimated_robot.get_lower_q_dot_limit());
                //datalogger.log("q_max", estimated_robot.get_upper_q_limit());
                //datalogger.log("q_dot_max", estimated_robot.get_upper_q_dot_limit());
                //datalogger.log("x_hat", vec8(x_hat));

                // Task state:
                //datalogger.log("xd", vec8(xd));
                //datalogger.log("task_error", x_tilde);
                //datalogger.log("task_error_norm", x_tilde.norm());

                // Measurement:
                //datalogger.log("y", vec8(y));
                //datalogger.log("y_partial", vec8(y_partial));
                //datalogger.log("measurement_error", y_tilde);
                //datalogger.log("measurement_error_norm", y_tilde.norm());

                //datalogger.log("time", clock.get_elapsed_time_sec());

                // Control signals
                //datalogger.log("uq", uq);
                //datalogger.log("uq_norm", uq.norm());
                //datalogger.log("ua", ua);
                //datalogger.log("ua_norm", ua.norm());

                // Control strategy
                //datalogger.log("control_strategy", static_cast<int>(control_strategy));

                // Box measurement
                //datalogger.log("box_measurement", box_measurement.vec8());

                // Estimated parameters:
                //datalogger.log("a_hat", a_hat);
                //datalogger.log("a_hat_min", std::get<0>(parameter_boundaries));
                //datalogger.log("a_hat_max", std::get<1>(parameter_boundaries));

                // Clock
                //datalogger.log("computation_time", clock.get_computation_time());
                //datalogger.log("overrun_count", clock.get_overrun_count());
                //datalogger.log("get_effective_thread_sampling_time_sec", clock.get_effective_thread_sampling_time_sec());
                //datalogger.log("get_desired_thread_sampling_time_sec", clock.get_desired_thread_sampling_time_sec());
                //datalogger.log("elapsed_time", clock.get_elapsed_time_sec());

                // VFI
                //for(const VFI_Information& vfi: vfis)
                //{
                //datalogger.log("estimated_distance_"+vfi.get_vfi_name(), vfi.get_last_estimated_distance());
                //datalogger.log("real_distance_"+vfi.get_vfi_name(), vfi.get_last_real_distance());
                //datalogger.log("safe_distance_"+vfi.get_vfi_name(), vfi.get_safe_distance());
                //}


                if(simulation_arguments.use_adaptation)
                {
                    a_hat += ua*sampling_time_sec;
                    estimated_robot.set_parameter_space_values(a_hat);
                }


                q += uq*sampling_time_sec;
                vrep_robot.set_configuration_space_positions(q);
                //if(simulation_arguments.use_real_robot)
                //{
                //    throw std::runtime_error("NOT IMPLEMENTED");
                //}

                //ros::spinOnce();
                clock.update_and_sleep();
                //ros::spinOnce();

                if(control_strategy != DQ_AdaptiveControlStrategy::MEASUREMENT_ONLY && clock.get_elapsed_time_sec() > FULL_ADAPTATION_TIME*(xd_counter+1) + MEASUREMENT_ONLY_TIME)
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

        std::cout << "[7] Saving log file..." << std::endl;

        //datalogger.log("proportional_gain", command_line_arguments.proportional_gain);
        //datalogger.log("vfi_gain", command_line_arguments.vfi_gain);
        //datalogger.log("vfi_weight", command_line_arguments.vfi_weight);
        //datalogger.log("measure_space", int(command_line_arguments.measure_space));
        //datalogger.log("damping", command_line_arguments.damping);
        //datalogger.log("use_adaptation", command_line_arguments.use_adaptation);

        std::chrono::system_clock::time_point p = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(p);
        std::stringstream ss;
        ss << std::ctime(&t);
        std::string current_time_as_a_string = ss.str();
        std::replace(current_time_as_a_string.begin(), current_time_as_a_string.end(), ':', '_');
        std::replace(current_time_as_a_string.begin(), current_time_as_a_string.end(), ' ', '_');
        std::replace(current_time_as_a_string.begin(), current_time_as_a_string.end(), '\n', '_');
        std::string filename("vfi_simulation" +
                             std::to_string(simulation_arguments.proportional_gain)        + std::string("_g_") +
                std::to_string(int(simulation_arguments.measure_space))       + std::string("_ms_") +
                std::to_string(simulation_arguments.damping)                  + std::string("_d_") +
                std::to_string(simulation_arguments.use_adaptation)           + std::string("_a_") +
                std::to_string(simulation_arguments.vfi_gain)                 + std::string("_vfig_") +
                std::to_string(simulation_arguments.vfi_weight)               + std::string("_vfiw_") +
                current_time_as_a_string +
                std::string(".mat"));
        //datalogger.log("filename", filename);

        //ros::spinOnce();

        //datalogger.save(filename);

        vi->stop_simulation();
    }
    catch(const std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }

    //if(robot_driver_denso)
    //{
    //    robot_driver_denso->deinitialize();
    //    robot_driver_denso->disconnect();
    //}

    return 0;
}

void initialize_robot(DQ_SerialManipulatorEDH& robot)
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


