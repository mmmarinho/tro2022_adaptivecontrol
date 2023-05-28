/**
(C) Copyright 2020-2023 Murilo Marques Marinho (www.murilomarinho.info)

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

#include "example/Example_AdaptiveController.h"
#include "example/Example_VFI.h"
#include "example/Example_SerialManipulatorEDH.h"
#include "example/Example_VS050VrepRobot.h"

#include <signal.h>
#include <atomic>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}

void set_parameter_space_boundaries(const std::shared_ptr<Example_SerialManipulatorEDH> &robot,
                                    const double& base_linear_confidence_meters = 0.1,
                                    const double& base_angular_confidence_degrees = 20,
                                    const double& effector_linear_confidence_meters = 0.01,
                                    const double& effector_angular_confidence_degrees = 5,
                                    const double& other_parameters_linear_confidence_meters = 0.001,
                                    const double& other_parameters_angular_confidence_degrees = 1);

std::vector<Example_VFI> get_example_scene_vfis(const std::shared_ptr<DQ_VrepInterface>& vi);

void randomize_parameters(const std::shared_ptr<Example_SerialManipulatorEDH> &estimated_robot,
                          const std::tuple<VectorXd, VectorXd> &parameter_boundaries,
                          const VectorXd &q,
                          const std::vector<Example_VFI> &vfis);


int main(int, char**)
{
    signal(SIGINT, sig_int_handler);

    try
    {

        std::cout << "Example code for: \n"
                     "M. M. Marinho and B. V. Adorno\n"
                     "Adaptive Constrained Kinematic Control Using Partial or Complete Task-Space Measurements\n"
                     "in IEEE Transactions on Robotics, vol. 38, no. 6, pp. 3498-3513, Dec. 2022\n"
                     "doi: 10.1109/TRO.2022.3181047\n"
                  << std::endl;

        std::cout << "Disclaimer: \n"
                     "This code has been modified from the one used in the paper's experiments to not depend\n"
                     "on ROS, sensors, etc.\n"
                     "Bugs might be present, so report them at https://github.com/mmmarinho/tro2022_adaptivecontrol/issues\n"
                  << std::endl;

        /// The behavior is somewhat robust to the change in parameters, and by choosing the ones below
        /// there's no claim that they are optimal for any case, even only for this example.

        Example_SimulationParameters simulation_parameters;
        simulation_parameters.measure_space = Example_MeasureSpace::Pose;
        simulation_parameters.proportional_gain = 20.0;
        simulation_parameters.vfi_gain = 5;
        simulation_parameters.vfi_weight = 0.02;
        simulation_parameters.damping = 0.01;
        simulation_parameters.sampling_time_sec = 0.08; //The physical VS050 have a default joint control frequency of 125 Hz
        simulation_parameters.reference_timeout_sec = 60;

        /// *********************************************************************
        /// Connect to CoppeliaSim
        /// *********************************************************************

        std::cout << "[1] Connecting to CoppeliaSim..." << std::endl;

        auto vi = std::make_shared<DQ_VrepInterface>();
        if(!vi->connect(19997, 100, 100))
        {
            vi->disconnect_all();
            throw std::runtime_error("Failed to connect to CoppeliaSim. "
                                     "Make sure that CoppeliaSim is running "
                                     "with the correct scene file opened."
                         #ifdef __APPLE__
                                     "\nFor macos users, note that 'simRemoteApi.start(19997)' "
                                     "must be added to the main script and the simulation "
                                     "must be started before running this example. "
                         #endif
                                     );
        }

#ifndef __APPLE__
        //Limitations in CoppeliaSim on macos make this directive to not play well with macos
        vi->stop_simulation();
#endif

        /// ************************************************************************
        /// CoppeliaSim Robot and Models
        /// ************************************************************************

        std::cout << "[2] Initializing CoppeliaSim robot and models..." << std::endl;

        Example_VS050VrepRobot real_robot_in_vrep("VS050", vi);
        VectorXd q_init(real_robot_in_vrep.get_configuration_space_positions());

        //Real base (representing the ideal robot parameters)

        DQ real_base_frame(real_robot_in_vrep.get_base_frame());

        //Real effector (from the CAD model)

        const DQ& r = cos(-pi/4.0) + i_*sin(-pi/4.0);
        const DQ& effector_frame = r + 0.5 * E_ * k_ * 0.15688 * r;

        //Real robot (representing the ideal robot parameters)

        auto real_robot = std::make_shared<Example_SerialManipulatorEDH>(Example_VS050VrepRobot::raw_kinematics());
        real_robot->set_base_frame(real_base_frame);
        real_robot->set_effector_frame(effector_frame);
        set_parameter_space_boundaries(real_robot);

        //Estimated robot (the estimated kinematic model, the one that needs adaptation)

        auto estimated_robot = std::make_shared<Example_SerialManipulatorEDH>(Example_VS050VrepRobot::raw_kinematics());
        estimated_robot->set_base_frame(real_base_frame);
        estimated_robot->set_effector_frame(effector_frame);
        set_parameter_space_boundaries(estimated_robot);

        auto parameter_boundaries = estimated_robot->get_parameter_space_boundaries();
        //Velocity limits (simplified, this also helps to keep the real robot from moving too fast)
        const double& ROBOT_JOINT_VELOCITY_LIMIT = 0.1;
        estimated_robot->set_upper_q_dot_limit(ROBOT_JOINT_VELOCITY_LIMIT*VectorXd::Ones(estimated_robot->get_dim_configuration_space()));
        estimated_robot->set_lower_q_dot_limit(-ROBOT_JOINT_VELOCITY_LIMIT*VectorXd::Ones(estimated_robot->get_dim_configuration_space()));

        /// ************************************************************************
        /// Initialize xd
        /// ************************************************************************

        std::cout << "[3] Initializing xd..." << std::endl;

        const DQ& xd_1 = vi->get_object_pose("xd1");
        const DQ& xd_0 = vi->get_object_pose("xd0");
        std::vector<DQ> xds = {
            xd_0, //Safe approach prior DQ (Going)
            xd_1, //Target DQ
        };

        /// ************************************************************************
        /// Initialize VFIs
        /// ************************************************************************

        std::cout << "[4] Initializing VFIs..." << std::endl;

        std::vector<Example_VFI> vfis(get_example_scene_vfis(vi));

        /// ************************************************************************
        /// Randomize the estimated parameters, so that our adaptive controller
        /// has something to do in this example!
        /// ************************************************************************

        std::cout << "[5] Making our initial parameter estimate wrong, but plausible..." << std::endl;

        randomize_parameters(estimated_robot,
                             parameter_boundaries,
                             q_init,
                             vfis);

        VectorXd a_hat_init(estimated_robot->get_parameter_space_values());

        /// ************************************************************************
        /// Adaptive Control Loop
        /// ************************************************************************

        Example_AdaptiveController adaptive_controller(estimated_robot,
                                                       simulation_parameters);

#ifndef __APPLE__
        vi->start_video_recording();
        vi->start_simulation();
#endif


        /// ************************************************************************
        /// Run once with adaptation and once without adaptation
        /// ************************************************************************
        for(const auto& control_strategy : {Example_AdaptiveControlStrategy::FULL,
            Example_AdaptiveControlStrategy::TASK_ONLY})
        {
            VectorXd q = q_init;
            real_robot_in_vrep.set_configuration_space_positions(q_init);
            estimated_robot->set_parameter_space_values(a_hat_init);
            VectorXd a_hat = a_hat_init;

            if(control_strategy == Example_AdaptiveControlStrategy::FULL)
            {
                std::cout << "[6] Running with full adaptation. " << std::endl;
            }
            else if(control_strategy == Example_AdaptiveControlStrategy::TASK_ONLY)
            {
                std::cout << "[7] Running WITHOUT adaptation. " << std::endl;
            }
            else
            {
                throw std::runtime_error("Not implemented yet.");
            }

            sas::Clock clock(simulation_parameters.sampling_time_sec);
            clock.init();

            /// ************************************************************************
            /// For all xds
            /// ************************************************************************
            for(int xd_counter = 0; xd_counter < xds.size(); xd_counter++)
            {
                const DQ& xd = xds[xd_counter];

                while(!kill_this_process)
                {
                    const DQ& x_hat = estimated_robot->fkm(q);
                    vi->set_object_pose("x_hat",  x_hat);


                    DQ y = real_robot->fkm(q);
                    vi->set_object_pose("x", y);

                    auto [uq, ua, x_tilde, y_tilde, y_partial] = adaptive_controller.compute_setpoint_control_signal(control_strategy,
                            q,
                            xd,
                            y,
                            vfis);

                    a_hat += ua*simulation_parameters.sampling_time_sec;
                    estimated_robot->set_parameter_space_values(a_hat);

                    q += uq*simulation_parameters.sampling_time_sec;
                    real_robot_in_vrep.set_configuration_space_positions(q);

                    clock.update_and_sleep();

                    if(clock.get_elapsed_time_sec() > simulation_parameters.reference_timeout_sec*(xd_counter+1))
                    {
                        std::cout << "Reference timeout for xd" << xd_counter << std::endl;
                        std::cout << "  Average computational time = " << clock.get_statistics(sas::Statistics::Mean,sas::Clock::TimeType::Computational) << " seconds." << std::endl;
                        std::cout << "  Clock overruns = " << clock.get_overrun_count() << " (Too many, i.e. hundreds, indicate that the sampling time is too low for this CPU)."<< std::endl;
                        std::cout << "  Final task pose error norm = " << x_tilde.norm() << " (Dual quaternion norm)." << std::endl;
                        std::cout << "  Final task translation error norm = " << (translation(x_hat)-translation(xd)).norm() << " (in meters)." << std::endl;
                        std::cout << "  Final measurement error norm = " << y_tilde.norm() << " (Dual quaternion norm)." << std::endl;
                        if(is_unit(y))
                            std::cout << "  Final measurement translation error norm = " << (translation(x_hat)-translation(y)).norm() << " (in meters)." << std::endl;
                        else
                            std::cout << "  measurement translation error norm: y not unit" << std::endl;
                        break;
                    }

                }
            }
        }

        vi->stop_simulation();

    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}

/**
 * @brief set_parameter_space_boundaries In this example, the confidence of the parameters of the base is lower than the confidence of the other
 * parameters of the robot. This function sets proper values for the boundaries, which are used by the adaptive controller.
 * @param robot a DQ_SerialManipulatorEDH with base and effector properly initialized.
 * @param base_linear_confidence_meters confidence interval for the linear parameters of the base [m].
 * @param base_angular_confidence_degrees confidence interval for the angular parameters of the base [deg].
 * @param effector_linear_confidence_meters confidence interval for the linear parameters of the effector [m].
 * @param effector_angular_confidence_degrees confidence interval for the angular parameters of the effector [deg].
 * @param other_parameters_linear_confidence_meters confidence interval for the linear parameters of the other parameters [m].
 * @param other_parameters_angular_confidence_degrees confidence interval for the angular parameters of the other parameters [deg].
 */
void set_parameter_space_boundaries(const std::shared_ptr<Example_SerialManipulatorEDH>& robot,
                                    const double& base_linear_confidence_meters,
                                    const double& base_angular_confidence_degrees,
                                    const double& effector_linear_confidence_meters,
                                    const double& effector_angular_confidence_degrees,
                                    const double& other_parameters_linear_confidence_meters,
                                    const double& other_parameters_angular_confidence_degrees)
{
    //Aliases
    const double& bl = base_linear_confidence_meters;
    const double& ba = base_angular_confidence_degrees;
    const double& el = effector_linear_confidence_meters;
    const double& ea = effector_angular_confidence_degrees;
    const double& opl = other_parameters_linear_confidence_meters;
    const double& opa = other_parameters_angular_confidence_degrees;

    std::vector<Example_ParameterSpaceEDH::Example_Parameter> bp = robot->get_base_parameters();
    std::vector<Example_ParameterSpaceEDH::Example_Parameter> ep = robot->get_effector_parameters();

    const std::shared_ptr<Example_SerialManipulatorEDH>& r = robot;
    std::vector<Example_ParameterSpaceEDH::Example_Parameter> parameter_space;

    parameter_space    =
    {
        Example_ParameterSpaceEDH::Example_Parameter(-1, Example_ParameterSpaceEDH::Example_ParameterType::base_x,     bp[0].value_,     bp[0].value_-bl,            bp[0].value_+bl),
        Example_ParameterSpaceEDH::Example_Parameter(-1, Example_ParameterSpaceEDH::Example_ParameterType::base_y,     bp[1].value_,     bp[1].value_-bl,            bp[1].value_+bl),
        Example_ParameterSpaceEDH::Example_Parameter(-1, Example_ParameterSpaceEDH::Example_ParameterType::base_z,     bp[2].value_,     bp[2].value_-bl,            bp[2].value_+bl),
        Example_ParameterSpaceEDH::Example_Parameter(-1, Example_ParameterSpaceEDH::Example_ParameterType::base_alpha, bp[3].value_,     bp[3].value_-deg2rad(ba),   bp[3].value_+deg2rad(ba)),
        Example_ParameterSpaceEDH::Example_Parameter(-1, Example_ParameterSpaceEDH::Example_ParameterType::base_beta,  bp[4].value_,     bp[4].value_-deg2rad(ba),   bp[4].value_+deg2rad(ba)),
        Example_ParameterSpaceEDH::Example_Parameter(-1, Example_ParameterSpaceEDH::Example_ParameterType::base_gamma, bp[5].value_,     bp[5].value_-deg2rad(ba),   bp[5].value_+deg2rad(ba)),

        Example_ParameterSpaceEDH::Example_Parameter(0, Example_ParameterSpaceEDH::Example_ParameterType::theta,       r->get_theta(0),   r->get_theta(0)-deg2rad(opa),    r->get_theta(0)+deg2rad(opa)),
        Example_ParameterSpaceEDH::Example_Parameter(0, Example_ParameterSpaceEDH::Example_ParameterType::d,           r->get_d(0),       r->get_d(0)-opl,                 r->get_d(0)+opl),
        Example_ParameterSpaceEDH::Example_Parameter(0, Example_ParameterSpaceEDH::Example_ParameterType::a,           r->get_a(0),       r->get_a(0)-opl,                 r->get_a(0)+opl),
        Example_ParameterSpaceEDH::Example_Parameter(0, Example_ParameterSpaceEDH::Example_ParameterType::alpha,       r->get_alpha(0),   r->get_alpha(0)-deg2rad(opa),    r->get_alpha(0)+deg2rad(opa)),

        Example_ParameterSpaceEDH::Example_Parameter(1, Example_ParameterSpaceEDH::Example_ParameterType::theta,       r->get_theta(1),   r->get_theta(1)-deg2rad(opa),    r->get_theta(1)+deg2rad(opa)),
        Example_ParameterSpaceEDH::Example_Parameter(1, Example_ParameterSpaceEDH::Example_ParameterType::d,           r->get_d(1),       r->get_d(1)-opl,                 r->get_d(1)+opl),
        Example_ParameterSpaceEDH::Example_Parameter(1, Example_ParameterSpaceEDH::Example_ParameterType::a,           r->get_a(1),       r->get_a(1)-opl,                 r->get_a(1)+opl),
        Example_ParameterSpaceEDH::Example_Parameter(1, Example_ParameterSpaceEDH::Example_ParameterType::alpha,       r->get_alpha(1),   r->get_alpha(1)-deg2rad(opa),    r->get_alpha(1)+deg2rad(opa)),

        Example_ParameterSpaceEDH::Example_Parameter(2, Example_ParameterSpaceEDH::Example_ParameterType::theta,       r->get_theta(2),   r->get_theta(2)-deg2rad(opa),    r->get_theta(2)+deg2rad(opa)),
        Example_ParameterSpaceEDH::Example_Parameter(2, Example_ParameterSpaceEDH::Example_ParameterType::d,           r->get_d(2),       r->get_d(2)-opl,                 r->get_d(2)+opl),
        Example_ParameterSpaceEDH::Example_Parameter(2, Example_ParameterSpaceEDH::Example_ParameterType::a,           r->get_a(2),       r->get_a(2)-opl,                 r->get_a(2)+opl),
        Example_ParameterSpaceEDH::Example_Parameter(2, Example_ParameterSpaceEDH::Example_ParameterType::alpha,       r->get_alpha(2),   r->get_alpha(2)-deg2rad(opa),    r->get_alpha(2)+deg2rad(opa)),

        Example_ParameterSpaceEDH::Example_Parameter(3, Example_ParameterSpaceEDH::Example_ParameterType::theta,       r->get_theta(3),   r->get_theta(3)-deg2rad(opa),    r->get_theta(3)+deg2rad(opa)),
        Example_ParameterSpaceEDH::Example_Parameter(3, Example_ParameterSpaceEDH::Example_ParameterType::d,           r->get_d(3),       r->get_d(3)-opl,                 r->get_d(3)+opl),
        Example_ParameterSpaceEDH::Example_Parameter(3, Example_ParameterSpaceEDH::Example_ParameterType::a,           r->get_a(3),       r->get_a(3)-opl,                 r->get_a(3)+opl),
        Example_ParameterSpaceEDH::Example_Parameter(3, Example_ParameterSpaceEDH::Example_ParameterType::alpha,       r->get_alpha(3),   r->get_alpha(3)-deg2rad(opa),    r->get_alpha(3)+deg2rad(opa)),

        Example_ParameterSpaceEDH::Example_Parameter(4, Example_ParameterSpaceEDH::Example_ParameterType::theta,       r->get_theta(4),   r->get_theta(4)-deg2rad(opa),    r->get_theta(4)+deg2rad(opa)),
        Example_ParameterSpaceEDH::Example_Parameter(4, Example_ParameterSpaceEDH::Example_ParameterType::d,           r->get_d(4),       r->get_d(4)-opl,                 r->get_d(4)+opl),
        Example_ParameterSpaceEDH::Example_Parameter(4, Example_ParameterSpaceEDH::Example_ParameterType::a,           r->get_a(4),       r->get_a(4)-opl,                 r->get_a(4)+opl),
        Example_ParameterSpaceEDH::Example_Parameter(4, Example_ParameterSpaceEDH::Example_ParameterType::alpha,       r->get_alpha(4),   r->get_alpha(4)-deg2rad(opa),    r->get_alpha(4)+deg2rad(opa)),

        Example_ParameterSpaceEDH::Example_Parameter(5, Example_ParameterSpaceEDH::Example_ParameterType::theta,       r->get_theta(5),   r->get_theta(5)-deg2rad(opa),    r->get_theta(5)+deg2rad(opa)),
        Example_ParameterSpaceEDH::Example_Parameter(5, Example_ParameterSpaceEDH::Example_ParameterType::d,           r->get_d(5),       r->get_d(5)-opl,                 r->get_d(5)+opl),
        Example_ParameterSpaceEDH::Example_Parameter(5, Example_ParameterSpaceEDH::Example_ParameterType::a,           r->get_a(5),       r->get_a(5)-opl,                 r->get_a(5)+opl),
        Example_ParameterSpaceEDH::Example_Parameter(5, Example_ParameterSpaceEDH::Example_ParameterType::alpha,       r->get_alpha(5),   r->get_alpha(5)-deg2rad(opa),    r->get_alpha(5)+deg2rad(opa)),

        Example_ParameterSpaceEDH::Example_Parameter(6, Example_ParameterSpaceEDH::Example_ParameterType::eff_x,     ep[0].value_,     ep[0].value_-el,           ep[0].value_+el),
        Example_ParameterSpaceEDH::Example_Parameter(6, Example_ParameterSpaceEDH::Example_ParameterType::eff_y,     ep[1].value_,     ep[1].value_-el,           ep[1].value_+el),
        Example_ParameterSpaceEDH::Example_Parameter(6, Example_ParameterSpaceEDH::Example_ParameterType::eff_z,     ep[2].value_,     ep[2].value_-el,           ep[2].value_+el),
        Example_ParameterSpaceEDH::Example_Parameter(6, Example_ParameterSpaceEDH::Example_ParameterType::eff_alpha, ep[3].value_,     ep[3].value_-deg2rad(ea),   ep[3].value_+deg2rad(ea)),
        Example_ParameterSpaceEDH::Example_Parameter(6, Example_ParameterSpaceEDH::Example_ParameterType::eff_beta,  ep[4].value_,     ep[4].value_-deg2rad(ea),   ep[4].value_+deg2rad(ea)),
        Example_ParameterSpaceEDH::Example_Parameter(6, Example_ParameterSpaceEDH::Example_ParameterType::eff_gamma, ep[5].value_,     ep[5].value_-deg2rad(ea),   ep[5].value_+deg2rad(ea)),
    };

    robot->set_parameter_space(parameter_space);
}

/**
 * @brief get_example_scene_vfis Initializes the VFIs in the example CoppeliaSim scene,
 * TRO2022_MarinhoAdorno_ReferenceScene.ttt
 * @param vi a DQ_VrepInterface that has already connected.
 * @return a vector of VFI_Information containing all relevant VFIs in the scene.
 */
std::vector<Example_VFI> get_example_scene_vfis(const std::shared_ptr<DQ_VrepInterface>& vi)
{
    std::vector<Example_VFI> vfis;

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
        vfis.push_back(Example_VFI("cube_40x40_wall_1",
                                   std::get<2>(tp),
                                   Example_Primitive::Plane,
                                   vi,
                                   std::get<double>(tp)+wall_distance,
                                   Example_VFI_Direction::FORBIDDEN_ZONE,
                                   7,
                                   std::get<DQ>(tp))
                       );
        vfis.push_back(Example_VFI("cube_40x40_wall_2",
                                   std::get<2>(tp),
                                   Example_Primitive::Plane,
                                   vi,
                                   std::get<double>(tp)+wall_distance,
                                   Example_VFI_Direction::FORBIDDEN_ZONE,
                                   7,
                                   std::get<DQ>(tp))
                       );
        vfis.push_back(Example_VFI("cube_40x40_wall_3",
                                   std::get<2>(tp),
                                   Example_Primitive::Plane,
                                   vi,
                                   std::get<double>(tp)+wall_distance,
                                   Example_VFI_Direction::FORBIDDEN_ZONE,
                                   7,
                                   std::get<DQ>(tp))
                       );
        vfis.push_back(Example_VFI("cube_40x40_wall_4",
                                   std::get<2>(tp),
                                   Example_Primitive::Plane,
                                   vi,
                                   std::get<double>(tp)+wall_distance,
                                   Example_VFI_Direction::FORBIDDEN_ZONE,
                                   7,
                                   std::get<DQ>(tp))
                       );
        vfis.push_back(Example_VFI("cube_40x40_tube_1",
                                   std::get<2>(tp),
                                   Example_Primitive::Line,
                                   vi,
                                   std::pow(std::get<double>(tp)+tube_distance, 2.),
                                   Example_VFI_Direction::FORBIDDEN_ZONE,
                                   7,
                                   std::get<DQ>(tp))
                       );

        vfis.push_back(Example_VFI("cube_40x40_tube_2",
                                   std::get<2>(tp),
                                   Example_Primitive::Line,
                                   vi,
                                   std::pow(std::get<double>(tp)+tube_distance, 2.),
                                   Example_VFI_Direction::FORBIDDEN_ZONE,
                                   7,
                                   std::get<DQ>(tp))
                       );
    }

    for(Example_VFI& vfi: vfis)
    {
        vfi.initialize();
    }

    return vfis;
}

/**
 * @brief randomize_parameters a function to randomize the parameters of the estimated_robot within
 * its boundaries, considering its current configuration q and VFI violations using vfis.
 * @param estimated_robot the estimated_robot whose parameters will be changed.
 * @param parameter_boundaries the {lower,upper} limits of the parameters.
 * @param q the current configuration of the robot.
 * @param vfis the std::vector of Example_VFIs with all VFIs to be checked.
 */
void randomize_parameters(
        const std::shared_ptr<Example_SerialManipulatorEDH>& estimated_robot,
        const std::tuple<VectorXd,VectorXd>& parameter_boundaries,
        const VectorXd& q,
        const std::vector<Example_VFI>& vfis)
{
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

        //Try a set of random parameters
        finding_suitable_parameters_counter++;

        const VectorXd One_p = VectorXd::Ones(estimated_robot->get_dim_parameter_space());
        const VectorXd noise_weights = (VectorXd::Random(estimated_robot->get_dim_parameter_space())+One_p)*0.5;
        const VectorXd noise_affected_parameters = noise_weights.cwiseProduct(std::get<0>(parameter_boundaries)) + (One_p - noise_weights).cwiseProduct(std::get<1>(parameter_boundaries));
        estimated_robot->set_parameter_space_values(noise_affected_parameters);
        const VectorXd a_hat_candidate = estimated_robot->get_parameter_space_values();

        // The random parameters must not penetrate any obstacle
        const DQ x_hat = estimated_robot->fkm(q);
        found_suitable_parameters = true;
        for(const Example_VFI& vfi: vfis)
        {
            const double& distance_error = vfi.get_distance_error(x_hat);
            switch(vfi.get_distance_type())
            {
            case Example_VFI_DistanceType::None:
                throw std::runtime_error("Expected valid type.");
            case Example_VFI_DistanceType::EUCLIDEAN:
                if(distance_error < -0.001)
                {
                    found_suitable_parameters = false;
                    break;
                }
            case Example_VFI_DistanceType::EUCLIDEAN_SQUARED:
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

    }
    if(!found_suitable_parameters)
    {
        throw std::runtime_error("Unable to find suitable parameters.");
    }
}
