/**
 * (C) Copyright 2023-2026 Murilo Marques Marinho (www.murilomarinho.info)
 *
 * This file is part of adaptive_control_example.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "marinholab/papers/tro2022/adaptive_control/AdaptiveController.h"
#include "marinholab/papers/tro2022/adaptive_control/VFI.h"
#include "marinholab/papers/tro2022/adaptive_control/SerialManipulatorEDH.h"
#include "marinholab/papers/tro2022/adaptive_control/SimulatorDummy.h"
#include "marinholab/papers/tro2022/adaptive_control/MeasurementSpace.h"

using namespace marinholab::papers::tro2022::adaptive_control;

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;
using namespace DQ_robotics;

PYBIND11_MODULE(_core, m) {

    /// "example/MeasurementSpace.h"

    //enum class MeasureSpace
    py::enum_<MeasureSpace>(m, "M3_MeasureSpace")
            .value("None", MeasureSpace::None)
            .value("Pose", MeasureSpace::Pose)
            .value("Rotation", MeasureSpace::Rotation)
            .value("Translation", MeasureSpace::Translation)
            .value("Distance", MeasureSpace::Distance)
            .export_values();

    /// "example/AdaptiveController.h"

    //enum class Example_AdaptiveControlStrategy
    py::enum_<Example_AdaptiveControlStrategy>(m, "Example_AdaptiveControlStrategy")
            .value("NONE", Example_AdaptiveControlStrategy::NONE)
            .value("TASK_ONLY", Example_AdaptiveControlStrategy::TASK_ONLY)
            .value("MEASUREMENT_ONLY", Example_AdaptiveControlStrategy::MEASUREMENT_ONLY)
            .value("FULL", Example_AdaptiveControlStrategy::FULL)
            .export_values();

    //struct Example_SimulationParameters
    py::class_<Example_SimulationParameters>(m, "Example_SimulationParameters")
            .def(py::init
                 <
                 const MeasureSpace&,
                 const double&,
                 const double&,
                 const double&,
                 const double&,
                 const double&,
                 const double&
                 >(),
                py::arg("measure_space"),
                py::arg("proportional_gain"),
                py::arg("vfi_gain"),
                py::arg("vfi_weight"),
                py::arg("damping"),
                py::arg("sampling_time_sec"),
                py::arg("reference_timeout_sec")
             )
            .def_readwrite("measure_space",&Example_SimulationParameters::measure_space)
            .def_readwrite("proportional_gain",&Example_SimulationParameters::proportional_gain)
            .def_readwrite("vfi_gain",&Example_SimulationParameters::vfi_gain)
            .def_readwrite("vfi_weight",&Example_SimulationParameters::vfi_weight)
            .def_readwrite("damping",&Example_SimulationParameters::damping)
            .def_readwrite("sampling_time_sec",&Example_SimulationParameters::sampling_time_sec)
            .def_readwrite("reference_timeout_sec",&Example_SimulationParameters::reference_timeout_sec);

    //class AdaptiveController
    py::class_
            <
            AdaptiveController,
            std::shared_ptr<AdaptiveController>
            >(m, "M3_AdaptiveController")
            .def(py::init
                 <const std::shared_ptr<SerialManipulatorEDH>&,
                 const Example_SimulationParameters &
                 >())
            .def("compute_setpoint_control_signal",&AdaptiveController::compute_setpoint_control_signal,"");

    /// "example/AdaptiveController.h"

    //enum class Primitive
    py::enum_<Primitive>(m, "M3_Primitive")
            .value("None", Primitive::None)
            .value("Point", Primitive::Point)
            .value("Plane", Primitive::Plane)
            .value("Line", Primitive::Line)
            .export_values();

    //enum class VFI_Direction
    py::enum_<VFI_Direction>(m, "M3_VFI_Direction")
            .value("None", VFI_Direction::None)
            .value("FORBIDDEN_ZONE", VFI_Direction::FORBIDDEN_ZONE)
            .value("SAFE_ZONE", VFI_Direction::SAFE_ZONE)
            .export_values();

    //enum class VFI_DistanceType
    py::enum_<VFI_DistanceType>(m, "M3_VFI_DistanceType")
            .value("None", VFI_DistanceType::None)
            .value("EUCLIDEAN", VFI_DistanceType::EUCLIDEAN)
            .value("EUCLIDEAN_SQUARED", VFI_DistanceType::EUCLIDEAN_SQUARED)
            .export_values();

    //class VFI
    py::class_<VFI,std::shared_ptr<VFI>>(m, "M3_VFI")
            .def(py::init
                 <
                 const std::string&,
                 const std::string&,
                 const Primitive&,
                 const std::shared_ptr<SimulatorDummy>&,
                 const double&,
                 const VFI_Direction&,
                 const int&,
                 const DQ&,
                 const std::string&
                 >())
            .def("initialize",&VFI::initialize,"")
            .def("get_value",&VFI::get_value,"")
            .def("set_value",&VFI::set_value,"")
            .def("get_distance_jacobian",&VFI::get_distance_jacobian,"")
            .def("get_vfi_matrix",&VFI::get_vfi_matrix,"")
            .def("get_distance",&VFI::get_distance,"")
            .def("get_distance_error",&VFI::get_distance_error,"")
            .def("get_safe_distance",&VFI::get_safe_distance,"")
            .def("get_distance_type",&VFI::get_distance_type,"")
            .def("set_last_real_distance",&VFI::set_last_real_distance,"")
            .def("get_last_real_distance",&VFI::get_last_real_distance,"")
            .def("set_last_estimated_distance",&VFI::set_last_estimated_distance,"")
            .def("get_last_estimated_distance",&VFI::get_last_estimated_distance,"")
            .def("get_vfi_name",&VFI::get_vfi_name,"");

    /// "example/SerialManipulatorEDH.h"

    //namespace ParameterSpaceEDH
    py::module ParameterSpaceEDH = m.def_submodule("_M3_ParameterSpaceEDH", "");

    //enum class Example_ParameterType
    py::enum_<ParameterSpaceEDH::Example_ParameterType>(ParameterSpaceEDH, "Example_ParameterType")
            .value("theta", ParameterSpaceEDH::Example_ParameterType::theta)
            .value("d", ParameterSpaceEDH::Example_ParameterType::d)
            .value("a", ParameterSpaceEDH::Example_ParameterType::a)
            .value("alpha", ParameterSpaceEDH::Example_ParameterType::alpha)

            .value("base_x", ParameterSpaceEDH::Example_ParameterType::base_x)
            .value("base_y", ParameterSpaceEDH::Example_ParameterType::base_y)
            .value("base_z", ParameterSpaceEDH::Example_ParameterType::base_z)
            .value("base_alpha", ParameterSpaceEDH::Example_ParameterType::base_alpha)
            .value("base_beta", ParameterSpaceEDH::Example_ParameterType::base_beta)
            .value("base_gamma", ParameterSpaceEDH::Example_ParameterType::base_gamma)

            .value("eff_x", ParameterSpaceEDH::Example_ParameterType::eff_x)
            .value("eff_y", ParameterSpaceEDH::Example_ParameterType::eff_y)
            .value("eff_z", ParameterSpaceEDH::Example_ParameterType::eff_z)
            .value("eff_alpha", ParameterSpaceEDH::Example_ParameterType::eff_alpha)
            .value("eff_beta", ParameterSpaceEDH::Example_ParameterType::eff_beta)
            .value("eff_gamma", ParameterSpaceEDH::Example_ParameterType::eff_gamma)
            .export_values();

    //struct Example_Parameter
    py::class_<ParameterSpaceEDH::Example_Parameter>(ParameterSpaceEDH, "Example_Parameter")
            .def(py::init
                 <
                 const int&,
                 const ParameterSpaceEDH::Example_ParameterType&,
                 const double&,
                 const double&,
                 const double&
                 >())
            .def_readwrite("link_index_",&ParameterSpaceEDH::Example_Parameter::link_index_)
            .def_readwrite("type_",&ParameterSpaceEDH::Example_Parameter::type_)
            .def_readwrite("value_",&ParameterSpaceEDH::Example_Parameter::value_)
            .def_readwrite("min_",&ParameterSpaceEDH::Example_Parameter::min_)
            .def_readwrite("max_",&ParameterSpaceEDH::Example_Parameter::max_);

    // This is originally wrapped in dqrobotics. Better to know what to expect in this case. Nonetheless, this was needed
    // in 2025.05 and otherwise had the error
    // ImportError: generic_type: type "M3_SerialManipulatorEDH" referenced unknown base type "DQ_robotics::DQ_SerialManipulator"
    // This was solved by me a long time ago, the DQ_SerialManipulator imported be installed beforehand in Python.
    // see test_python_wrapper.sh or look up the history if the file no longer exists.
    // However, this does not seem to always work. It might fail for the slightest of differences in versions.
    // https://pybind11.readthedocs.io/en/stable/advanced/misc.html
    // py::module_::import("dqrobotics");
    // py::module_::import("dqrobotics.robot_modeling");

    //class SerialManipulatorEDH : public DQ_SerialManipulator
    py::class_
            <
            SerialManipulatorEDH,
            std::shared_ptr<SerialManipulatorEDH>,
            DQ_SerialManipulator
            > SerialManipulatorEDH
            (
                m,
                "M3_SerialManipulatorEDH"
                );

    py::enum_<SerialManipulatorEDH::JOINT_TYPES>(SerialManipulatorEDH, "JOINT_TYPES")
            .value("JOINT_ROTATIONAL", SerialManipulatorEDH::JOINT_TYPES::JOINT_ROTATIONAL)
            .value("JOINT_PRISMATIC", SerialManipulatorEDH::JOINT_TYPES::JOINT_PRISMATIC)
            .export_values();

    SerialManipulatorEDH.def(py::init
                                     <
                                     const MatrixXd&
                                     >());

    //    DQ get_base_frame() const;
    SerialManipulatorEDH.def("get_base_frame",&SerialManipulatorEDH::get_base_frame,"");
    //    std::vector<ParameterSpaceEDH::Example_Parameter> get_base_parameters() const;
    SerialManipulatorEDH.def("get_base_parameters",&SerialManipulatorEDH::get_base_parameters,"");
    //    void set_base_frame(const std::vector<ParameterSpaceEDH::Example_Parameter> &base_parameters);
    SerialManipulatorEDH.def("set_base_frame",
                                     (void (SerialManipulatorEDH::*)(const std::vector<ParameterSpaceEDH::Example_Parameter>&))
                                     (&SerialManipulatorEDH::set_base_frame),
                                     "");
    //    void set_base_frame(const DQ& base);
    SerialManipulatorEDH.def("set_base_frame",
                                     (void (SerialManipulatorEDH::*)(const DQ&))
                                     (&SerialManipulatorEDH::set_base_frame),
                                     "");
    //    DQ get_effector_frame() const;
    SerialManipulatorEDH.def("get_effector_frame",&SerialManipulatorEDH::get_effector_frame,"");
    //    std::vector<ParameterSpaceEDH::Example_Parameter> get_effector_parameters() const;
    SerialManipulatorEDH.def("get_effector_parameters",&SerialManipulatorEDH::get_effector_parameters,"");
    //    void set_effector_frame(const std::vector<ParameterSpaceEDH::Example_Parameter>& effector_parameters);
    SerialManipulatorEDH.def("set_effector_frame",
                                     (void (SerialManipulatorEDH::*)(const std::vector<ParameterSpaceEDH::Example_Parameter>&))
                                     (&SerialManipulatorEDH::set_effector_frame),
                                     "");
    //    void set_effector_frame(const DQ& effector);
    SerialManipulatorEDH.def("set_effector_frame",
                                     (void (SerialManipulatorEDH::*)(const DQ&))
                                     (&SerialManipulatorEDH::set_effector_frame),
                                     "");
    //    void set_parameter_space(const std::vector<ParameterSpaceEDH::Example_Parameter>& parameter_space);
    SerialManipulatorEDH.def("set_parameter_space",&SerialManipulatorEDH::set_parameter_space,"");
    //    bool is_parameter_space_set() const;
    SerialManipulatorEDH.def("is_parameter_space_set",&SerialManipulatorEDH::is_parameter_space_set,"");
    //    int get_dim_parameter_space() const;
    SerialManipulatorEDH.def("get_dim_parameter_space",&SerialManipulatorEDH::get_dim_parameter_space,"");
    //    VectorXd get_parameter_space_values() const;
    SerialManipulatorEDH.def("get_parameter_space_values",&SerialManipulatorEDH::get_parameter_space_values,"");
    //    void set_parameter_space_values(const VectorXd& parameter_space_vector);
    SerialManipulatorEDH.def("set_parameter_space_values",&SerialManipulatorEDH::set_parameter_space_values,"");
    //    void set_parameter_space_boundaries(const std::tuple<VectorXd, VectorXd>& boundaries);
    SerialManipulatorEDH.def("set_parameter_space_boundaries",&SerialManipulatorEDH::set_parameter_space_boundaries,"");
    //    std::tuple<VectorXd,VectorXd> get_parameter_space_boundaries() const;
    SerialManipulatorEDH.def("get_parameter_space_boundaries",&SerialManipulatorEDH::get_parameter_space_boundaries,"");
    //    std::vector<ParameterSpaceEDH::Example_ParameterType> get_parameter_types() const;
    SerialManipulatorEDH.def("get_parameter_types",&SerialManipulatorEDH::get_parameter_types,"");
    //    VectorXd get_link_types() const;
    SerialManipulatorEDH.def("get_link_types",&SerialManipulatorEDH::get_link_types,"");
    //    double get_link_type(const int& link_index) const;
    SerialManipulatorEDH.def("get_link_type",&SerialManipulatorEDH::get_link_type,"");
    //    VectorXd get_thetas() const;
    SerialManipulatorEDH.def("get_thetas",&SerialManipulatorEDH::get_thetas,"");
    //    double get_theta(const int& link_index) const;
    SerialManipulatorEDH.def("get_theta",&SerialManipulatorEDH::get_theta,"");
    //    void set_theta(const int& link_index, double const& value);
    SerialManipulatorEDH.def("set_theta",&SerialManipulatorEDH::set_theta,"");
    //    VectorXd get_ds() const;
    SerialManipulatorEDH.def("get_ds",&SerialManipulatorEDH::get_ds,"");
    //    double get_d(const int& link_index) const;
    SerialManipulatorEDH.def("get_d",&SerialManipulatorEDH::get_d,"");
    //    void set_d(const int& link_index, double const& value);
    SerialManipulatorEDH.def("set_d",&SerialManipulatorEDH::set_d,"");
    //    VectorXd get_as() const;
    SerialManipulatorEDH.def("get_as",&SerialManipulatorEDH::get_as,"");
    //    double get_a(const int& link_index) const;
    SerialManipulatorEDH.def("get_a",&SerialManipulatorEDH::get_a,"");
    //    void set_a(const int& link_index, double const& value);
    SerialManipulatorEDH.def("set_a",&SerialManipulatorEDH::set_a,"");
    //    VectorXd get_alphas() const;
    SerialManipulatorEDH.def("get_alphas",&SerialManipulatorEDH::get_alphas,"");
    //    double get_alpha(const int& link_index) const;
    SerialManipulatorEDH.def("get_alpha",&SerialManipulatorEDH::get_alpha,"");
    //    void set_alpha(const int& link_index, double const& value);
    SerialManipulatorEDH.def("set_alpha",&SerialManipulatorEDH::set_alpha,"");
    //    MatrixXd parameter_pose_jacobian(const VectorXd& joint_values, const int& to_ith_link) const;
    SerialManipulatorEDH.def("parameter_pose_jacobian",
                                     (MatrixXd (SerialManipulatorEDH::*)(const VectorXd&,const int&) const)
                                     (&SerialManipulatorEDH::parameter_pose_jacobian),
                                     "");
    //    MatrixXd parameter_pose_jacobian(const VectorXd& joint_values) const;
    SerialManipulatorEDH.def("parameter_pose_jacobian",
                                     (MatrixXd (SerialManipulatorEDH::*)(const VectorXd&) const)
                                     (&SerialManipulatorEDH::parameter_pose_jacobian),
                                     "");
    //    //Virtual methods from DQ_SerialManipulator
    //    DQ raw_fkm(const VectorXd& joint_values, const int& to_ith_link) const override;
    SerialManipulatorEDH.def("raw_fkm",&SerialManipulatorEDH::raw_fkm,"");
    //    MatrixXd raw_pose_jacobian(const VectorXd& joint_values, const int& to_ith_link) const override;
    SerialManipulatorEDH.def("raw_pose_jacobian",&SerialManipulatorEDH::raw_pose_jacobian,"");
    //    DQ fkm (const VectorXd& joint_values) const override;
    SerialManipulatorEDH.def("fkm",
                                     (DQ (SerialManipulatorEDH::*)(const VectorXd&) const)
                                     (&SerialManipulatorEDH::fkm),
                                     "");
    //    DQ fkm (const VectorXd& joint_values, const int& to_ith_link) const override;
    SerialManipulatorEDH.def("fkm",
                                     (DQ (SerialManipulatorEDH::*)(const VectorXd&,const int&) const)
                                     (&SerialManipulatorEDH::fkm),
                                     "");
    //    MatrixXd pose_jacobian(const VectorXd& joint_values, const int& to_ith_link) const override;
    SerialManipulatorEDH.def("pose_jacobian",
                                     (MatrixXd (SerialManipulatorEDH::*)(const VectorXd&,const int&) const)
                                     (&SerialManipulatorEDH::pose_jacobian),
                                     "");
    //    MatrixXd pose_jacobian (const VectorXd& joint_values) const override;
    SerialManipulatorEDH.def("pose_jacobian",
                                     (MatrixXd (SerialManipulatorEDH::*)(const VectorXd&) const)
                                     (&SerialManipulatorEDH::pose_jacobian),
                                     "");
    //    int get_dim_configuration_space() const override;
    SerialManipulatorEDH.def("get_dim_configuration_space",&SerialManipulatorEDH::get_dim_configuration_space,"");


    /// "SimulatorDummy.h"

    //class SimulatorDummy
    py::class_<SimulatorDummy, std::shared_ptr<SimulatorDummy>>(m, "M3_SimulatorDummy")
            .def(py::init<>())
            .def("get_object_pose", &SimulatorDummy::get_object_pose, "", py::arg("object_name"))
            .def("set_object_pose", &SimulatorDummy::set_object_pose, "", py::arg("object_name"), py::arg("pose"))
            .def("has_object", &SimulatorDummy::has_object, "", py::arg("object_name"))
            .def("get_object_names", &SimulatorDummy::get_object_names)
            .def("get_configuration_space_positions", &SimulatorDummy::get_configuration_space_positions)
            .def("set_configuration_space_positions", &SimulatorDummy::set_configuration_space_positions, "", py::arg("q"))
            .def("start_simulation", &SimulatorDummy::start_simulation)
            .def("stop_simulation", &SimulatorDummy::stop_simulation)
            .def("is_running", &SimulatorDummy::is_running)
            .def("load_reference_scene", &SimulatorDummy::load_reference_scene)
            .def_static("vs050_raw_kinematics", &SimulatorDummy::vs050_raw_kinematics,
                         "Return the VS050 kinematics of the TRO2022 example (ideal base/effector).");

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
