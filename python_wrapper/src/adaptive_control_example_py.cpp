#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "marinholab/papers/tro2022/adaptive_control/M3_AdaptiveController.h"
#include "marinholab/papers/tro2022/adaptive_control/M3_VFI.h"
#include "marinholab/papers/tro2022/adaptive_control/M3_SerialManipulatorEDH.h"
#include "marinholab/papers/tro2022/adaptive_control/M3_SimulatorDummy.h"
#include "marinholab/papers/tro2022/adaptive_control/M3_MeasurementSpace.h"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;
using namespace DQ_robotics;

PYBIND11_MODULE(_core, m) {

    /// "example/M3_MeasurementSpace.h"

    //enum class M3_MeasureSpace
    py::enum_<M3_MeasureSpace>(m, "M3_MeasureSpace")
            .value("None", M3_MeasureSpace::None)
            .value("Pose", M3_MeasureSpace::Pose)
            .value("Rotation", M3_MeasureSpace::Rotation)
            .value("Translation", M3_MeasureSpace::Translation)
            .value("Distance", M3_MeasureSpace::Distance)
            .export_values();

    /// "example/M3_AdaptiveController.h"

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
                 const M3_MeasureSpace&,
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

    //class M3_AdaptiveController
    py::class_
            <
            M3_AdaptiveController,
            std::shared_ptr<M3_AdaptiveController>
            >(m, "M3_AdaptiveController")
            .def(py::init
                 <const std::shared_ptr<M3_SerialManipulatorEDH>&,
                 const Example_SimulationParameters &
                 >())
            .def("compute_setpoint_control_signal",&M3_AdaptiveController::compute_setpoint_control_signal,"");

    /// "example/M3_AdaptiveController.h"

    //enum class M3_Primitive
    py::enum_<M3_Primitive>(m, "M3_Primitive")
            .value("None", M3_Primitive::None)
            .value("Point", M3_Primitive::Point)
            .value("Plane", M3_Primitive::Plane)
            .value("Line", M3_Primitive::Line)
            .export_values();

    //enum class M3_VFI_Direction
    py::enum_<M3_VFI_Direction>(m, "M3_VFI_Direction")
            .value("None", M3_VFI_Direction::None)
            .value("FORBIDDEN_ZONE", M3_VFI_Direction::FORBIDDEN_ZONE)
            .value("SAFE_ZONE", M3_VFI_Direction::SAFE_ZONE)
            .export_values();

    //enum class M3_VFI_DistanceType
    py::enum_<M3_VFI_DistanceType>(m, "M3_VFI_DistanceType")
            .value("None", M3_VFI_DistanceType::None)
            .value("EUCLIDEAN", M3_VFI_DistanceType::EUCLIDEAN)
            .value("EUCLIDEAN_SQUARED", M3_VFI_DistanceType::EUCLIDEAN_SQUARED)
            .export_values();

    //class M3_VFI
    py::class_<M3_VFI,std::shared_ptr<M3_VFI>>(m, "M3_VFI")
            .def(py::init
                 <
                 const std::string&,
                 const std::string&,
                 const M3_Primitive&,
                 const std::shared_ptr<M3_SimulatorDummy>&,
                 const double&,
                 const M3_VFI_Direction&,
                 const int&,
                 const DQ&,
                 const std::string&
                 >())
            .def("initialize",&M3_VFI::initialize,"")
            .def("get_value",&M3_VFI::get_value,"")
            .def("set_value",&M3_VFI::set_value,"")
            .def("get_distance_jacobian",&M3_VFI::get_distance_jacobian,"")
            .def("get_vfi_matrix",&M3_VFI::get_vfi_matrix,"")
            .def("get_distance",&M3_VFI::get_distance,"")
            .def("get_distance_error",&M3_VFI::get_distance_error,"")
            .def("get_safe_distance",&M3_VFI::get_safe_distance,"")
            .def("get_distance_type",&M3_VFI::get_distance_type,"")
            .def("set_last_real_distance",&M3_VFI::set_last_real_distance,"")
            .def("get_last_real_distance",&M3_VFI::get_last_real_distance,"")
            .def("set_last_estimated_distance",&M3_VFI::set_last_estimated_distance,"")
            .def("get_last_estimated_distance",&M3_VFI::get_last_estimated_distance,"")
            .def("get_vfi_name",&M3_VFI::get_vfi_name,"");

    /// "example/M3_SerialManipulatorEDH.h"

    //namespace M3_ParameterSpaceEDH
    py::module M3_ParameterSpaceEDH = m.def_submodule("_M3_ParameterSpaceEDH", "");

    //enum class Example_ParameterType
    py::enum_<M3_ParameterSpaceEDH::Example_ParameterType>(M3_ParameterSpaceEDH, "Example_ParameterType")
            .value("theta", M3_ParameterSpaceEDH::Example_ParameterType::theta)
            .value("d", M3_ParameterSpaceEDH::Example_ParameterType::d)
            .value("a", M3_ParameterSpaceEDH::Example_ParameterType::a)
            .value("alpha", M3_ParameterSpaceEDH::Example_ParameterType::alpha)

            .value("base_x", M3_ParameterSpaceEDH::Example_ParameterType::base_x)
            .value("base_y", M3_ParameterSpaceEDH::Example_ParameterType::base_y)
            .value("base_z", M3_ParameterSpaceEDH::Example_ParameterType::base_z)
            .value("base_alpha", M3_ParameterSpaceEDH::Example_ParameterType::base_alpha)
            .value("base_beta", M3_ParameterSpaceEDH::Example_ParameterType::base_beta)
            .value("base_gamma", M3_ParameterSpaceEDH::Example_ParameterType::base_gamma)

            .value("eff_x", M3_ParameterSpaceEDH::Example_ParameterType::eff_x)
            .value("eff_y", M3_ParameterSpaceEDH::Example_ParameterType::eff_y)
            .value("eff_z", M3_ParameterSpaceEDH::Example_ParameterType::eff_z)
            .value("eff_alpha", M3_ParameterSpaceEDH::Example_ParameterType::eff_alpha)
            .value("eff_beta", M3_ParameterSpaceEDH::Example_ParameterType::eff_beta)
            .value("eff_gamma", M3_ParameterSpaceEDH::Example_ParameterType::eff_gamma)
            .export_values();

    //struct Example_Parameter
    py::class_<M3_ParameterSpaceEDH::Example_Parameter>(M3_ParameterSpaceEDH, "Example_Parameter")
            .def(py::init
                 <
                 const int&,
                 const M3_ParameterSpaceEDH::Example_ParameterType&,
                 const double&,
                 const double&,
                 const double&
                 >())
            .def_readwrite("link_index_",&M3_ParameterSpaceEDH::Example_Parameter::link_index_)
            .def_readwrite("type_",&M3_ParameterSpaceEDH::Example_Parameter::type_)
            .def_readwrite("value_",&M3_ParameterSpaceEDH::Example_Parameter::value_)
            .def_readwrite("min_",&M3_ParameterSpaceEDH::Example_Parameter::min_)
            .def_readwrite("max_",&M3_ParameterSpaceEDH::Example_Parameter::max_);

    // This is originally wrapped in dqrobotics. Better to know what to expect in this case. Nonetheless, this was needed
    // in 2025.05 and otherwise had the error
    // ImportError: generic_type: type "M3_SerialManipulatorEDH" referenced unknown base type "DQ_robotics::DQ_SerialManipulator"
    // This was solved by me a long time ago, the DQ_SerialManipulator imported be installed beforehand in Python.
    // see test_python_wrapper.sh or look up the history if the file no longer exists.
    // However, this does not seem to always work. It might fail for the slightest of differences in versions.
    // https://pybind11.readthedocs.io/en/stable/advanced/misc.html
    // py::module_::import("dqrobotics");
    // py::module_::import("dqrobotics.robot_modeling");

    //class M3_SerialManipulatorEDH : public DQ_SerialManipulator
    py::class_
            <
            M3_SerialManipulatorEDH,
            std::shared_ptr<M3_SerialManipulatorEDH>,
            DQ_SerialManipulator
            > M3_SerialManipulatorEDH
            (
                m,
                "M3_SerialManipulatorEDH"
                );

    py::enum_<M3_SerialManipulatorEDH::JOINT_TYPES>(M3_SerialManipulatorEDH, "JOINT_TYPES")
            .value("JOINT_ROTATIONAL", M3_SerialManipulatorEDH::JOINT_TYPES::JOINT_ROTATIONAL)
            .value("JOINT_PRISMATIC", M3_SerialManipulatorEDH::JOINT_TYPES::JOINT_PRISMATIC)
            .export_values();

    M3_SerialManipulatorEDH.def(py::init
                                     <
                                     const MatrixXd&
                                     >());

    //    DQ get_base_frame() const;
    M3_SerialManipulatorEDH.def("get_base_frame",&M3_SerialManipulatorEDH::get_base_frame,"");
    //    std::vector<M3_ParameterSpaceEDH::Example_Parameter> get_base_parameters() const;
    M3_SerialManipulatorEDH.def("get_base_parameters",&M3_SerialManipulatorEDH::get_base_parameters,"");
    //    void set_base_frame(const std::vector<M3_ParameterSpaceEDH::Example_Parameter> &base_parameters);
    M3_SerialManipulatorEDH.def("set_base_frame",
                                     (void (M3_SerialManipulatorEDH::*)(const std::vector<M3_ParameterSpaceEDH::Example_Parameter>&))
                                     (&M3_SerialManipulatorEDH::set_base_frame),
                                     "");
    //    void set_base_frame(const DQ& base);
    M3_SerialManipulatorEDH.def("set_base_frame",
                                     (void (M3_SerialManipulatorEDH::*)(const DQ&))
                                     (&M3_SerialManipulatorEDH::set_base_frame),
                                     "");
    //    DQ get_effector_frame() const;
    M3_SerialManipulatorEDH.def("get_effector_frame",&M3_SerialManipulatorEDH::get_effector_frame,"");
    //    std::vector<M3_ParameterSpaceEDH::Example_Parameter> get_effector_parameters() const;
    M3_SerialManipulatorEDH.def("get_effector_parameters",&M3_SerialManipulatorEDH::get_effector_parameters,"");
    //    void set_effector_frame(const std::vector<M3_ParameterSpaceEDH::Example_Parameter>& effector_parameters);
    M3_SerialManipulatorEDH.def("set_effector_frame",
                                     (void (M3_SerialManipulatorEDH::*)(const std::vector<M3_ParameterSpaceEDH::Example_Parameter>&))
                                     (&M3_SerialManipulatorEDH::set_effector_frame),
                                     "");
    //    void set_effector_frame(const DQ& effector);
    M3_SerialManipulatorEDH.def("set_effector_frame",
                                     (void (M3_SerialManipulatorEDH::*)(const DQ&))
                                     (&M3_SerialManipulatorEDH::set_effector_frame),
                                     "");
    //    void set_parameter_space(const std::vector<M3_ParameterSpaceEDH::Example_Parameter>& parameter_space);
    M3_SerialManipulatorEDH.def("set_parameter_space",&M3_SerialManipulatorEDH::set_parameter_space,"");
    //    bool is_parameter_space_set() const;
    M3_SerialManipulatorEDH.def("is_parameter_space_set",&M3_SerialManipulatorEDH::is_parameter_space_set,"");
    //    int get_dim_parameter_space() const;
    M3_SerialManipulatorEDH.def("get_dim_parameter_space",&M3_SerialManipulatorEDH::get_dim_parameter_space,"");
    //    VectorXd get_parameter_space_values() const;
    M3_SerialManipulatorEDH.def("get_parameter_space_values",&M3_SerialManipulatorEDH::get_parameter_space_values,"");
    //    void set_parameter_space_values(const VectorXd& parameter_space_vector);
    M3_SerialManipulatorEDH.def("set_parameter_space_values",&M3_SerialManipulatorEDH::set_parameter_space_values,"");
    //    void set_parameter_space_boundaries(const std::tuple<VectorXd, VectorXd>& boundaries);
    M3_SerialManipulatorEDH.def("set_parameter_space_boundaries",&M3_SerialManipulatorEDH::set_parameter_space_boundaries,"");
    //    std::tuple<VectorXd,VectorXd> get_parameter_space_boundaries() const;
    M3_SerialManipulatorEDH.def("get_parameter_space_boundaries",&M3_SerialManipulatorEDH::get_parameter_space_boundaries,"");
    //    std::vector<M3_ParameterSpaceEDH::Example_ParameterType> get_parameter_types() const;
    M3_SerialManipulatorEDH.def("get_parameter_types",&M3_SerialManipulatorEDH::get_parameter_types,"");
    //    VectorXd get_link_types() const;
    M3_SerialManipulatorEDH.def("get_link_types",&M3_SerialManipulatorEDH::get_link_types,"");
    //    double get_link_type(const int& link_index) const;
    M3_SerialManipulatorEDH.def("get_link_type",&M3_SerialManipulatorEDH::get_link_type,"");
    //    VectorXd get_thetas() const;
    M3_SerialManipulatorEDH.def("get_thetas",&M3_SerialManipulatorEDH::get_thetas,"");
    //    double get_theta(const int& link_index) const;
    M3_SerialManipulatorEDH.def("get_theta",&M3_SerialManipulatorEDH::get_theta,"");
    //    void set_theta(const int& link_index, double const& value);
    M3_SerialManipulatorEDH.def("set_theta",&M3_SerialManipulatorEDH::set_theta,"");
    //    VectorXd get_ds() const;
    M3_SerialManipulatorEDH.def("get_ds",&M3_SerialManipulatorEDH::get_ds,"");
    //    double get_d(const int& link_index) const;
    M3_SerialManipulatorEDH.def("get_d",&M3_SerialManipulatorEDH::get_d,"");
    //    void set_d(const int& link_index, double const& value);
    M3_SerialManipulatorEDH.def("set_d",&M3_SerialManipulatorEDH::set_d,"");
    //    VectorXd get_as() const;
    M3_SerialManipulatorEDH.def("get_as",&M3_SerialManipulatorEDH::get_as,"");
    //    double get_a(const int& link_index) const;
    M3_SerialManipulatorEDH.def("get_a",&M3_SerialManipulatorEDH::get_a,"");
    //    void set_a(const int& link_index, double const& value);
    M3_SerialManipulatorEDH.def("set_a",&M3_SerialManipulatorEDH::set_a,"");
    //    VectorXd get_alphas() const;
    M3_SerialManipulatorEDH.def("get_alphas",&M3_SerialManipulatorEDH::get_alphas,"");
    //    double get_alpha(const int& link_index) const;
    M3_SerialManipulatorEDH.def("get_alpha",&M3_SerialManipulatorEDH::get_alpha,"");
    //    void set_alpha(const int& link_index, double const& value);
    M3_SerialManipulatorEDH.def("set_alpha",&M3_SerialManipulatorEDH::set_alpha,"");
    //    MatrixXd parameter_pose_jacobian(const VectorXd& joint_values, const int& to_ith_link) const;
    M3_SerialManipulatorEDH.def("parameter_pose_jacobian",
                                     (MatrixXd (M3_SerialManipulatorEDH::*)(const VectorXd&,const int&) const)
                                     (&M3_SerialManipulatorEDH::parameter_pose_jacobian),
                                     "");
    //    MatrixXd parameter_pose_jacobian(const VectorXd& joint_values) const;
    M3_SerialManipulatorEDH.def("parameter_pose_jacobian",
                                     (MatrixXd (M3_SerialManipulatorEDH::*)(const VectorXd&) const)
                                     (&M3_SerialManipulatorEDH::parameter_pose_jacobian),
                                     "");
    //    //Virtual methods from DQ_SerialManipulator
    //    DQ raw_fkm(const VectorXd& joint_values, const int& to_ith_link) const override;
    M3_SerialManipulatorEDH.def("raw_fkm",&M3_SerialManipulatorEDH::raw_fkm,"");
    //    MatrixXd raw_pose_jacobian(const VectorXd& joint_values, const int& to_ith_link) const override;
    M3_SerialManipulatorEDH.def("raw_pose_jacobian",&M3_SerialManipulatorEDH::raw_pose_jacobian,"");
    //    DQ fkm (const VectorXd& joint_values) const override;
    M3_SerialManipulatorEDH.def("fkm",
                                     (DQ (M3_SerialManipulatorEDH::*)(const VectorXd&) const)
                                     (&M3_SerialManipulatorEDH::fkm),
                                     "");
    //    DQ fkm (const VectorXd& joint_values, const int& to_ith_link) const override;
    M3_SerialManipulatorEDH.def("fkm",
                                     (DQ (M3_SerialManipulatorEDH::*)(const VectorXd&,const int&) const)
                                     (&M3_SerialManipulatorEDH::fkm),
                                     "");
    //    MatrixXd pose_jacobian(const VectorXd& joint_values, const int& to_ith_link) const override;
    M3_SerialManipulatorEDH.def("pose_jacobian",
                                     (MatrixXd (M3_SerialManipulatorEDH::*)(const VectorXd&,const int&) const)
                                     (&M3_SerialManipulatorEDH::pose_jacobian),
                                     "");
    //    MatrixXd pose_jacobian (const VectorXd& joint_values) const override;
    M3_SerialManipulatorEDH.def("pose_jacobian",
                                     (MatrixXd (M3_SerialManipulatorEDH::*)(const VectorXd&) const)
                                     (&M3_SerialManipulatorEDH::pose_jacobian),
                                     "");
    //    int get_dim_configuration_space() const override;
    M3_SerialManipulatorEDH.def("get_dim_configuration_space",&M3_SerialManipulatorEDH::get_dim_configuration_space,"");


    /// "M3_SimulatorDummy.h"

    //class M3_SimulatorDummy
    py::class_<M3_SimulatorDummy, std::shared_ptr<M3_SimulatorDummy>>(m, "M3_SimulatorDummy")
            .def(py::init<>())
            .def("get_object_pose", &M3_SimulatorDummy::get_object_pose, "", py::arg("object_name"))
            .def("set_object_pose", &M3_SimulatorDummy::set_object_pose, "", py::arg("object_name"), py::arg("pose"))
            .def("has_object", &M3_SimulatorDummy::has_object, "", py::arg("object_name"))
            .def("get_object_names", &M3_SimulatorDummy::get_object_names)
            .def("get_configuration_space_positions", &M3_SimulatorDummy::get_configuration_space_positions)
            .def("set_configuration_space_positions", &M3_SimulatorDummy::set_configuration_space_positions, "", py::arg("q"))
            .def("start_simulation", &M3_SimulatorDummy::start_simulation)
            .def("stop_simulation", &M3_SimulatorDummy::stop_simulation)
            .def("is_running", &M3_SimulatorDummy::is_running)
            .def("load_reference_scene", &M3_SimulatorDummy::load_reference_scene)
            .def_static("vs050_raw_kinematics", &M3_SimulatorDummy::vs050_raw_kinematics,
                         "Return the VS050 kinematics of the TRO2022 example (ideal base/effector).");

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
