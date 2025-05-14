#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "example/Example_AdaptiveController.h"
#include "example/Example_VFI.h"
#include "example/Example_SerialManipulatorEDH.h"
//#include "example/Example_VS050VrepRobot.h"
#include "example/Example_MeasurementSpace.h"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;
using namespace DQ_robotics;

PYBIND11_MODULE(_core, m) {

    /// "example/Example_MeasurementSpace.h"

    //enum class Example_MeasureSpace
    py::enum_<Example_MeasureSpace>(m, "Example_MeasureSpace")
            .value("None", Example_MeasureSpace::None)
            .value("Pose", Example_MeasureSpace::Pose)
            .value("Rotation", Example_MeasureSpace::Rotation)
            .value("Translation", Example_MeasureSpace::Translation)
            .value("Distance", Example_MeasureSpace::Distance)
            .export_values();

    /// "example/Example_AdaptiveController.h"

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
                 const Example_MeasureSpace&,
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

    //class Example_AdaptiveController
    py::class_
            <
            Example_AdaptiveController,
            std::shared_ptr<Example_AdaptiveController>
            >(m, "Example_AdaptiveController")
            .def(py::init
                 <const std::shared_ptr<Example_SerialManipulatorEDH>&,
                 const Example_SimulationParameters &
                 >())
            .def("compute_setpoint_control_signal",&Example_AdaptiveController::compute_setpoint_control_signal,"");

    /// "example/Example_AdaptiveController.h"

    //enum class Example_Primitive
    py::enum_<Example_Primitive>(m, "Example_Primitive")
            .value("None", Example_Primitive::None)
            .value("Point", Example_Primitive::Point)
            .value("Plane", Example_Primitive::Plane)
            .value("Line", Example_Primitive::Line)
            .export_values();

    //enum class Example_VFI_Direction
    py::enum_<Example_VFI_Direction>(m, "Example_VFI_Direction")
            .value("None", Example_VFI_Direction::None)
            .value("FORBIDDEN_ZONE", Example_VFI_Direction::FORBIDDEN_ZONE)
            .value("SAFE_ZONE", Example_VFI_Direction::SAFE_ZONE)
            .export_values();

    //enum class Example_VFI_DistanceType
    py::enum_<Example_VFI_DistanceType>(m, "Example_VFI_DistanceType")
            .value("None", Example_VFI_DistanceType::None)
            .value("EUCLIDEAN", Example_VFI_DistanceType::EUCLIDEAN)
            .value("EUCLIDEAN_SQUARED", Example_VFI_DistanceType::EUCLIDEAN_SQUARED)
            .export_values();

    //class Example_VFI
    py::class_<Example_VFI,std::shared_ptr<Example_VFI>>(m, "Example_VFI")
            .def(py::init
                 <
                 const std::string&,
                 const std::string&,
                 const Example_Primitive&,
                 const std::shared_ptr<DQ_CoppeliaSimInterface>&,
                 const double&,
                 const Example_VFI_Direction&,
                 const int&,
                 const DQ&,
                 const std::string&
                 >())
            .def("initialize",&Example_VFI::initialize,"")
            .def("get_value",&Example_VFI::get_value,"")
            .def("set_value",&Example_VFI::set_value,"")
            .def("get_distance_jacobian",&Example_VFI::get_distance_jacobian,"")
            .def("get_vfi_matrix",&Example_VFI::get_vfi_matrix,"")
            .def("get_distance",&Example_VFI::get_distance,"")
            .def("get_distance_error",&Example_VFI::get_distance_error,"")
            .def("get_safe_distance",&Example_VFI::get_safe_distance,"")
            .def("get_distance_type",&Example_VFI::get_distance_type,"")
            .def("set_last_real_distance",&Example_VFI::set_last_real_distance,"")
            .def("get_last_real_distance",&Example_VFI::get_last_real_distance,"")
            .def("set_last_estimated_distance",&Example_VFI::set_last_estimated_distance,"")
            .def("get_last_estimated_distance",&Example_VFI::get_last_estimated_distance,"")
            .def("get_vfi_name",&Example_VFI::get_vfi_name,"");

    /// "example/Example_SerialManipulatorEDH.h"

    //namespace Example_ParameterSpaceEDH
    py::module Example_ParameterSpaceEDH = m.def_submodule("_Example_ParameterSpaceEDH", "");

    //enum class Example_ParameterType
    py::enum_<Example_ParameterSpaceEDH::Example_ParameterType>(Example_ParameterSpaceEDH, "Example_ParameterType")
            .value("theta", Example_ParameterSpaceEDH::Example_ParameterType::theta)
            .value("d", Example_ParameterSpaceEDH::Example_ParameterType::d)
            .value("a", Example_ParameterSpaceEDH::Example_ParameterType::a)
            .value("alpha", Example_ParameterSpaceEDH::Example_ParameterType::alpha)

            .value("base_x", Example_ParameterSpaceEDH::Example_ParameterType::base_x)
            .value("base_y", Example_ParameterSpaceEDH::Example_ParameterType::base_y)
            .value("base_z", Example_ParameterSpaceEDH::Example_ParameterType::base_z)
            .value("base_alpha", Example_ParameterSpaceEDH::Example_ParameterType::base_alpha)
            .value("base_beta", Example_ParameterSpaceEDH::Example_ParameterType::base_beta)
            .value("base_gamma", Example_ParameterSpaceEDH::Example_ParameterType::base_gamma)

            .value("eff_x", Example_ParameterSpaceEDH::Example_ParameterType::eff_x)
            .value("eff_y", Example_ParameterSpaceEDH::Example_ParameterType::eff_y)
            .value("eff_z", Example_ParameterSpaceEDH::Example_ParameterType::eff_z)
            .value("eff_alpha", Example_ParameterSpaceEDH::Example_ParameterType::eff_alpha)
            .value("eff_beta", Example_ParameterSpaceEDH::Example_ParameterType::eff_beta)
            .value("eff_gamma", Example_ParameterSpaceEDH::Example_ParameterType::eff_gamma)
            .export_values();

    //struct Example_Parameter
    py::class_<Example_ParameterSpaceEDH::Example_Parameter>(Example_ParameterSpaceEDH, "Example_Parameter")
            .def(py::init
                 <
                 const int&,
                 const Example_ParameterSpaceEDH::Example_ParameterType&,
                 const double&,
                 const double&,
                 const double&
                 >())
            .def_readwrite("link_index_",&Example_ParameterSpaceEDH::Example_Parameter::link_index_)
            .def_readwrite("type_",&Example_ParameterSpaceEDH::Example_Parameter::type_)
            .def_readwrite("value_",&Example_ParameterSpaceEDH::Example_Parameter::value_)
            .def_readwrite("min_",&Example_ParameterSpaceEDH::Example_Parameter::min_)
            .def_readwrite("max_",&Example_ParameterSpaceEDH::Example_Parameter::max_);

    // This is originally wrapped in dqrobotics. Better to know what to expect in this case. Nonetheless, this was needed
    // in 2025.05 and otherwise had the error
    // ImportError: generic_type: type "Example_SerialManipulatorEDH" referenced unknown base type "DQ_robotics::DQ_SerialManipulator"
    // This was solved by me a long time ago, the DQ_SerialManipulator imported be installed beforehand in Python.
    // see test_python_wrapper.sh or look up the history if the file no longer exists.
    // However, this does not seem to always work. It might fail for the slightest of differences in versions.
    py::class_<DQ_SerialManipulator,std::shared_ptr<DQ_SerialManipulator>>(m,"DQ_SerialManipulator");

    //class Example_SerialManipulatorEDH : public DQ_SerialManipulator
    py::class_
            <
            Example_SerialManipulatorEDH,
            std::shared_ptr<Example_SerialManipulatorEDH>,
            DQ_SerialManipulator
            > example_serialmanipulatoredh
            (
                m,
                "Example_SerialManipulatorEDH"
                );

    py::enum_<Example_SerialManipulatorEDH::JOINT_TYPES>(example_serialmanipulatoredh, "JOINT_TYPES")
            .value("JOINT_ROTATIONAL", Example_SerialManipulatorEDH::JOINT_TYPES::JOINT_ROTATIONAL)
            .value("JOINT_PRISMATIC", Example_SerialManipulatorEDH::JOINT_TYPES::JOINT_PRISMATIC)
            .export_values();

    example_serialmanipulatoredh.def(py::init
                                     <
                                     const MatrixXd&
                                     >());

    //    DQ get_base_frame() const;
    example_serialmanipulatoredh.def("get_base_frame",&Example_SerialManipulatorEDH::get_base_frame,"");
    //    std::vector<Example_ParameterSpaceEDH::Example_Parameter> get_base_parameters() const;
    example_serialmanipulatoredh.def("get_base_parameters",&Example_SerialManipulatorEDH::get_base_parameters,"");
    //    void set_base_frame(const std::vector<Example_ParameterSpaceEDH::Example_Parameter> &base_parameters);
    example_serialmanipulatoredh.def("set_base_frame",
                                     (void (Example_SerialManipulatorEDH::*)(const std::vector<Example_ParameterSpaceEDH::Example_Parameter>&))
                                     (&Example_SerialManipulatorEDH::set_base_frame),
                                     "");
    //    void set_base_frame(const DQ& base);
    example_serialmanipulatoredh.def("set_base_frame",
                                     (void (Example_SerialManipulatorEDH::*)(const DQ&))
                                     (&Example_SerialManipulatorEDH::set_base_frame),
                                     "");
    //    DQ get_effector_frame() const;
    example_serialmanipulatoredh.def("get_effector_frame",&Example_SerialManipulatorEDH::get_effector_frame,"");
    //    std::vector<Example_ParameterSpaceEDH::Example_Parameter> get_effector_parameters() const;
    example_serialmanipulatoredh.def("get_effector_parameters",&Example_SerialManipulatorEDH::get_effector_parameters,"");
    //    void set_effector_frame(const std::vector<Example_ParameterSpaceEDH::Example_Parameter>& effector_parameters);
    example_serialmanipulatoredh.def("set_effector_frame",
                                     (void (Example_SerialManipulatorEDH::*)(const std::vector<Example_ParameterSpaceEDH::Example_Parameter>&))
                                     (&Example_SerialManipulatorEDH::set_effector_frame),
                                     "");
    //    void set_effector_frame(const DQ& effector);
    example_serialmanipulatoredh.def("set_effector_frame",
                                     (void (Example_SerialManipulatorEDH::*)(const DQ&))
                                     (&Example_SerialManipulatorEDH::set_effector_frame),
                                     "");
    //    void set_parameter_space(const std::vector<Example_ParameterSpaceEDH::Example_Parameter>& parameter_space);
    example_serialmanipulatoredh.def("set_parameter_space",&Example_SerialManipulatorEDH::set_parameter_space,"");
    //    bool is_parameter_space_set() const;
    example_serialmanipulatoredh.def("is_parameter_space_set",&Example_SerialManipulatorEDH::is_parameter_space_set,"");
    //    int get_dim_parameter_space() const;
    example_serialmanipulatoredh.def("get_dim_parameter_space",&Example_SerialManipulatorEDH::get_dim_parameter_space,"");
    //    VectorXd get_parameter_space_values() const;
    example_serialmanipulatoredh.def("get_parameter_space_values",&Example_SerialManipulatorEDH::get_parameter_space_values,"");
    //    void set_parameter_space_values(const VectorXd& parameter_space_vector);
    example_serialmanipulatoredh.def("set_parameter_space_values",&Example_SerialManipulatorEDH::set_parameter_space_values,"");
    //    void set_parameter_space_boundaries(const std::tuple<VectorXd, VectorXd>& boundaries);
    example_serialmanipulatoredh.def("set_parameter_space_boundaries",&Example_SerialManipulatorEDH::set_parameter_space_boundaries,"");
    //    std::tuple<VectorXd,VectorXd> get_parameter_space_boundaries() const;
    example_serialmanipulatoredh.def("get_parameter_space_boundaries",&Example_SerialManipulatorEDH::get_parameter_space_boundaries,"");
    //    std::vector<Example_ParameterSpaceEDH::Example_ParameterType> get_parameter_types() const;
    example_serialmanipulatoredh.def("get_parameter_types",&Example_SerialManipulatorEDH::get_parameter_types,"");
    //    VectorXd get_link_types() const;
    example_serialmanipulatoredh.def("get_link_types",&Example_SerialManipulatorEDH::get_link_types,"");
    //    double get_link_type(const int& link_index) const;
    example_serialmanipulatoredh.def("get_link_type",&Example_SerialManipulatorEDH::get_link_type,"");
    //    VectorXd get_thetas() const;
    example_serialmanipulatoredh.def("get_thetas",&Example_SerialManipulatorEDH::get_thetas,"");
    //    double get_theta(const int& link_index) const;
    example_serialmanipulatoredh.def("get_theta",&Example_SerialManipulatorEDH::get_theta,"");
    //    void set_theta(const int& link_index, double const& value);
    example_serialmanipulatoredh.def("set_theta",&Example_SerialManipulatorEDH::set_theta,"");
    //    VectorXd get_ds() const;
    example_serialmanipulatoredh.def("get_ds",&Example_SerialManipulatorEDH::get_ds,"");
    //    double get_d(const int& link_index) const;
    example_serialmanipulatoredh.def("get_d",&Example_SerialManipulatorEDH::get_d,"");
    //    void set_d(const int& link_index, double const& value);
    example_serialmanipulatoredh.def("set_d",&Example_SerialManipulatorEDH::set_d,"");
    //    VectorXd get_as() const;
    example_serialmanipulatoredh.def("get_as",&Example_SerialManipulatorEDH::get_as,"");
    //    double get_a(const int& link_index) const;
    example_serialmanipulatoredh.def("get_a",&Example_SerialManipulatorEDH::get_a,"");
    //    void set_a(const int& link_index, double const& value);
    example_serialmanipulatoredh.def("set_a",&Example_SerialManipulatorEDH::set_a,"");
    //    VectorXd get_alphas() const;
    example_serialmanipulatoredh.def("get_alphas",&Example_SerialManipulatorEDH::get_alphas,"");
    //    double get_alpha(const int& link_index) const;
    example_serialmanipulatoredh.def("get_alpha",&Example_SerialManipulatorEDH::get_alpha,"");
    //    void set_alpha(const int& link_index, double const& value);
    example_serialmanipulatoredh.def("set_alpha",&Example_SerialManipulatorEDH::set_alpha,"");
    //    MatrixXd parameter_pose_jacobian(const VectorXd& joint_values, const int& to_ith_link) const;
    example_serialmanipulatoredh.def("parameter_pose_jacobian",
                                     (MatrixXd (Example_SerialManipulatorEDH::*)(const VectorXd&,const int&) const)
                                     (&Example_SerialManipulatorEDH::parameter_pose_jacobian),
                                     "");
    //    MatrixXd parameter_pose_jacobian(const VectorXd& joint_values) const;
    example_serialmanipulatoredh.def("parameter_pose_jacobian",
                                     (MatrixXd (Example_SerialManipulatorEDH::*)(const VectorXd&) const)
                                     (&Example_SerialManipulatorEDH::parameter_pose_jacobian),
                                     "");
    //    //Virtual methods from DQ_SerialManipulator
    //    DQ raw_fkm(const VectorXd& joint_values, const int& to_ith_link) const override;
    example_serialmanipulatoredh.def("raw_fkm",&Example_SerialManipulatorEDH::raw_fkm,"");
    //    MatrixXd raw_pose_jacobian(const VectorXd& joint_values, const int& to_ith_link) const override;
    example_serialmanipulatoredh.def("raw_pose_jacobian",&Example_SerialManipulatorEDH::raw_pose_jacobian,"");
    //    DQ fkm (const VectorXd& joint_values) const override;
    example_serialmanipulatoredh.def("fkm",
                                     (DQ (Example_SerialManipulatorEDH::*)(const VectorXd&) const)
                                     (&Example_SerialManipulatorEDH::fkm),
                                     "");
    //    DQ fkm (const VectorXd& joint_values, const int& to_ith_link) const override;
    example_serialmanipulatoredh.def("fkm",
                                     (DQ (Example_SerialManipulatorEDH::*)(const VectorXd&,const int&) const)
                                     (&Example_SerialManipulatorEDH::fkm),
                                     "");
    //    MatrixXd pose_jacobian(const VectorXd& joint_values, const int& to_ith_link) const override;
    example_serialmanipulatoredh.def("pose_jacobian",
                                     (MatrixXd (Example_SerialManipulatorEDH::*)(const VectorXd&,const int&) const)
                                     (&Example_SerialManipulatorEDH::pose_jacobian),
                                     "");
    //    MatrixXd pose_jacobian (const VectorXd& joint_values) const override;
    example_serialmanipulatoredh.def("pose_jacobian",
                                     (MatrixXd (Example_SerialManipulatorEDH::*)(const VectorXd&) const)
                                     (&Example_SerialManipulatorEDH::pose_jacobian),
                                     "");
    //    int get_dim_configuration_space() const override;
    example_serialmanipulatoredh.def("get_dim_configuration_space",&Example_SerialManipulatorEDH::get_dim_configuration_space,"");


    /// "example/Example_VS050VrepRobot.h"

    //class Example_VS050VrepRobot: public DQ_SerialVrepRobot
    // py::class_
    //         <
    //         Example_VS050VrepRobot,
    //         std::shared_ptr<Example_VS050VrepRobot>,
    //         DQ_SerialVrepRobot
    //         >(m, "Example_VS050VrepRobot")
    //         .def(py::init
    //              <
    //              const std::string&,
    //              const std::shared_ptr<DQ_VrepInterface>&
    //              >())
    //         .def_static("raw_kinematics",&Example_VS050VrepRobot::raw_kinematics,"")
    //         .def("get_base_frame",&Example_VS050VrepRobot::get_base_frame,"")
    //         .def("set_base_frame",&Example_VS050VrepRobot::set_base_frame,"");

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
