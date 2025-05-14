cmake_minimum_required(VERSION 3.15...3.26)

project(adaptive_control_example_python_wrapper LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Werror=return-type)

if(APPLE)
    find_package(cppzmq)
    message("Building not supported on this platform.")
    include_directories(
            /usr/local/include/
            /usr/local/include/eigen3
            /opt/homebrew/include
            /opt/homebrew/include/eigen3
    )
endif()

if(WIN32)
    message("Building not supported on this platform.")

    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
    FIND_PACKAGE(Eigen3 CONFIG REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
endif()

include_directories(
    include
    submodules/dqrobotics/cpp/include
    submodules/dqrobotics/cpp-interface-coppeliasim/include
    submodules/dqrobotics/cpp-interface-coppeliasim-zmq/include
    submodules/dqrobotics/cpp-interface-qpoases/include
    submodules/sas_core/include
    )

add_subdirectory(pybind11)
add_subdirectory(submodules/dqrobotics/cpp)
add_subdirectory(submodules/dqrobotics/cpp-interface-coppeliasim)
add_subdirectory(submodules/dqrobotics/cpp-interface-coppeliasim-zmq)
add_subdirectory(submodules/qpOASES)

pybind11_add_module(_core MODULE

    src/adaptive_control_example_py.cpp

    src/example/Example_AdaptiveController.cpp
    src/example/Example_VFI.cpp
    src/example/Example_SerialManipulatorEDH.cpp
    #../src/example/Example_VS050VrepRobot.cpp
    src/example/Example_MeasurementSpace.cpp

    submodules/sas_core/src/sas_object.cpp
    submodules/sas_core/src/sas_clock.cpp

    )

if(APPLE)
    target_link_libraries(_core PRIVATE dqrobotics dqrobotics-interface-coppeliasim-zmq cppzmq qpOASES ${Boost_LIBRARIES})
endif()
if(UNIX AND NOT APPLE)
    target_link_libraries(_core PRIVATE dqrobotics dqrobotics-interface-coppeliasim-zmq zmq qpOASES ${Boost_LIBRARIES})
endif()

target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})