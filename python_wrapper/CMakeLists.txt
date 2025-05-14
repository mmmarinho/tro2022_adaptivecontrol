cmake_minimum_required(VERSION 3.15...3.26)

project(adaptive_control_example_python_wrapper LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Werror=return-type)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

if(APPLE)
    INCLUDE_DIRECTORIES(
            /usr/local/include/
            /usr/local/include/eigen3
            # Most recent versions of brew install here
            /opt/homebrew/include
            /opt/homebrew/include/eigen3)
endif()

include_directories(
    ../include
    ../submodules/sas_core/include
    )

add_subdirectory(pybind11)
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
    
target_link_libraries(_core PRIVATE dqrobotics dqrobotics-interface-coppeliasim dqrobotics-interface-coppeliasim-zmq qpOASES)
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

install(TARGETS _core DESTINATION .)


