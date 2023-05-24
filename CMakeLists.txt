cmake_minimum_required(VERSION 3.5)

project(adaptive_control_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Werror=return-type)

include_directories(
    include
    submodules/sas_core/include
    )

add_executable(${PROJECT_NAME}
    src/${PROJECT_NAME}.cpp
    src/robot_control/Example_AdaptiveController.cpp
    src/constraints_modeling/Example_VFI.cpp
    src/robot_modeling/Example_SerialManipulatorEDH.cpp
    src/interfaces/vrep/robots/Example_VS050VrepRobot.cpp

    submodules/sas_core/src/sas_object.cpp
    submodules/sas_core/src/sas_clock.cpp
    )


target_link_libraries(${PROJECT_NAME}
    qpOASES

    dqrobotics
    dqrobotics-interface-vrep

    )

