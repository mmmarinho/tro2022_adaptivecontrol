cmake_minimum_required(VERSION 3.15...3.26)

project(adaptive_control_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Werror=return-type)

find_package(Threads REQUIRED)

add_library(adaptive_control
        src/example/M3_AdaptiveController.cpp
        src/example/M3_VFI.cpp
        src/example/M3_SerialManipulatorEDH.cpp
        src/example/M3_SimulatorDummy.cpp
        src/example/M3_MeasurementSpace.cpp

        submodules/sas_core/src/sas_object.cpp
        submodules/sas_core/src/sas_clock.cpp
)

if(APPLE)
    target_include_directories(adaptive_control PRIVATE
        /usr/local/include/
        /usr/local/include/eigen3
        # Most recent versions of brew install here
        /opt/homebrew/include
        /opt/homebrew/include/eigen3)
endif()
if(WIN32)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
    FIND_PACKAGE(Eigen3 CONFIG REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
endif()

target_include_directories(adaptive_control PUBLIC
    include
    submodules/qpOASES/include
    submodules/sas_core/include
    submodules/dqrobotics/python/cpp/include
    submodules/dqrobotics/cpp-interface-qpoases/include
    )

# CMAKE was not passing BUILD_SHARED_LIBS nicely to the add_subdirectory.
# https://thatonegamedev.com/cpp/cmake/how-to-manage-dependencies-with-cmake/
set(BUILD_SHARED_LIBS FALSE CACHE BOOL "x" FORCE)
add_subdirectory(submodules/dqrobotics/python/cpp)
add_subdirectory(submodules/qpOASES)

target_link_libraries(adaptive_control PUBLIC dqrobotics qpOASES)

# Stand-alone example. Runs headless on the in-memory M3_SimulatorDummy,
# so it no longer requires any external simulator.
add_executable(${PROJECT_NAME}
    src/adaptive_control_example.cpp
    )

target_link_libraries(${PROJECT_NAME}
    adaptive_control
    dqrobotics
    qpOASES
    Threads::Threads
    )

install(TARGETS
    ${PROJECT_NAME}
    DESTINATION ${CMAKE_SOURCE_DIR}/bin/)
    
