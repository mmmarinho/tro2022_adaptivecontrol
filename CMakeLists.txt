# (C) Copyright 2023-2026 Murilo Marques Marinho (www.murilomarinho.info)
#
# This file is part of adaptive_control_example.
#
# SPDX-License-Identifier: MIT
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Single CMake project for the TRO2022 adaptive-control example.
# It builds:
#   1. `adaptive_control`      - static library with the example's C++ logic
#   2. `adaptive_control_cpp`  - standalone (headless) example executable
#   3. `_core`                 - pybind11 module consumed by the Python package
#
# Building the Python wheel from the repository root (`pip install .`)
# drives this same project through setup.py, so there is one CMake
# configuration for both the C++ and the Python artifacts. The Python
# module is gated behind BUILD_PYTHON_WRAPPER so a pure C++ build (e.g.
# .build.sh) does not require a Python interpreter.
cmake_minimum_required(VERSION 3.15...3.29)

project(adaptive_control_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Werror=return-type)

# The pybind11 `_core` module is a shared object that links the static
# `adaptive_control` library (and the static dqrobotics/qpOASES libraries),
# so every object must be position-independent. Setting this here (before any
# target is created, including in the add_subdirectory calls below) forces
# PIC across the whole build.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Threads REQUIRED)

add_library(adaptive_control
        src/example/M3_AdaptiveController.cpp
        src/example/M3_VFI.cpp
        src/example/M3_SerialManipulatorEDH.cpp
        src/example/M3_SimulatorDummy.cpp
        src/example/M3_MeasurementSpace.cpp

        # sas::Clock (timing statistics) depends on sas::Object internally,
        # so both translation units are needed.
        submodules/sas_core/src/sas_clock.cpp
        submodules/sas_core/src/sas_object.cpp
)

# Eigen does not ship a CMake config file, so find_package(Eigen3) is not
# reliable (in particular with the macOS homebrew install, which places the
# headers under /opt/homebrew/include/eigen3). sas_core.hpp includes
# <eigen3/Eigen/Dense>, so the include path must contain the *parent*
# directory (/opt/homebrew/include), not just the eigen3 subdir. These are
# added as directory-scope includes so they apply to every target built here
# (the adaptive_control library, the executable, and the _core module),
# mirroring the original python_wrapper/CMakeLists.txt.
if(APPLE)
    include_directories(
        /usr/local/include
        /usr/local/include/eigen3
        # Most recent versions of brew install here
        /opt/homebrew/include
        /opt/homebrew/include/eigen3
    )
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
# https://thatonegamedev.com/cpp/cmake/how-to/manage-dependencies-with-cmake/
set(BUILD_SHARED_LIBS FALSE CACHE BOOL "x" FORCE)
add_subdirectory(submodules/dqrobotics/python/cpp)
# The example only needs the qpOASES solver library, not its demo apps.
set(QPOASES_BUILD_EXAMPLES OFF CACHE BOOL "x" FORCE)
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

# ----------------------------------------------------------------------
# Python extension (pybind11). setup.py's CMakeExtension
# ('marinholab.papers.tro2022.adaptive_control._core') points at the
# repository root, so this `_core` target is what lands inside the
# installed package. The default pybind11 suffix keeps the name
# `_core.cpython-XY-ARCH.so`, which the package __init__.py imports as
# `marinholab.papers.tro2022.adaptive_control._core`.
# ----------------------------------------------------------------------
option(BUILD_PYTHON_WRAPPER "Build the pybind11 Python module" ON)
if(BUILD_PYTHON_WRAPPER)
    # PYBIND11_FINDPYTHON makes pybind11 pick up the interpreter that
    # setup.py passes in (PYTHON_EXECUTABLE) when the wheel is built.
    set(PYBIND11_FINDPYTHON ON)
    add_subdirectory(submodules/dqrobotics/python/pybind11)

    pybind11_add_module(_core MODULE
        src/adaptive_control_example_py.cpp
        )

    target_link_libraries(_core PRIVATE adaptive_control)
    set_target_properties(_core PROPERTIES PREFIX "")
endif()

install(TARGETS
    ${PROJECT_NAME}
    DESTINATION ${CMAKE_SOURCE_DIR}/bin/)
