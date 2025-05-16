cmake_minimum_required(VERSION 3.15...3.26)

project(adaptive_control_example_python_wrapper LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Werror=return-type -fPIC)

if(APPLE)
    include_directories(
            /usr/local/include/
            /usr/local/include/eigen3
            /opt/homebrew/include
            /opt/homebrew/include/eigen3
    )
endif()

if(WIN32)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
    FIND_PACKAGE(Eigen3 CONFIG REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
endif()

include_directories(
    include
    /usr/include/eigen3/
    dqrobotics_python/cpp/include
    dqrobotics_python/interfaces/cpp-interface-coppeliasim/include
    submodules/qpOASES/include
    submodules/dqrobotics/cpp-interface-qpoases/include
    )

add_subdirectory(dqrobotics_python/pybind11)
pybind11_add_module(_core MODULE
    src/adaptive_control_example_py.cpp

    )

# CMAKE was not passing BUILD_SHARED_LIBS nicely to the add_subdirectory.
# https://thatonegamedev.com/cpp/cmake/how-to-manage-dependencies-with-cmake/
set(BUILD_SHARED_LIBS FALSE CACHE BOOL "x" FORCE)
add_subdirectory(cpp_version)
add_subdirectory(dqrobotics_python/cpp)
add_subdirectory(submodules/qpOASES)

target_link_libraries(_core PRIVATE adaptive_control dqrobotics qpOASES)

target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})