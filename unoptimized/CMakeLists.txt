cmake_minimum_required(VERSION 3.10)
project(UnoptimizedProject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED NO_MODULE)

set(SOURCES
    src/bodies.cpp
    src/constraints.cpp
    src/multibody_solver.cpp
    src/multibody_system.cpp
    src/quaternion_operations.cpp
)

add_executable(test src/test.cpp ${SOURCES})

target_include_directories(test PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    Eigen3::Eigen
)
