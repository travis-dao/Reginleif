cmake_minimum_required(VERSION 3.16)
project(host_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(COMP ${CMAKE_CURRENT_SOURCE_DIR}/../components)

find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(testbench_ik
    testbench_ik.cpp
    ${COMP}/algorithms/ik.cpp
    ${COMP}/util/types.cpp
)

target_include_directories(testbench_ik PRIVATE
    ${COMP}/algorithms/include
    # ${COMP}/core/include
    ${COMP}/core/leg/include
    ${COMP}/util/include
)

target_link_libraries(testbench_ik PRIVATE Eigen3::Eigen)