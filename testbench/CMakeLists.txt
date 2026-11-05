cmake_minimum_required(VERSION 3.16)
project(host_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(COMP ${CMAKE_CURRENT_SOURCE_DIR}/../components)

# Eigen (header-only, needed because types.h includes it)
find_package(Eigen3 REQUIRED NO_MODULE)

add_library(testbench_host
    ${COMP}/util/types.cpp
    ${COMP}/algorithms/ik.cpp
)

target_include_directories(testbench_host PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs
    ${COMP}/util/include
    ${COMP}/algorithms/include
    ${COMP}/motor/include
    ${COMP}/core/leg/include
)

target_link_libraries(testbench_host PUBLIC Eigen3::Eigen)

add_executable(testbench_ik testbench_ik.cpp)
target_link_libraries(testbench_ik PRIVATE testbench_host)