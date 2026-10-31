cmake_minimum_required(VERSION 3.16)
project(host_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(COMP ${CMAKE_CURRENT_SOURCE_DIR}/../components)

# Eigen (header-only, needed because types.h includes it)
find_package(Eigen3 REQUIRED NO_MODULE)

add_library(mpc_host
    ${COMP}/mpc/mpc.cpp
)

target_include_directories(mpc_host PUBLIC
    ${COMP}/mpc/include
    ${COMP}/util/include
)

target_link_libraries(mpc_host PUBLIC Eigen3::Eigen)

add_executable(foothold_test foothold_test.cpp)
target_link_libraries(foothold_test PRIVATE mpc_host)