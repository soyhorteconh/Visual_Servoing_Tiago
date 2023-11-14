cmake_minimum_required(VERSION 2.8.12)
project(inria_eiquadprog)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
)

add_compile_options(-std=c++17)

install(DIRECTORY include/ DESTINATION include)

