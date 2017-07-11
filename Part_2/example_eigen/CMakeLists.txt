cmake_minimum_required(VERSION 3.5)

project(example_eigen) ### JAS change me!

### JAS change me as needed!
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# https://stackoverflow.com/a/12258855
set(EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}")
if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
endif()
include_directories("${EIGEN3_INCLUDE_DIR}")

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

set(target example_eigen_plane_fit)

add_executable(${target} src/${target}.cpp)
ament_target_dependencies(${target} "rclcpp")
install(TARGETS ${target} DESTINATION bin)

ament_package()
