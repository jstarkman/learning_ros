cmake_minimum_required(VERSION 3.5)
project(localization_w_gps)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)

find_package(std_msgs)
find_package(nav_msgs)
find_package(geometry_msgs)
find_package(xform_utils)
find_package(sensor_msgs)

# https://stackoverflow.com/a/12258855
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3/")
include_directories("${EIGEN3_INCLUDE_DIR}")

set(target localization_w_gps)

add_executable(${target} src/${target}.cpp)
ament_target_dependencies(${target} "rclcpp" "std_msgs" "nav_msgs" "geometry_msgs" "xform_utils" "sensor_msgs")
install(TARGETS ${target} DESTINATION bin)

ament_package()
