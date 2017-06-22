cmake_minimum_required(VERSION 3.5)

project(mobot_drifty_odom)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

set(target mobot_drifty_odom)

add_executable(${target} src/${target}.cpp)
ament_target_dependencies(${target}
  "rcl" "rclcpp" "sensor_msgs" "nav_msgs" "geometry_msgs")
install(TARGETS ${target}
DESTINATION bin)

ament_package()
