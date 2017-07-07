cmake_minimum_required(VERSION 3.5)

project(example_tf_listener)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

set(target example_tf_listener)

add_executable(${target} src/${target}.cpp)
ament_target_dependencies(${target}
  "rcl" "rclcpp" "console_bridge" "tf2" "tf2_ros" "nav_msgs" "geometry_msgs")
install(TARGETS ${target}
DESTINATION bin)

# custom_executable(example_tf_listener)

ament_package()
