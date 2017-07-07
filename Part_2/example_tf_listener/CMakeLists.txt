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

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(target example_tf_listener)

add_executable(${target} src/${target}.cpp)
ament_target_dependencies(${target}
  "rcl" "rclcpp" "console_bridge" "tf2" "tf2_ros" "nav_msgs" "geometry_msgs")
install(TARGETS ${target}
DESTINATION bin)

# custom_executable(example_tf_listener)

ament_package()
