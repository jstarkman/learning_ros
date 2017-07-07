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

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(target mobot_drifty_odom)

add_executable(${target} src/${target}.cpp)
ament_target_dependencies(${target}
  "rcl" "rclcpp" "sensor_msgs" "nav_msgs" "geometry_msgs")
install(TARGETS ${target}
DESTINATION lib/${PROJECT_NAME})

ament_package()
