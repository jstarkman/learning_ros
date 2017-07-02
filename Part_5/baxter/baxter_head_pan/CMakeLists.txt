cmake_minimum_required(VERSION 3.5)

project(baxter_head_pan)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(baxter_core_msgs REQUIRED)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp" "std_msgs" "baxter_core_msgs")
  install(TARGETS ${target}
    DESTINATION bin)
endfunction()

custom_executable(baxter_head_pan)
custom_executable(baxter_head_pan_zero)

ament_package()
