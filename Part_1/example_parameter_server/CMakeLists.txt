cmake_minimum_required(VERSION 3.5)
project(example_parameter_server)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp")
  install(TARGETS ${target}
    DESTINATION bin)
endfunction()

custom_executable(read_param_from_node src/read_param_from_node.cpp)
custom_executable(paramsrv src/paramsrv.cpp)

ament_package()
