cmake_minimum_required(VERSION 3.5)

project(minimal_nodes) ### JAS change me!

### JAS change me as needed!
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)

### JAS no touchy!
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

### JAS read me and maybe change me!
function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp"
    "std_msgs")
  install(TARGETS ${target}
    DESTINATION bin)
endfunction()
## how to extend the above function for special cases
# custom_executable(imu_listener)
# ament_target_dependencies(imu_listener "sensor_msgs")
## make sure to find_package(sensor_msgs REQUIRED) first

### JAS change me!
custom_executable(minimal_publisher)
custom_executable(sleepy_minimal_publisher)
custom_executable(minimal_subscriber)
custom_executable(minimal_simulator)
custom_executable(minimal_controller)

### JAS no touchy!
ament_package()
