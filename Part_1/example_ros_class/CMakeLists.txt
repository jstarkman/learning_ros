cmake_minimum_required(VERSION 3.5)
project(example_ros_class)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)


function(add_install_exe target files)
  add_executable(${target} ${files})
  ament_target_dependencies(${target}
    "rclcpp" "rcutils" "std_msgs" "std_srvs")
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()


add_install_exe(example_ros_class src/example_ros_class.cpp)

ament_package()