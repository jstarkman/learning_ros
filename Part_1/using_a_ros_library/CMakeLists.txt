cmake_minimum_required(VERSION 3.5)
project(using_a_ros_library)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
# Depending on our own library
find_package(creating_a_ros_library REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

function(add_link_install_exe target files)
  add_executable(${target} ${files})
  # Depending on our own library
  ament_target_dependencies(${target}
    "rclcpp" "creating_a_ros_library" "std_msgs" "std_srvs")
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

add_link_install_exe(example_ros_class_test_main
                     src/example_ros_class_test_main.cpp)

ament_package()
