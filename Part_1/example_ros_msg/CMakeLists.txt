cmake_minimum_required(VERSION 3.5)
project(example_ros_msg)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

rosidl_generate_interfaces(example_ros_msg
  "msg/ExampleMessage.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

function(add_link_install_exe target files)
  add_executable(${target} ${files})
  ament_target_dependencies(${target}
    "rclcpp" "rcutils" "std_msgs" "custom_msgs")
  rosidl_target_interfaces(${target}
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  install(TARGETS ${target} DESTINATION bin)
endfunction()

add_link_install_exe(example_ros_message_publisher src/example_ros_message_publisher.cpp)
add_link_install_exe(vector_publisher src/vector_publisher.cpp)
add_link_install_exe(vector_subscriber src/vector_subscriber.cpp)

ament_package()
