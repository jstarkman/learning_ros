cmake_minimum_required(VERSION 3.5)

project(generic_gripper_services)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(baxter_core_msgs REQUIRED)
find_package(simple_baxter_gripper_interface REQUIRED)
find_package(std_srvs REQUIRED)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GenericGripperInterface.srv"
)

function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp" "baxter_core_msgs" "simple_baxter_gripper_interface" "std_srvs")
  rosidl_target_interfaces(${target}
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  install(TARGETS ${target}
    DESTINATION bin)
endfunction()

custom_executable(rethink_rt_gripper_service)
# custom_executable(virtual_vacuum_gripper_service) # std_srvs/srv/SetBool.srv is missing in ROS2
custom_executable(example_generic_gripper_client)

ament_package()
