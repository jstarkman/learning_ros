cmake_minimum_required(VERSION 3.5)

project(generic_gripper_services)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(baxter_core_msgs REQUIRED)
find_package(simple_baxter_gripper_interface REQUIRED)
find_package(std_srvs REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GenericGripperInterface.srv"
)

function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp" "rcutils" "baxter_core_msgs" "simple_baxter_gripper_interface" "std_srvs")
  rosidl_target_interfaces(${target}
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

custom_executable(rethink_rt_gripper_service)
# custom_executable(virtual_vacuum_gripper_service) # std_srvs/srv/SetBool.srv is missing in ROS2
custom_executable(example_generic_gripper_client)

ament_package()
