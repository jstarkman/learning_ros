cmake_minimum_required(VERSION 3.5)
project(example_ros_service)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(example_ros_service
  "srv/ExampleServiceMsg.srv"
  "srv/PathSrv.srv"
  DEPENDENCIES builtin_interfaces std_msgs nav_msgs
)


function(add_install_exe target files)
  add_executable(${target} ${files})
  ament_target_dependencies(${target}
    "rclcpp" "rcutils" "std_msgs" "nav_msgs" "geometry_msgs")
  rosidl_target_interfaces(${target}
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  install(TARGETS ${target} DESTINATION bin)
endfunction()

add_install_exe(example_ros_service_exe src/example_ros_service.cpp)
add_install_exe(example_ros_client src/example_ros_client.cpp)
add_install_exe(path_service src/path_service.cpp)
add_install_exe(path_service2 src/path_service2.cpp)
add_install_exe(path_client src/path_client.cpp)

ament_package()
