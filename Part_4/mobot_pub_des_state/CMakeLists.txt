cmake_minimum_required(VERSION 3.5)

project(mobot_pub_des_state)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(traj_builder REQUIRED)

find_package(rosidl_default_generators REQUIRED)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Path.srv"
  DEPENDENCIES nav_msgs
)

# https://stackoverflow.com/a/12258855
set(EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}")
if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
endif()
include_directories("${EIGEN3_INCLUDE_DIR}")

macro(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp" "std_msgs" "std_srvs" "nav_msgs" "geometry_msgs" "traj_builder")
  rosidl_target_interfaces(${target}
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  install(TARGETS ${target}
    DESTINATION bin)
endmacro()

custom_executable(pub_des_state_main)
custom_executable(pub_des_state_startup_main)
custom_executable(open_loop_controller)
custom_executable(pub_des_state_path_client)
custom_executable(pub_des_state_path_client_3mx3m_sqr)
# custom_executable(pub_des_state_path_client_10mx10m_sqr)
custom_executable(starting_pen_pub_des_state_path_client)

ament_package()
