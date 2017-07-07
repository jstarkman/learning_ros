cmake_minimum_required(VERSION 3.5)

project(arm7dof_fk_ik)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

# https://stackoverflow.com/a/12258855
set(EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}")
if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
endif()
include_directories("${EIGEN3_INCLUDE_DIR}")

include_directories(include)
ament_export_include_directories(include)

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp sensor_msgs)
# ament_export_libraries(${PROJECT_NAME})
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "arm7dof_fk_ik_BUILDING_LIBRARY")
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(${PROJECT_NAME})

function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  target_link_libraries(${target} ${PROJECT_NAME})
  ament_target_dependencies(${target} "rclcpp" "rcutils" "sensor_msgs" ${PROJECT_NAME})
  install(TARGETS ${target} DESTINATION bin)
endfunction()

custom_executable(arm7dof_fk_ik_test_main)
custom_executable(arm7dof_fk_ik_test_main2)

ament_package()

install(DIRECTORY include/
  DESTINATION include
)
