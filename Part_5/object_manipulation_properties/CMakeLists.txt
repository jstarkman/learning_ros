cmake_minimum_required(VERSION 3.5)
project(object_manipulation_properties)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(xform_utils REQUIRED)
find_package(object_manipulation_msgs REQUIRED)

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
  rclcpp std_msgs geometry_msgs xform_utils object_manipulation_msgs)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE ${PROJECT_NAME}"_BUILDING_LIBRARY")
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(${PROJECT_NAME})

macro(add_install_exe target)
  add_executable(${target} src/${target}.cpp)
  target_link_libraries(${target} ${PROJECT_NAME})
  ament_target_dependencies(${target}
    "rclcpp" "rcutils" "std_msgs" "geometry_msgs" "xform_utils" "object_manipulation_msgs")
  install(TARGETS ${target} DESTINATION bin)
endmacro()

add_install_exe(object_manipulation_query_svc)
add_install_exe(example_object_manip_query_client)
add_install_exe(object_manipulation_properties_test_main)

ament_package()

install(DIRECTORY include/
  DESTINATION include
)
