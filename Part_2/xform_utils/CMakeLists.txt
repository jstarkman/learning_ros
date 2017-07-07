cmake_minimum_required(VERSION 3.5)

project(xform_utils)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# https://stackoverflow.com/a/12258855
set(EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}")
if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
endif()
include_directories("${EIGEN3_INCLUDE_DIR}")

include_directories(include)
ament_export_include_directories(include)

add_library(${PROJECT_NAME} SHARED src/xform_utils.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp std_msgs nav_msgs geometry_msgs tf2)
# ament_export_libraries(${PROJECT_NAME})
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "XFORM_UTILS_BUILDING_LIBRARY")
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(${PROJECT_NAME})

set(target example_xform_utils)

add_executable(${target} src/${target}.cpp)
target_link_libraries(${target} ${PROJECT_NAME})
ament_target_dependencies(${target} "rclcpp" "std_msgs" "nav_msgs" "geometry_msgs" "tf2" ${PROJECT_NAME})
install(TARGETS ${target} DESTINATION bin)

ament_package()

install(DIRECTORY include/
  DESTINATION include
)
