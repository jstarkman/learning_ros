cmake_minimum_required(VERSION 3.5)

project(joint_space_planner)

find_package(ament_cmake REQUIRED)

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

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE ${PROJECT_NAME}"_BUILDING_LIBRARY")
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ament_export_libraries(${PROJECT_NAME})

ament_package()

install(DIRECTORY include/
  DESTINATION include
)
