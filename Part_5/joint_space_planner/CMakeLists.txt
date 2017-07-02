cmake_minimum_required(VERSION 3.5)

project(joint_space_planner)

find_package(ament_cmake REQUIRED)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

# https://stackoverflow.com/a/12258855
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3/")
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
  RUNTIME DESTINATION bin
)
ament_export_libraries(${PROJECT_NAME})

ament_package()

install(DIRECTORY include/
  DESTINATION include
)
