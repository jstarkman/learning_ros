cmake_minimum_required(VERSION 3.5)

project(baxter_fk_ik)

find_package(ament_cmake REQUIRED)

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
  PRIVATE "XFORM_UTILS_BUILDING_LIBRARY")
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(${PROJECT_NAME})

set(target baxter_reachability_from_above)

add_executable(${target} src/${target}.cpp)
target_link_libraries(${target} ${PROJECT_NAME})
ament_target_dependencies(${target} ${PROJECT_NAME})
install(TARGETS ${target} DESTINATION bin)

ament_package()

install(DIRECTORY include/
  DESTINATION include
)
