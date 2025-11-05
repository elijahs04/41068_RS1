cmake_minimum_required(VERSION 3.5)
project(pyrosens_pathPlanning)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)
else()
  add_compile_options(-Wunused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)


##################################################################################

add_library(pathPlanning src/pathPlanning.cpp)
target_include_directories(pathPlanning
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(pathPlanning
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  nav_msgs
  std_srvs
  sensor_msgs
  tf2
)

add_executable(runner src/runner.cpp)
target_include_directories(runner
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
ament_target_dependencies(runner
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  nav_msgs
  std_srvs
  sensor_msgs
  tf2
)
target_link_libraries(runner pathPlanning)


install (TARGETS
  runner
  pathPlanning
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES src/pathPlanning.h
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  nav_msgs
  std_srvs
  sensor_msgs
  tf2
)

ament_package()
