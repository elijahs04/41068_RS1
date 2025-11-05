cmake_minimum_required(VERSION 3.8)
project(pyrosens_progress)

# Language + warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Common deps (add more here if you want them global)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)   # for goals
find_package(nav2_msgs REQUIRED)

# Public headers for all targets
include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Pull in component cmake files
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/thermal.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/wind.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/movement.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/integration.cmake)

# Shared data
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz FILES_MATCHING PATTERN "*.rviz")

ament_package()
