cmake_minimum_required(VERSION 3.8)
project(pyrosens_prediction)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(pyrosens_prediction_nodes
  src/predictionNode.cpp
  src/predictionSim.cpp
)
target_include_directories(pyrosens_prediction_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pyrosens_prediction_nodes
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs)

add_executable(prediction_main main/main_prediction.cpp)
target_link_libraries(prediction_main pyrosens_prediction_nodes)

install(TARGETS
  pyrosens_prediction_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  prediction_main
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(pyrosens_prediction_nodes)
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs visualization_msgs)

ament_package()

