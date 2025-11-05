cmake_minimum_required(VERSION 3.8)
project(pyrosens_integrated_sensors)

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
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)

# Public headers for all targets
set(PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(SRC_FILES
  src/hotspotDetector.cpp
  src/hotspotMapperNode.cpp
  src/hotspotTransform.cpp
  src/thermalHotspotNode.cpp
  src/windInterpolationNode.cpp
  src/windSensNode.cpp
  src/windSimNode.cpp
)

add_library(${PROJECT_NAME}_lib SHARED ${SRC_FILES})
target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  cv_bridge
  std_srvs
)
target_link_libraries(${PROJECT_NAME}_lib
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
)

function(add_sensor_executable target source)
  add_executable(${target} ${source})
  target_link_libraries(${target} ${PROJECT_NAME}_lib)
  ament_target_dependencies(${target}
    rclcpp
  )
  install(TARGETS ${target}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

add_sensor_executable(thermal_hotspot_node mains/main_thermalHotspotNode.cpp)
add_sensor_executable(hotspot_mapper_node mains/main_hotspotMapperNode.cpp)
add_sensor_executable(wind_sim_node mains/main_windSim.cpp)
add_sensor_executable(wind_sensor_node mains/main_windSens.cpp)
add_sensor_executable(wind_interpolation_node mains/main_windInterpolation.cpp)

install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz FILES_MATCHING PATTERN "*.rviz")

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  cv_bridge
  std_srvs
)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
