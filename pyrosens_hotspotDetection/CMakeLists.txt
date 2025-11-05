cmake_minimum_required(VERSION 3.8)
project(pyrosens_hotspotDetection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------- Dependencies ----------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)

# ---------------------- Library (shared core) -------------
add_library(hotspot_core_lib
  src/thermalHotspotNode.cpp
  src/hotspotDetector.cpp
  src/hotspotTransform.cpp
)

target_include_directories(hotspot_core_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(hotspot_core_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  cv_bridge
  vision_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  message_filters
  OpenCV
)

target_link_libraries(hotspot_core_lib
  ${OpenCV_LIBS}
)

# ---------------------- Executables -----------------------
# 1) Thermal detector node
add_executable(thermalHotspotNode
  mains/main_thermalHotspotNode.cpp
)
target_link_libraries(thermalHotspotNode hotspot_core_lib)
ament_target_dependencies(thermalHotspotNode rclcpp)

# 2) Heatmap/mapper node
add_executable(hotspotMapperNode
  src/hotspotMapperNode.cpp
  mains/main_hotspotMapperNode.cpp
)
# If Mapper reuses helper code from the core lib, link it as well
target_link_libraries(hotspotMapperNode hotspot_core_lib)
ament_target_dependencies(hotspotMapperNode
  rclcpp sensor_msgs geometry_msgs nav_msgs std_srvs
  cv_bridge tf2 tf2_ros OpenCV
)

# ---------------------- Install ---------------------------
install(TARGETS
  thermalHotspotNode
  hotspotMapperNode
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  hotspot_core_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ---------------------- Export ----------------------------
ament_export_include_directories(include)
ament_export_libraries(hotspot_core_lib)
ament_export_dependencies(
  rclcpp sensor_msgs geometry_msgs nav_msgs std_srvs
  cv_bridge vision_msgs OpenCV tf2 tf2_ros tf2_geometry_msgs
  Eigen3 message_filters
)

ament_package()
