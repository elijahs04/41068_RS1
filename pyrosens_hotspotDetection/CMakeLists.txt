cmake_minimum_required(VERSION 3.8)
project(pyrosens_hotspotDetection)

# C++ standard & warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------- Dependencies ----------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)

# ---------------------- Library ---------------------------
# Core implementation (node + detector in a lib), like your other package
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
  cv_bridge
  vision_msgs
  Eigen3
)

target_link_libraries(hotspot_core_lib
  ${OpenCV_LIBS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  message_filters::message_filters
)

# ---------------------- Executable ------------------------
add_executable(thermalHotspotNode
  mains/main_thermalHotspotNode.cpp
)

target_link_libraries(thermalHotspotNode
  hotspot_core_lib
)

# ---------------------- Install ---------------------------
install(TARGETS
  thermalHotspotNode
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

# (Optional later)
# install(DIRECTORY launch config
#   DESTINATION share/${PROJECT_NAME})

# ---------------------- Export ----------------------------
ament_export_include_directories(include)
ament_export_libraries(hotspot_core_lib)
ament_export_dependencies(rclcpp sensor_msgs cv_bridge vision_msgs OpenCV)

# ---------------------- Package ---------------------------
ament_package()
