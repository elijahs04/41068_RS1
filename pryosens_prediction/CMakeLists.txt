cmake_minimum_required(VERSION 3.8)
project(pyrosens_prediction)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------- Dependencies ----------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ---------------------- Libraries -------------------------
# Sensor / sim lib
add_library(thermSens_lib src/thermSensNode.cpp)
target_include_directories(thermSens_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(thermSens_lib
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros)

# Interpolation lib (matches your camelCase files)
add_library(thermInterpolation_lib src/thermInterpolationNode.cpp)
target_include_directories(thermInterpolation_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(thermInterpolation_lib
  rclcpp sensor_msgs geometry_msgs nav_msgs)

# ---------------------- Executables -----------------------
add_executable(thermSensNode src/main_thermSens.cpp)
target_link_libraries(thermSensNode thermSens_lib)

add_executable(thermInterpolationNode src/main_thermInterpolation.cpp)
target_link_libraries(thermInterpolationNode thermInterpolation_lib)

# Standalone sim executable (if you keep it)
add_executable(thermalSimNode src/thermalSimNode.cpp)
ament_target_dependencies(thermalSimNode rclcpp sensor_msgs geometry_msgs)

# ---------------------- Install ---------------------------
install(TARGETS
  thermSensNode
  thermInterpolationNode
  thermalSimNode
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  thermSens_lib
  thermInterpolation_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

# ---------------------- Export ----------------------------
ament_export_include_directories(include)
ament_export_libraries(thermSens_lib thermInterpolation_lib)
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros)

# ---------------------- Package ---------------------------
ament_package()


