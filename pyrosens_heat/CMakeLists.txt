cmake_minimum_required(VERSION 3.8)
project(pyrosens_heat)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# --- Executables
add_executable(sensorNode src/sensorNode.cpp)
ament_target_dependencies(sensorNode rclcpp sensor_msgs geometry_msgs)

add_executable(mapperNode src/mapperNode.cpp)
ament_target_dependencies(mapperNode rclcpp sensor_msgs geometry_msgs nav_msgs)

# --- Install
install(TARGETS
  sensorNode
  mapperNode
  DESTINATION lib/${PROJECT_NAME}
)

# (Optional) install launch/ if you have it:
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# --- Testing lint (unchanged)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
