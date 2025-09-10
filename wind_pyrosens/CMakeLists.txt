cmake_minimum_required(VERSION 3.8)
project(wind_pyrosens)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ standard (recommended)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# --- Include headers
include_directories(include)

# --- Build libraries (node classes)
add_library(sensorNode_lib src/sensorNode.cpp)
ament_target_dependencies(sensorNode_lib
  rclcpp sensor_msgs geometry_msgs)

add_library(mapperNode_lib src/mapperNode.cpp)
ament_target_dependencies(mapperNode_lib
  rclcpp geometry_msgs visualization_msgs)

# --- Build executables (mains) linking the libs
add_executable(sensorNode src/main_sensor.cpp)
target_link_libraries(sensorNode sensorNode_lib)

add_executable(mapperNode src/main_mapper.cpp)
target_link_libraries(mapperNode mapperNode_lib)

install(TARGETS
  sensorNode_lib
  mapperNode_lib
  sensorNode
  mapperNode
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
