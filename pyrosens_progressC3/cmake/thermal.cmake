# Libraries for thermal nodes (3 nodes total: sensor, interpolation, sim)
add_library(thermSens_lib src/thermSensNode.cpp)
ament_target_dependencies(thermSens_lib rclcpp sensor_msgs geometry_msgs tf2 tf2_ros)

add_library(thermInterpolation_lib src/thermInterpolationNode.cpp)
ament_target_dependencies(thermInterpolation_lib rclcpp sensor_msgs geometry_msgs nav_msgs)

add_executable(thermalSimNode src/thermalSimNode.cpp)
ament_target_dependencies(thermalSimNode rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  thermSens_lib
  thermInterpolation_lib
  thermalSimNode
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
