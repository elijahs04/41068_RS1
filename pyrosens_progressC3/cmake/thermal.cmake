# Libraries for thermal nodes (3 nodes total: sensor, interpolation, sim)
add_library(thermSens_lib src/thermSensNode.cpp)
ament_target_dependencies(thermSens_lib rclcpp sensor_msgs geometry_msgs tf2 tf2_ros)

add_library(thermInterpolation_lib src/thermInterpolationNode.cpp)
ament_target_dependencies(thermInterpolation_lib rclcpp sensor_msgs geometry_msgs nav_msgs)

add_library(thermalSim_lib src/thermalSimNode.cpp)
ament_target_dependencies(thermalSim_lib rclcpp sensor_msgs geometry_msgs)


install(TARGETS
  thermSens_lib
  thermInterpolation_lib
  thermalSim_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
