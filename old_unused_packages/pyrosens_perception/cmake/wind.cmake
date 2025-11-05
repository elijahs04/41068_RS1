# Libraries for wind nodes (3 nodes total: sensor, interpolation, sim)
add_library(windSensor_lib src/windSensNode.cpp)
ament_target_dependencies(windSensor_lib rclcpp sensor_msgs geometry_msgs)

add_library(windInterpolation_lib src/windInterpolationNode.cpp)
ament_target_dependencies(windInterpolation_lib rclcpp geometry_msgs visualization_msgs)

add_library(windSim_lib src/windSimNode.cpp)
ament_target_dependencies(windSim_lib rclcpp sensor_msgs)

install(TARGETS
  windSensor_lib
  windInterpolation_lib
  windSim_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
