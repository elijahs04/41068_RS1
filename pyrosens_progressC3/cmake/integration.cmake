# Runners that spin multiple nodes in one process

# Thermal-only runner
add_executable(main_thermal mains/main_thermal.cpp)
target_link_libraries(main_thermal thermSens_lib thermInterpolation_lib)
ament_target_dependencies(main_thermal rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros)

# Wind-only runner
add_executable(main_wind mains/main_wind.cpp)
target_link_libraries(main_wind windSensor_lib windInterpolation_lib)
ament_target_dependencies(main_wind rclcpp sensor_msgs geometry_msgs visualization_msgs)

# Interpolation-only runner (thermal + wind interpolation, no sensors) — optional
add_executable(main_interpolation mains/main_interpolation.cpp)
target_link_libraries(main_interpolation thermInterpolation_lib windInterpolation_lib)
ament_target_dependencies(main_interpolation rclcpp sensor_msgs geometry_msgs nav_msgs visualization_msgs)

# All-perception runner (4 nodes: therm sens+interp, wind sens+interp)
add_executable(main_perception mains/main_perception.cpp)
target_link_libraries(main_perception
  thermSens_lib thermInterpolation_lib
  windSensor_lib  windInterpolation_lib
)
ament_target_dependencies(main_perception rclcpp sensor_msgs geometry_msgs nav_msgs visualization_msgs tf2 tf2_ros)

# Standalone mains for each node (for testing/debugging)
add_executable(main_thermSens mains/main_thermSens.cpp)
target_link_libraries(main_thermSens thermSens_lib)
ament_target_dependencies(main_thermSens rclcpp sensor_msgs geometry_msgs tf2 tf2_ros)

add_executable(main_thermInterpolation mains/main_thermInterpolation.cpp)
target_link_libraries(main_thermInterpolation thermInterpolation_lib)
ament_target_dependencies(main_thermInterpolation rclcpp sensor_msgs geometry_msgs nav_msgs)

add_executable(main_windSens mains/main_windSens.cpp)
target_link_libraries(main_windSens windSensor_lib)
ament_target_dependencies(main_windSens rclcpp sensor_msgs geometry_msgs)

add_executable(main_windInterpolation mains/main_windInterpolation.cpp)
target_link_libraries(main_windInterpolation windInterpolation_lib)
ament_target_dependencies(main_windInterpolation rclcpp geometry_msgs visualization_msgs)

add_executable(main_thermalSim mains/main_thermalSim.cpp)
target_link_libraries(main_thermalSim thermalSim_lib)
ament_target_dependencies(main_thermalSim rclcpp sensor_msgs geometry_msgs)

add_executable(main_windSim mains/main_windSim.cpp)
target_link_libraries(main_windSim windSim_lib)
ament_target_dependencies(main_windSim rclcpp sensor_msgs)


install(TARGETS
  # Runners
  main_thermal
  main_wind
  main_interpolation
  main_perception

  # Standalone mains
  main_thermSens
  main_thermInterpolation
  main_windSens
  main_windInterpolation
  main_thermalSim
  main_windSim

  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
