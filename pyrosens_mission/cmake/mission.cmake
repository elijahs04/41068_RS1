add_library(mission_core_lib src/mission.cpp)
ament_target_dependencies(mission_core_lib
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  nav2_msgs
)

add_executable(main_mission mains/main_mission.cpp)
target_link_libraries(main_mission mission_core_lib)
ament_target_dependencies(main_mission rclcpp)

add_executable(mission_cmd_bridge src/mission_cmd_bridge.cpp)
ament_target_dependencies(mission_cmd_bridge rclcpp std_msgs std_srvs)

install(TARGETS
  mission_core_lib
  main_mission
  mission_cmd_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
