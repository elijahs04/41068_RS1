add_library(goals_lib src/goals.cpp)
ament_target_dependencies(goals_lib
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

add_executable(main_goals mains/main_goals.cpp)
target_link_libraries(main_goals goals_lib)
ament_target_dependencies(main_goals
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

install(TARGETS
  goals_lib
  main_goals
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
