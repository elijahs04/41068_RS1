cmake_minimum_required(VERSION 3.5)
project(pyrosens)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)
else()
  add_compile_options(-Wunused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)

##################################################################################

add_library(goals src/goals.cpp)
ament_target_dependencies(goals 
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  std_srvs
)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(runner src/runner.cpp)
ament_target_dependencies(runner
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  std_srvs
)
target_link_libraries(runner goals)



ament_target_dependencies(runner
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  std_srvs
) 


install (TARGETS
  runner
  goals
  DESTINATION lib/${PROJECT_NAME}
)




ament_package()
