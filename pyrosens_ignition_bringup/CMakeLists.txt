cmake_minimum_required(VERSION 3.8)
project(pyrosens_ignition_bringup)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_ign_interfaces REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets Qml Quick)
find_package(ignition-gui6 REQUIRED)
set(GZ_GUI_TARGET ignition-gui6::ignition-gui6)

qt5_add_resources(pyrosens_gui_qml
  qml/pyrosens_gui.qrc
)

add_library(pyrosens_gui SHARED
  src/pyrosens_gui.cpp
  include/pyrosens_gui/pyrosens_gui.hpp
  ${pyrosens_gui_qml}
)
target_include_directories(pyrosens_gui
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(pyrosens_gui
  Qt5::Core
  Qt5::Widgets
  Qt5::Qml
  Qt5::Quick
  ${GZ_GUI_TARGET}
)

ament_target_dependencies(pyrosens_gui rclcpp std_msgs)

install(TARGETS pyrosens_gui
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY qml/
  DESTINATION share/${PROJECT_NAME}/qml
)

install(
  DIRECTORY gui/
  DESTINATION share/${PROJECT_NAME}/gui
)

install(
  DIRECTORY config launch models worlds urdf_drone sim_env_models
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/pyrosens_gui.sh.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
