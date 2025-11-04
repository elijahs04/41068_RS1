cmake_minimum_required(VERSION 3.8)
project(pyrosens_gui)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include("${ament_cmake_core_DIR}/environment_hooks/ament_environment_hooks.cmake")

#Qt
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Qml Quick)

# Gazebo GUI (newer gz-gui first; fallback to ignition-gui)
# Adjust versions if your distro uses different majors.

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

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY qml/
  DESTINATION share/${PROJECT_NAME}/qml
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
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/pyrosens_gui.sh.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
