### Standard Directory Layout for the Project ###
pyrosens_core/
  package.xml
  CMakeLists.txt
  include/pyrosens_core/      # public headers
  src/
    lib/                       # reusable libs (no main())
    nodes/                     # executables (have main())
    components/                # rclcpp_components (no main())
  launch/                      # optional per-node launch
  config/                      # node YAMLs
  resource/                    # auto-filled for components
  test/                        # gtests (optional)

pyrosens_bringup/
  package.xml
  CMakeLists.txt
  launch/   # system-level launch files
  config/   # system or multi-node YAML

Rules to follow:
Any files with a main() go in src/nodes
Files that define a component (i.e., no main, class derives rclcpp::Node) go in src/components
Common helpers go in src/lib with headers in include/pyrosens_core

To write launch files, add a YAML with their node name and extend the list found in pyrosens_bringup/launch/launch.py

To run the system:
ros2 launch pyrosens_bringup launch.py

Or a specific node:
ros2 run pyrosens_core my_node (replace my_node with said node)

The my_node and my_component files are placeholders to demonstrate how code is to be implemented.