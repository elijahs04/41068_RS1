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

To run the integrated system (uses 41068 Ignition bringup resources while launching PyroSENS mission + perception):

```
ros2 launch pyrosens_bringup launch.py
```

Key launch arguments (all optional):
- `world:=simple_trees|large_demo|test_world` &mdash; selects bundled 41068 worlds
- `world_file:=/abs/path/to/world.sdf` &mdash; override with a custom SDF (e.g. `${share.pyrosens_world}/test_world.sdf`)
- `launch_nav2:=true|false` &mdash; toggle the Nav2 stack configured by 41068
- `launch_rviz:=true|false` &mdash; bring up the 41068 RViz layout
- `nav_action:=/navigate_to_pose` &mdash; NavigateToPose action name the mission proxies to
- `launch_bridge:=true|false` &mdash; enable the auxiliary ros_gz bridge for mission-related topics

Example using the PyroSENS world:

```
ros2 launch pyrosens_bringup launch.py world_file:=$(ros2 pkg prefix pyrosens_world)/share/pyrosens_world/test_world.sdf
```

Or a specific node:

```
ros2 run pyrosens_core my_node   # replace my_node with the executable name
```

The my_node and my_component files are placeholders to demonstrate how code is to be implemented.
