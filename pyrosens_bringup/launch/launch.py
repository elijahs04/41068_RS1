import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # --- Common launch arguments ---
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo simulated time.",
    )
    world = DeclareLaunchArgument(
        "world",
        default_value="simple_trees",
        description="World selection forwarded to 41068 Ignition bringup.",
    )
    world_file = DeclareLaunchArgument(
        "world_file",
        default_value="",
        description="Absolute path to an SDF world. When set, overrides the built-in world selection.",
    )
    launch_nav2 = DeclareLaunchArgument(
        "launch_nav2",
        default_value="true",
        description="Whether to launch the Nav2 stack provided by 41068 bringup.",
    )
    launch_rviz = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Forward to 41068 bringup to launch RViz2 with their config.",
    )
    nav_action = DeclareLaunchArgument(
        "nav_action",
        default_value="/navigate_to_pose",
        description="NavigateToPose action name exposed by Nav2 (proxy target).",
    )
    launch_bridge = DeclareLaunchArgument(
        "launch_bridge",
        default_value="false",
        description="Launch ros_gz/ign bridge for auxiliary mission topics.",
    )
    ld.add_action(use_sim_time)
    ld.add_action(world)
    ld.add_action(world_file)
    ld.add_action(launch_nav2)
    ld.add_action(launch_rviz)
    ld.add_action(nav_action)
    ld.add_action(launch_bridge)

    # --- Environment so Ignition can find PyroSENS resources/plugins ---
    pyrosens_gui_share = get_package_share_directory("pyrosens_gui")
    pyrosens_gui_prefix = get_package_prefix("pyrosens_gui")
    pyrosens_world_share = get_package_share_directory("pyrosens_world")

    gui_config = os.path.join(pyrosens_gui_share, "gui", "pyrosens_with_panel.gui")
    plugin_dir = os.path.join(pyrosens_gui_prefix, "lib")
    existing_plugin_path = os.environ.get("IGN_GUI_PLUGIN_PATH", "")
    plugin_path = (
        f"{plugin_dir}{os.pathsep}{existing_plugin_path}"
        if existing_plugin_path
        else plugin_dir
    )

    existing_resource_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    resource_path = (
        f"{pyrosens_world_share}{os.pathsep}{existing_resource_path}"
        if existing_resource_path
        else pyrosens_world_share
    )

    ld.add_action(SetEnvironmentVariable("IGN_GAZEBO_GUI_CONFIG_PATH", gui_config))
    ld.add_action(SetEnvironmentVariable("IGN_GUI_PLUGIN_PATH", plugin_path))
    ld.add_action(SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_path))

    # --- Core 41068 bringup components (robot, Gazebo, Nav2) ---
    pkg_41068 = FindPackageShare("41068_ignition_bringup")
    config_path = PathJoinSubstitution([pkg_41068, "config"])
    urdf_file = PathJoinSubstitution([pkg_41068, "urdf", "husky.urdf.xacro"])
    world_default_path = PathJoinSubstitution(
        [pkg_41068, "worlds", [LaunchConfiguration("world"), ".sdf"]]
    )

    robot_description = ParameterValue(
        Command(["xacro ", urdf_file]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="robot_localization",
        output="screen",
        parameters=[
            PathJoinSubstitution([config_path, "robot_localization.yaml"]),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    world_file_is_set = PythonExpression(
        ['"', LaunchConfiguration("world_file"), '" != ""']
    )

    gui_config_arg = [" --gui-config ", gui_config]

    gazebo_default = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_ign_gazebo"), "launch", "ign_gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "ign_args": [
                world_default_path,
                " -r",
                *gui_config_arg,
            ]
        }.items(),
        condition=UnlessCondition(world_file_is_set),
    )

    gazebo_custom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_ign_gazebo"), "launch", "ign_gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "ign_args": [
                LaunchConfiguration("world_file"),
                " -r",
                *gui_config_arg,
            ]
        }.items(),
        condition=IfCondition(world_file_is_set),
    )

    robot_spawner = Node(
        package="ros_ign_gazebo",
        executable="create",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=["-topic", "/robot_description", "-z", "0.4"],
    )

    gazebo_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": PathJoinSubstitution([config_path, "gazebo_bridge.yaml"]),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=["-d", PathJoinSubstitution([config_path, "41068.rviz"])],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_41068, "launch", "41068_navigation.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_nav2")),
    )

    ld.add_action(robot_state_publisher)
    ld.add_action(robot_localization)
    ld.add_action(gazebo_default)
    ld.add_action(gazebo_custom)
    ld.add_action(robot_spawner)
    ld.add_action(gazebo_bridge)
    ld.add_action(rviz_node)
    ld.add_action(nav2_launch)

    # --- Mission manager and helpers (PyroSENS stacks) ---
    mission_params = PathJoinSubstitution(
        [FindPackageShare("pyrosens_mission"), "config", "mission_params.yaml"]
    )

    mission_node = Node(
        package="pyrosens_mission",
        executable="main_mission",
        name="mission_manager",
        output="screen",
        parameters=[
            mission_params,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"downstream_nav_action_name": LaunchConfiguration("nav_action")},
        ],
    )
    mission_cmd_bridge = Node(
        package="pyrosens_mission",
        executable="mission_cmd_bridge",
        name="mission_cmd_bridge",
        output="screen",
    )
    perception_bundle = Node(
        package="pyrosens_mission",
        executable="main_perception",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    simulation_bundle = Node(
        package="pyrosens_mission",
        executable="main_sim",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    bridge_condition = PythonExpression(
        ['"', LaunchConfiguration("launch_bridge"), '" == "true"']
    )

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/mission/status@std_msgs/msg/String]gz.msgs.StringMsg",
            "/mission/goals@std_msgs/msg/String]gz.msgs.StringMsg",
            "/mission/cmd@std_msgs/msg/String[gz.msgs.StringMsg",
            "/mission/load_goals@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
            "/wind/test_cloud@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/heat/temperature@sensor_msgs/msg/Temperature[gz.msgs.Temperature",
            "/sim/wind@std_msgs/msg/String]gz.msgs.StringMsg",
        ],
        condition=IfCondition(bridge_condition),
    )

    ld.add_action(mission_node)
    ld.add_action(mission_cmd_bridge)
    ld.add_action(perception_bundle)
    ld.add_action(simulation_bundle)
    ld.add_action(bridge_node)

    return ld
