from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Launch arguments ---
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([
            FindPackageShare("pyrosens_world"),
            "test_world.sdf",
        ]),
        description="Absolute path to the Gazebo world file.",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulated time in all ROS nodes.",
    )

    enable_nav2_arg = DeclareLaunchArgument(
        "enable_nav2",
        default_value="false",
        description="Bring up Nav2 alongside the mission manager.",
    )

    nav2_params_arg = DeclareLaunchArgument(
        "nav2_params",
        default_value=PathJoinSubstitution([
            FindPackageShare("pyrosens_mission"),
            "config",
            "nav2_params.yaml",
        ]),
        description="Nav2 parameter file.",
    )

    nav2_map_arg = DeclareLaunchArgument(
        "nav2_map",
        default_value=PathJoinSubstitution([
            FindPackageShare("pyrosens_mission"),
            "maps",
            "test_map.yaml",
        ]),
        description="Nav2 map YAML.",
    )

    nav2_slam_arg = DeclareLaunchArgument(
        "nav2_slam",
        default_value="False",
        description="Enable Nav2 SLAM instead of static map (True/False).",
    )

    # --- Include existing GUI launcher (spawns Ignition Gazebo + panel) ---
    gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("pyrosens_gui"),
            "launch",
            "pyrosens_with_panel.launch.py",
        ])),
        launch_arguments={
            "world": LaunchConfiguration("world"),
        }.items(),
    )

    # --- Optional Nav2 bring-up ---
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("nav2_bringup"),
            "launch",
            "bringup_launch.py",
        ])),
        launch_arguments={
            "namespace": "nav2_downstream",
            "use_namespace": "true",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": "true",
            "params_file": LaunchConfiguration("nav2_params"),
            "slam": LaunchConfiguration("nav2_slam"),
            "map": LaunchConfiguration("nav2_map"),
        }.items(),
    )

    nav2_group = GroupAction(
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
        actions=[nav2_launch],
    )

    # --- ROS ↔ Gazebo bridge ---
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            # Mission status exported to Gazebo consumers (ROS -> Gazebo)
            "/mission/status@std_msgs/msg/String]gz.msgs.StringMsg",
            "/mission/goals@std_msgs/msg/String]gz.msgs.StringMsg",
            # GUI command channel (Gazebo -> ROS)
            "/mission/cmd@std_msgs/msg/String[gz.msgs.StringMsg",
            # Goal ingestion from Gazebo systems (Gazebo -> ROS)
            "/mission/load_goals@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
            # Wind and thermal telemetry (Gazebo -> ROS)
            "/wind/test_cloud@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/heat/temperature@sensor_msgs/msg/Temperature[gz.msgs.Temperature",
            # Wind string back to Gazebo (ROS -> Gazebo)
            "/sim/wind@std_msgs/msg/String]gz.msgs.StringMsg",
        ],
    )

    # --- Mission manager + adapters ---
    mission_params = PathJoinSubstitution([
        FindPackageShare("pyrosens_mission"),
        "config",
        "mission_params.yaml",
    ])

    mission_node = Node(
        package="pyrosens_mission",
        executable="main_mission",
        name="mission_manager",
        output="screen",
        parameters=[
            mission_params,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"downstream_nav_action_name": "/navigate_to_pose",
             "upstream_nav_action_name": "/mission/navigate_to_pose"},
        ],
    )

    cmd_bridge_node = Node(
        package="pyrosens_mission",
        executable="mission_cmd_bridge",
        name="mission_cmd_bridge",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # --- Perception + simulation bundles (thermal / wind) ---
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

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        enable_nav2_arg,
        nav2_params_arg,
        nav2_map_arg,
        nav2_slam_arg,
        gui_launch,
        nav2_group,
        bridge_node,
        mission_node,
        cmd_bridge_node,
        perception_bundle,
        simulation_bundle,
    ])
