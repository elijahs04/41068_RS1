# launch/mission_proxy_with_nav2.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ---- Nav2 bringup args (adjust to your map/slam setup) ----
    nav2_params = DeclareLaunchArgument(
        "nav2_params",
        default_value=PathJoinSubstitution([FindPackageShare("nav2_bringup"), "params", "nav2_params.yaml"]),
        description="Nav2 params file"
    )
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    slam = DeclareLaunchArgument("slam", default_value="False")   # set True if you want SLAM
    map_yaml = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Path to a map YAML (leave empty if using SLAM)"
    )

    # ---- Include Nav2 bringup (un-namespaced) ----
    nav2_namespace = ""

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"])
        ),
        launch_arguments={
            "namespace": nav2_namespace,
            "use_namespace": "false",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": "true",
            "params_file": LaunchConfiguration("nav2_params"),
            "slam": LaunchConfiguration("slam"),
            "map": LaunchConfiguration("map"),
        }.items(),
    )

    # ---- Mission proxy node ----
    mission = Node(
        package="mission_pyrosens",
        executable="main_mission",
        name="mission_manager",
        output="screen",
        parameters=[
            # Proxy forwards to Nav2 running on its default namespace
            {"downstream_nav_action_name": "/navigate_to_pose",
             "upstream_nav_action_name": "/mission/navigate_to_pose"},
            # Optional: load other mission params from YAML (below)
            PathJoinSubstitution([FindPackageShare("mission_pyrosens"), "config", "mission_params.yaml"]),
        ],
        # If your teammate also sends waypoints as topics/paths, you can remap or set params here
        # remappings=[("some/topic", "other/topic")],
    )

    return LaunchDescription([
        nav2_params, use_sim_time, slam, map_yaml,
        nav2,
        mission,
    ])
