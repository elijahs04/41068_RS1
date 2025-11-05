import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix, get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    # Get paths to directories
    pkg_path = FindPackageShare('pyrosens_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path,
                                       'config'])
    pkg_share = get_package_share_directory('pyrosens_ignition_bringup')

    gui_config_path = os.path.join(pkg_share, 'gui', 'pyrosens_with_panel.gui')
    pyrosens_gui_prefix = get_package_prefix('pyrosens_ignition_bringup')
    plugin_dir = os.path.join(pyrosens_gui_prefix, 'lib')
    existing_plugin_path = os.environ.get('IGN_GUI_PLUGIN_PATH', '')
    combined_plugin_path = plugin_dir if not existing_plugin_path else plugin_dir + os.pathsep + existing_plugin_path

    ld.add_action(SetEnvironmentVariable(
        name='IGN_GAZEBO_GUI_CONFIG_PATH',
        value=gui_config_path
    ))
    ld.add_action(SetEnvironmentVariable(
        name='IGN_GUI_PLUGIN_PATH',
        value=combined_plugin_path
    ))

    # Additional command line arguments
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Flag to Not launch RViz'
    )
    ld.add_action(rviz_launch_arg)
    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        description='Flag to Not launch Nav2'
    )
    ld.add_action(nav2_launch_arg)

    # Load robot_description and start robot_state_publisher
    robot_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path,
                                       'urdf_drone',
                                       'parrot.urdf.xacro'])]),
        value_type=str)
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                          'use_sim_time': use_sim_time
                                      }])
    ld.add_action(robot_state_publisher_node)

    # Publish odom -> base_link transform **using robot_localization**
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([config_path,
                                          'robot_localization.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_localization_node)

    # Start Gazebo to simulate the robot in the chosen world
    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='test_world',
        description='Which world to load',
        choices=['simple_trees', 'large_demo', 'test_world_resize', 'test_world', 'multi_fire']
    )
    ld.add_action(world_launch_arg)
    gui_config_flag = f' --gui-config {gui_config_path}'
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                             'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([pkg_path,
                                               'worlds',
                                               [LaunchConfiguration('world'), '.sdf']]),
                         ' -r',
                         gui_config_flag]}.items()
    )
    ld.add_action(gazebo)

    # Spawn robot in Gazebo
    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description', '-z', '2.0'] # z is height above ground
    )
    ld.add_action(robot_spawner)

    # Bridge topics between gazebo and ROS2
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path,
                                                          'gazebo_bridge.yaml']),
                    'use_sim_time': use_sim_time}]
    )
    ld.add_action(gazebo_bridge)

    # rviz2 visualises data
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path,
                                               'pyrosens.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # Nav2 enables mapping and waypoint following
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path,
                              'launch',
                              'pyrosens_navigation.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    # Mission management stack (proxy + command bridge)
    mission_params = PathJoinSubstitution(
        [FindPackageShare("pyrosens_mission"), "config", "mission_params.yaml"]
    )

    mission_manager = Node(
        package="pyrosens_mission",
        executable="main_mission",
        name="mission_manager",
        output="screen",
        parameters=[
            mission_params,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {
                "downstream_nav_action_name": "/navigate_to_pose",
                "upstream_nav_action_name": "/mission/navigate_to_pose",
            },
        ],
    )
    ld.add_action(mission_manager)

    mission_cmd_bridge = Node(
        package="pyrosens_mission",
        executable="mission_cmd_bridge",
        name="mission_cmd_bridge",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    ld.add_action(mission_cmd_bridge)

    return ld
