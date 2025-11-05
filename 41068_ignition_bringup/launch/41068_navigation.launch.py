from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    config_path = PathJoinSubstitution([FindPackageShare('41068_ignition_bringup'), 'config'])

    # Additional command line arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    nav_namespace = LaunchConfiguration('namespace')
    nav_namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace to launch Nav2 under (empty for none).'
    )
    use_namespace = LaunchConfiguration('use_namespace')
    use_namespace_launch_arg = DeclareLaunchArgument(
        'use_namespace',
        default_value='False',
        description='Whether Nav2 bringup should apply the namespace.'
    )

    # Start Simultaneous Localisation and Mapping (SLaM)
    slam = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('slam_toolbox'),
                             'launch', 'online_async_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': PathJoinSubstitution([config_path, 'slam_params.yaml'])
        }.items()
    )

    # Start Navigation Stack
    navigation = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([config_path, 'nav2_params.yaml']),
            'namespace': nav_namespace,
            'use_namespace': use_namespace,
        }.items()
    )

    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(nav_namespace_launch_arg)
    ld.add_action(use_namespace_launch_arg)
    ld.add_action(slam)
    ld.add_action(navigation)

    return ld
