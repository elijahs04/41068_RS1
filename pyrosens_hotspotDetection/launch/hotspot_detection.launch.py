from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = LaunchDescription()

    pkg_path = FindPackageShare('pyrosens_hotspotDetection')
    default_rviz = PathJoinSubstitution([pkg_path, 'rviz', 'hotspot_detect_n_map.rviz'])

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='True')
    rviz_cfg_arg = DeclareLaunchArgument('rviz_config', default_value=default_rviz)

    ld.add_action(use_sim_time_arg)
    ld.add_action(rviz_arg)
    ld.add_action(rviz_cfg_arg)

    # Node: detects hotspots and publishes point clouds
    detector = Node(
        package='pyrosens_hotspotDetection',
        executable='thermalHotspotNode',
        name='thermal_hotspot_node',
        output='screen',
        parameters=[{
            'thermal_topic': '/thermal_camera/image',
            'depth_topic': '/camera/depth/image',
            'frame_id': 'thermal_camera_optical_frame',
            'target_frame': 'map',
            'plane_z': 0.0,
            'hfov_rad': 2.0944,
            'depth_hfov_rad': 2.0944,
            'hot_temp_c': 150.0,
            'use_percentile': False,
            'publish_overlay': True,
            'depth_stride': 3
        }]
    )
    ld.add_action(detector)


    # Node: accumulates the hotspot points into a heatmap
    mapper = Node(
        package='pyrosens_hotspotDetection',
        executable='hotspotMapperNode',
        name='hotspot_mapper',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'origin_x': -50.0,
            'origin_y': -50.0,
            'resolution': 0.25,
            'width': 400,
            'height': 400,
            'sigma_m': 0.75,
            'decay_rate_hz': 2.0,
            'decay_half_life_s': 30.0,
            'add_weight': 1.0
        }]
    )
    ld.add_action(mapper)    

    # rviz2 visualises data
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    return ld