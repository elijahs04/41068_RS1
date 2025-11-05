from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
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

    # RViz: auto-launch with a custom or default config
    rviz = ExecuteProcess(
        cmd=['rviz2', '-d', '$(find pyrosens_hotspotDetection)/rviz/hotspot_detection2.rviz'],
        output='screen'
    )

    return LaunchDescription([detector, mapper, rviz])
