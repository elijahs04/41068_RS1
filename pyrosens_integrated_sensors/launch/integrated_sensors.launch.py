from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the default PyroSens sensor stack."""
    nodes = [
        Node(
            package="pyrosens_integrated_sensors",
            executable="wind_sim_node",
            name="wind_sim",
            output="screen",
            parameters=[{"period_ms": 500}],
        ),
        Node(
            package="pyrosens_integrated_sensors",
            executable="wind_sensor_node",
            name="wind_sensor",
            output="screen",
        ),
        Node(
            package="pyrosens_integrated_sensors",
            executable="wind_interpolation_node",
            name="wind_interpolation",
            output="screen",
        ),
        Node(
            package="pyrosens_integrated_sensors",
            executable="thermal_hotspot_node",
            name="thermal_hotspot",
            output="screen",
        ),
        Node(
            package="pyrosens_integrated_sensors",
            executable="hotspot_mapper_node",
            name="hotspot_mapper",
            output="screen",
        ),
    ]

    return LaunchDescription(nodes)
