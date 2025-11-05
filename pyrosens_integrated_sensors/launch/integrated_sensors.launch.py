from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the default PyroSens sensor stack."""
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time for all sensor nodes.",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    nodes = [
        Node(
            package="pyrosens_integrated_sensors",
            executable="wind_sim_node",
            name="wind_sim",
            output="screen",
            parameters=[{"period_ms": 500, "use_sim_time": use_sim_time}],
        ),
        Node(
            package="pyrosens_integrated_sensors",
            executable="wind_sensor_node",
            name="wind_sensor",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="pyrosens_integrated_sensors",
            executable="wind_interpolation_node",
            name="wind_interpolation",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="pyrosens_integrated_sensors",
            executable="thermal_hotspot_node",
            name="thermal_hotspot",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="pyrosens_integrated_sensors",
            executable="hotspot_mapper_node",
            name="hotspot_mapper",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    return LaunchDescription([use_sim_time_arg, *nodes])
