import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pyrosens_gui_share = get_package_share_directory('pyrosens_gui')
    pyrosens_gui_prefix = get_package_prefix('pyrosens_gui')

    gui_config = os.path.join(pyrosens_gui_share, 'gui', 'pyrosens_with_panel.gui')
    pyrosens_world_share = get_package_share_directory('pyrosens_world')
    default_world = os.path.join(pyrosens_world_share, 'test_world.sdf')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Absolute path to the Gazebo world SDF file.',
    )

    existing_plugin_path = os.environ.get('IGN_GUI_PLUGIN_PATH', '')
    plugin_lib_dir = os.path.join(pyrosens_gui_prefix, 'lib')
    combined_plugin_path = plugin_lib_dir
    if existing_plugin_path:
        combined_plugin_path = plugin_lib_dir + os.pathsep + existing_plugin_path

    set_gui_config_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_GUI_CONFIG_PATH',
        value=gui_config,
    )

    prepend_plugin_path = SetEnvironmentVariable(
        name='IGN_GUI_PLUGIN_PATH',
        value=combined_plugin_path,
    )

    launch_gazebo = ExecuteProcess(
        cmd=[
            'ign',
            'gazebo',
            LaunchConfiguration('world'),
            '--gui-config',
            gui_config,
        ],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        set_gui_config_env,
        prepend_plugin_path,
        launch_gazebo,
    ])
