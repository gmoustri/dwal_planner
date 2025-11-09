from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'dwal_planner'
    default_params = os.path.join(
        get_package_share_directory(pkg_name), 'config/dwal_params.yaml'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to ROS 2 parameters file.'
    )

    params_file = LaunchConfiguration('params_file')

    ns_group = GroupAction([
        PushRosNamespace('dwal_planner'),

        Node(
            package=pkg_name,
            executable='dwal_generator',          
            name='dwal_generator',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
        ),
        Node(
            package=pkg_name,
            executable='dwal_clustering',
            name='dwal_clustering',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
        ),
    ])

    return LaunchDescription([
        params_file_arg,
        ns_group
    ])
