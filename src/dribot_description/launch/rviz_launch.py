import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    your_pkg_dir = get_package_share_directory('dribot_description')

    rviz_config_file = get_package_share_path('dribot_description') / 'rviz' / 'rviz.rviz'

    return LaunchDescription([
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(your_pkg_dir, 'launch', 'load_description_launch.py')
            )
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', str(rviz_config_file)],
        )
    ])
