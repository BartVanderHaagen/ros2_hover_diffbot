from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    urdf_file = get_package_share_path('dribot_description') / 'urdf' / 'dribot_description.xacro'

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', str(urdf_file)]), value_type=str
                )
            }]
        ),
    ]);
