import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    robot_name = "rrbot"
    robot_model_file = "rrbot.xacro"
    package_name = "rrbot"

    rviz_config = os.path.join(get_package_share_directory(
        "rrbot"), "rviz", "rrbot.rviz")

    robot_description = os.path.join(get_package_share_directory(
        package_name), "urdf", robot_model_file)

    robot_description_config = xacro.process_file(robot_description)

    controller_config = os.path.join(
        get_package_share_directory(
            "hoverboard_driver"), "config", "controllers.yaml"
    )

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description_config.toxml()}, controller_config],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
        ),

        # Node(
        #     package="controller_manager",
        #     executable="spawner",
        #     arguments=["joint_state_broadcaster",
        #                "--controller-manager", "/controller_manager"],
        # ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["hoverboard_joint_publisher",
                       "-c", "/controller_manager"],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["hoverboard_velocity_controller",
                       "-c", "/controller_manager"],
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config.toxml()}],
            output="screen"),

        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     name="rviz2",
        #     arguments=["-d", rviz_config],
        #     output={
        #         "stdout": "screen",
        #         "stderr": "log",
        #     },
        # )

    ])
