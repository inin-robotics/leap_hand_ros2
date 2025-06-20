import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    leap_xacro_file = os.path.join(
        get_package_share_directory("leap_hand_description"),
        "robots",
        "leap_hand_left.urdf.xacro",
    )
    robot_description = Command([FindExecutable(name="xacro"), " ", leap_xacro_file])

    rviz_file = os.path.join(
        get_package_share_directory("leap_hand_description"),
        "rviz",
        "visualize_leap_left.rviz",
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", rviz_file],
            ),
        ]
    )
