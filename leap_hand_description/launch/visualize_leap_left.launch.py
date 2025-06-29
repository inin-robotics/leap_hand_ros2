from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    xacro_file_path = (
        Path(get_package_share_directory("leap_hand_description"))
        / "robots"
        / "leap_hand_left.urdf.xacro"
    )
    if not xacro_file_path.exists():
        raise FileNotFoundError(f"XACRO file not found:{xacro_file_path}")

    robot_description = xacro.process_file(
        xacro_file_path.as_posix(), mappings={"connected_to": ""}
    ).toxml()  # type: ignore

    rviz_file_path = (
        Path(get_package_share_directory("leap_hand_description"))
        / "rviz"
        / "visualize_leap_left.rviz"
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
                arguments=["--display-config", rviz_file_path.as_posix()],
            ),
        ]
    )
