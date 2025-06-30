from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PORT_NAME = "port"
IDS_NAME = "ids"
SIDE_NAME = "side"
PUBLISH_DESCRIPTION_NAME = "publish_description"


# OpaqueFunction 将调用的函数
def launch_setup(context, *_, **__):
    port = LaunchConfiguration(PORT_NAME).perform(context)
    ids = LaunchConfiguration(IDS_NAME).perform(context)
    side = LaunchConfiguration(SIDE_NAME).perform(context)
    publish_description = LaunchConfiguration(PUBLISH_DESCRIPTION_NAME).perform(context)

    xacro_file_path = (
        Path(get_package_share_directory("leap_hand_description"))
        / "robots"
        / f"leap_hand_{side}.urdf.xacro"
    )

    if not xacro_file_path.exists():
        raise FileNotFoundError(f"XACRO file not found:{xacro_file_path}")

    robot_description_raw = xacro.process_file(
        xacro_file_path.as_posix()
    ).toxml()  # type: ignore
    namespace = "leap_hand_" + side

    nodes = []
    if publish_description:
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=namespace,
                output="screen",
                parameters=[{"robot_description": robot_description_raw}],
            )
        )
    nodes.append(
        Node(
            package="leap_hand",
            executable="leaphand_node.py",
            name="leaphand_node",
            namespace=namespace,
            emulate_tty=True,
            output="screen",
            parameters=[
                {"kP": 800.0},
                {"kI": 0.0},
                {"kD": 200.0},
                {"curr_lim": 500.0},
                {"port": port},
                {"ids": ids},
                {"side": side},
            ],
        )
    )

    return nodes


def generate_launch_description():
    default_id_str = ",".join(str(i) for i in range(16))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                PORT_NAME,
                default_value="/dev/leap_left_hand",
                description="Serial port for leaphand_node",
            ),
            DeclareLaunchArgument(
                IDS_NAME,
                default_value=default_id_str,
                description="Motor IDs for leaphand_node",
            ),
            DeclareLaunchArgument(
                SIDE_NAME,
                default_value="left",
                choices=["left", "right"],
                description="Side of the hand (left or right)",
            ),
            DeclareLaunchArgument(
                PUBLISH_DESCRIPTION_NAME,
                default_value="true",
                description="Publish the robot description through topic.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
