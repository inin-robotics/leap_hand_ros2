from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    leap_hand_ros2_pkg_share = get_package_share_directory("leap_hand")
    leap_launch_file = PathJoinSubstitution(
        [leap_hand_ros2_pkg_share, "launch", "leap.launch.py"]
    )

    left_port_name = "left_port"
    left_ids_name = "left_ids"
    right_port_name = "right_port"
    right_ids_name = "right_ids"
    left_port_arg = DeclareLaunchArgument(
        left_port_name,
        default_value="/dev/leap_left_hand",
        description="Serial port for the left leaphand_node",
    )
    left_ids_arg = DeclareLaunchArgument(
        left_ids_name,
        default_value="0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15",
        description="Motor IDs for the left leaphand_node",
    )
    right_port_arg = DeclareLaunchArgument(
        right_port_name,
        default_value="/dev/leap_right_hand",
        description="Serial port for the right leaphand_node",
    )
    right_ids_arg = DeclareLaunchArgument(
        right_ids_name,
        default_value="0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15",
        description="Motor IDs for the right leaphand_node",
    )

    left_hand_launch_group = GroupAction(
        actions=[
            PushRosNamespace("leap_hand_left"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(leap_launch_file),
                launch_arguments={
                    "port": LaunchConfiguration(left_port_name),
                    "ids": LaunchConfiguration(left_ids_name),
                    "side": "left",
                }.items(),
            ),
        ]
    )

    right_hand_launch_group = GroupAction(
        actions=[
            PushRosNamespace("leap_hand_right"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(leap_launch_file),
                launch_arguments={
                    "port": LaunchConfiguration(right_port_name),
                    "ids": LaunchConfiguration(right_ids_name),
                    "side": "right",
                }.items(),
            ),
        ]
    )

    return LaunchDescription(
        [
            left_port_arg,
            left_ids_arg,
            right_port_arg,
            right_ids_arg,
            left_hand_launch_group,
            right_hand_launch_group,
        ]
    )
