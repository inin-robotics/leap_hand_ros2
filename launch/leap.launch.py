from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/leap_left_hand',
        description='Serial port for leaphand_node'
    )
    ids_arg = DeclareLaunchArgument(
        'ids',
        default_value='0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15',
        description='Motor IDs for leaphand_node'
    )
    side_arg = DeclareLaunchArgument(
        'side',
        default_value='left',
        description='Side of the hand (left or right)'
    )
    port = LaunchConfiguration('port')
    ids = LaunchConfiguration('ids')
    side = LaunchConfiguration('side')
    return LaunchDescription([
        port_arg,
        ids_arg,
        Node(
            package='leap_hand',
            executable='leaphand_node.py',
            name='leaphand_node',
            emulate_tty=True,
            output='screen',
            parameters=[
                {'kP': 800.0},
                {'kI': 0.0},
                {'kD': 200.0},
                {'curr_lim': 500.0},
                {'port': port},
                {'ids': ids},
                {'side': side},
            ]
        ),
    ])
