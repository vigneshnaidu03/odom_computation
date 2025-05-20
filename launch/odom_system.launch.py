from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odom_computation',
            executable='wheel_tick_pub',
            name='wheel_tick_publisher',
            output='screen'
        ),
        Node(
            package='odom_computation',
            executable='odom_calc_node',
            name='odom_calculator',
            output='screen'
        ),
    ])
