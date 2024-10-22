from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_planner',
            executable='map_handler',
            name='map_handler',
            output='screen'
        ),
        Node(
            package='simple_planner',
            executable='path_planner',
            name='path_planner',
            output='screen'
        ),
    ])
