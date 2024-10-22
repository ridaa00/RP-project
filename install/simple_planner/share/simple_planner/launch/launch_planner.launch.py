from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events import matches_node_name

def generate_launch_description():
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': '/home/zeeshan/rida_ws/maps/map.yaml'}],
        namespace=''
    )

    return LaunchDescription([
        map_server_node,

        # Automatically configure and activate map_server after launch
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=map_server_node,
                on_start=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_node_name('/map_server'),
                            transition_id=1  # Transition to configure
                        )
                    )
                ]
            )
        ),
        RegisterEventHandler(
            event_handler=OnStateTransition(
                target_lifecycle_node=map_server_node,
                goal_state='configuring',
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_node_name('/map_server'),
                            transition_id=3  # Transition to activate
                        )
                    )
                ]
            )
        ),

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
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/zeeshan/ros2_ws/install/simple_planner/share/simple_planner/config/visualization.rviz'],
            output='screen'
        ),
    ])

