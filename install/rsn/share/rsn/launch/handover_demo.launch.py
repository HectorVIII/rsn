from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    xarm_controller = Node(
        package='rsn',
        executable='xarm_controller_node',
        name='xarm_controller_node',
        output='screen',
        parameters=['/home/huitao/ros2_ws/src/rsn/config/xarm_controller_params.yaml']
    )

    zed_hand = Node(
        package='rsn',
        executable='zed_hand_node',
        name='zed_hand_node',
        output='screen',
        parameters=[{
            'camera_fps': 30,
            'publish_topic': '/right_hand_pose_base',
            'stability_duration': 2.0,
            'stability_threshold': 0.05,
            'exit_delay_after_publish': 1.0,
            'show_viewer': True,
        }]
    )

    demo_coordinator = Node(
        package='rsn',
        executable='demo_coordinator',
        name='demo_coordinator',
        output='screen'
    )

    return LaunchDescription([
        xarm_controller,
        zed_hand,
        TimerAction(
            period=2.0,
            actions=[demo_coordinator],
        ),
    ])
