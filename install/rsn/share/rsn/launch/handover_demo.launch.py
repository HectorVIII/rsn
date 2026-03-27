from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rsn',
            executable='xarm_controller_node',
            name='xarm_controller_node',
            output='screen',
            parameters=['/home/huitao/ros2_ws/src/rsn/config/xarm_controller_params.yaml']
        ),
    ])
