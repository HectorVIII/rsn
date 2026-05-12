"""Launch low-level RSN nodes for FlexBE control."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for FlexBE low-level providers."""
    package_share_dir = get_package_share_directory('rsn')
    xarm_params_path = os.path.join(
        package_share_dir,
        'config',
        'xarm_controller_params.yaml'
    )
    instrument_detection_params_path = os.path.join(
        package_share_dir,
        'config',
        'instrument_detection_params.yaml'
    )
    voice_command_params_path = os.path.join(
        package_share_dir,
        'config',
        'voice_command_params.yaml'
    )
    zed_hand_params_path = os.path.join(
        package_share_dir,
        'config',
        'zed_hand_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='rsn',
            executable='xarm_controller_node',
            name='xarm_controller_node',
            output='screen',
            parameters=[xarm_params_path]
        ),
        Node(
            package='rsn',
            executable='instrument_detection_node',
            name='instrument_detection_node',
            output='screen',
            parameters=[instrument_detection_params_path]
        ),
        Node(
            package='rsn',
            executable='voice_command_node',
            name='voice_command_node',
            output='screen',
            parameters=[voice_command_params_path]
        ),
        Node(
            package='rsn',
            executable='zed_hand_node',
            name='zed_hand_node',
            output='screen',
            parameters=[zed_hand_params_path]
        ),
    ])
