from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
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
    zed_hand_params_path = os.path.join(
        package_share_dir,
        'config',
        'zed_hand_params.yaml'
    )
    demo_coordinator_params_path = os.path.join(
        package_share_dir,
        'config',
        'demo_coordinator_params.yaml'
    )
    voice_command_params_path = os.path.join(
        package_share_dir,
        'config',
        'voice_command_params.yaml'
    )

    xarm_controller = Node(
        package='rsn',
        executable='xarm_controller_node',
        name='xarm_controller_node',
        output='screen',
        parameters=[
            xarm_params_path
        ]
    )

    instrument_detection = Node(
        package='rsn',
        executable='instrument_detection_node',
        name='instrument_detection_node',
        output='screen',
        parameters=[
            instrument_detection_params_path
        ]
    )

    voice_command = Node(
        package='rsn',
        executable='voice_command_node',
        name='voice_command_node',
        output='screen',
        parameters=[
            voice_command_params_path
        ]
    )

    demo_coordinator = Node(
        package='rsn',
        executable='demo_coordinator',
        name='demo_coordinator',
        output='screen',
        parameters=[
            demo_coordinator_params_path,
            {'hand_node_params_file': zed_hand_params_path},
        ]
    )

    return LaunchDescription([
        xarm_controller,
        instrument_detection,
        voice_command,

        # Wait a little so core services are up before coordinator starts
        TimerAction(
            period=2.0,
            actions=[demo_coordinator],
        ),
    ])
