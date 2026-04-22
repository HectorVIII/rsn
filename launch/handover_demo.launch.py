from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    xarm_controller = Node(
        package='rsn',
        executable='xarm_controller_node',
        name='xarm_controller_node',
        output='screen',
        parameters=[
            '/home/huitao/ros2_ws/src/rsn/config/xarm_controller_params.yaml'
        ]
    )

    instrument_detection = Node(
        package='rsn',
        executable='instrument_detection_node',
        name='instrument_detection_node',
        output='screen',
        parameters=[{
            'camera_serial': 27204693,
            'camera_fps': 30,
            'voice_target_topic': '/voice_target_instrument',
            'publish_topic': '/instrument_grasp_pose_base',

            # Detection / publish behavior
            'show_viewer': True,
            'publish_once_then_stop': True,
            'exit_delay_after_publish': 0.5,

            # Keep these here only if you want launch-side override.
            # Otherwise they can stay inside the node defaults or a yaml later.
            'conf_threshold': 0.85,
            'min_mask_area': 500,
            'mask_threshold': 0.50,
            'close_kernel': 7,
            'open_kernel': 3,
            'center_ema_alpha': 0.18,
            'target_stable_frames': 8,

            # Current calibrated offsets
            'x_offset_m': 0.028,
            'y_offset_m': -0.015,
            'grasp_z_base_m': 0.0129,
            'hover_offset_m': 0.025,
        }]
    )

    voice_command = Node(
        package='rsn',
        executable='voice_command_node',
        name='voice_command_node',
        output='screen',
        parameters=[{
            'publish_topic': '/voice_target_instrument',
            'raw_text_topic': '/voice_recognized_text',

            'sample_rate': 16000,
            'channels': 1,
            'block_duration': 0.1,
            'silence_threshold': 0.01,
            'silence_seconds_end': 1.0,
            'max_record_seconds': 5.0,

            'enable_debug_log': True,
            'publish_raw_text': True,
        }]
    )

    demo_coordinator = Node(
        package='rsn',
        executable='demo_coordinator',
        name='demo_coordinator',
        output='screen',
        parameters=[{
            # Voice waiting
            'voice_target_topic': '/voice_target_instrument',
            'voice_wait_timeout_sec': 30.0,
            'voice_poll_interval_sec': 0.2,

            # Instrument move retry
            'instrument_move_max_attempts': 40,
            'instrument_move_retry_interval_sec': 0.5,

            # Hand move retry
            'hand_move_max_attempts': 20,
            'hand_move_retry_interval_sec': 1.0,

            # Sleep timings
            'sleep_after_move_p0_sec': 1.0,
            'sleep_after_open_gripper_sec': 1.0,
            'sleep_after_start_instrument_detection_sec': 1.0,
            'sleep_after_move_to_instrument_sec': 1.0,
            'sleep_after_close_gripper_sec': 1.0,
            'sleep_after_lift_after_grasp_sec': 1.0,

            'sleep_after_instrument_node_exit_sec': 1.5,
            'sleep_after_start_hand_detection_sec': 0.5,
            'sleep_after_move_to_hand_sec': 1.0,
            'sleep_after_wait_for_release_sec': 0.5,
            'sleep_after_release_open_sec': 0.5,
            'sleep_after_retreat_sec': 0.5,
            'sleep_after_final_return_p0_sec': 0.5,

            # Auto launch hand node AFTER instrument node exits
            'auto_launch_hand_node': True,
            'hand_node_package': 'rsn',
            'hand_node_executable': 'zed_hand_node',
            'hand_node_service_wait_timeout_sec': 20.0,
            'hand_node_launch_delay_sec': 0.5,
        }]
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