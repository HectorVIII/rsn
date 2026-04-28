import time
import subprocess
from typing import Optional

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String


class DemoCoordinator(Node):
    def __init__(self):
        super().__init__('demo_coordinator')

        # ===== Parameters =====
        self.declare_parameter('voice_target_topic', '/voice_target_instrument')
        self.declare_parameter('voice_wait_timeout_sec', 30.0)
        self.declare_parameter('voice_poll_interval_sec', 0.2)

        # ===== Instrument move retry =====
        self.declare_parameter('instrument_move_max_attempts', 40)
        self.declare_parameter('instrument_move_retry_interval_sec', 0.5)

        # ===== Hand move retry =====
        self.declare_parameter('hand_move_max_attempts', 20)
        self.declare_parameter('hand_move_retry_interval_sec', 1.0)

        # ===== Sleep timings =====
        self.declare_parameter('sleep_after_move_p0_sec', 1.0)
        self.declare_parameter('sleep_after_open_gripper_sec', 1.0)
        self.declare_parameter('sleep_after_start_instrument_detection_sec', 1.0)
        self.declare_parameter('sleep_after_move_to_instrument_sec', 1.0)
        self.declare_parameter('sleep_after_close_gripper_sec', 1.0)
        self.declare_parameter('sleep_after_lift_after_grasp_sec', 1.0)

        self.declare_parameter('sleep_after_instrument_node_exit_sec', 1.5)
        self.declare_parameter('sleep_after_start_hand_detection_sec', 0.5)
        self.declare_parameter('sleep_after_move_to_hand_sec', 1.0)
        self.declare_parameter('sleep_after_wait_for_release_sec', 0.5)
        self.declare_parameter('sleep_after_release_open_sec', 0.5)
        self.declare_parameter('sleep_after_retreat_sec', 0.5)
        self.declare_parameter('sleep_after_final_return_p0_sec', 0.5)

        # ===== Hand node auto launch =====
        self.declare_parameter('auto_launch_hand_node', True)
        self.declare_parameter('hand_node_package', 'rsn')
        self.declare_parameter('hand_node_executable', 'zed_hand_node')
        self.declare_parameter('hand_node_params_file', '')
        self.declare_parameter('hand_node_service_wait_timeout_sec', 20.0)
        self.declare_parameter('hand_node_launch_delay_sec', 0.5)

        # ===== Read parameters =====
        self.voice_target_topic = str(self.get_parameter('voice_target_topic').value)
        self.voice_wait_timeout_sec = float(self.get_parameter('voice_wait_timeout_sec').value)
        self.voice_poll_interval_sec = float(self.get_parameter('voice_poll_interval_sec').value)

        self.instrument_move_max_attempts = int(
            self.get_parameter('instrument_move_max_attempts').value
        )
        self.instrument_move_retry_interval_sec = float(
            self.get_parameter('instrument_move_retry_interval_sec').value
        )

        self.hand_move_max_attempts = int(self.get_parameter('hand_move_max_attempts').value)
        self.hand_move_retry_interval_sec = float(
            self.get_parameter('hand_move_retry_interval_sec').value
        )

        self.sleep_after_move_p0_sec = float(self.get_parameter('sleep_after_move_p0_sec').value)
        self.sleep_after_open_gripper_sec = float(
            self.get_parameter('sleep_after_open_gripper_sec').value
        )
        self.sleep_after_start_instrument_detection_sec = float(
            self.get_parameter('sleep_after_start_instrument_detection_sec').value
        )
        self.sleep_after_move_to_instrument_sec = float(
            self.get_parameter('sleep_after_move_to_instrument_sec').value
        )
        self.sleep_after_close_gripper_sec = float(
            self.get_parameter('sleep_after_close_gripper_sec').value
        )
        self.sleep_after_lift_after_grasp_sec = float(
            self.get_parameter('sleep_after_lift_after_grasp_sec').value
        )

        self.sleep_after_instrument_node_exit_sec = float(
            self.get_parameter('sleep_after_instrument_node_exit_sec').value
        )
        self.sleep_after_start_hand_detection_sec = float(
            self.get_parameter('sleep_after_start_hand_detection_sec').value
        )
        self.sleep_after_move_to_hand_sec = float(
            self.get_parameter('sleep_after_move_to_hand_sec').value
        )
        self.sleep_after_wait_for_release_sec = float(
            self.get_parameter('sleep_after_wait_for_release_sec').value
        )
        self.sleep_after_release_open_sec = float(
            self.get_parameter('sleep_after_release_open_sec').value
        )
        self.sleep_after_retreat_sec = float(self.get_parameter('sleep_after_retreat_sec').value)
        self.sleep_after_final_return_p0_sec = float(
            self.get_parameter('sleep_after_final_return_p0_sec').value
        )

        self.auto_launch_hand_node = bool(self.get_parameter('auto_launch_hand_node').value)
        self.hand_node_package = str(self.get_parameter('hand_node_package').value)
        self.hand_node_executable = str(self.get_parameter('hand_node_executable').value)
        self.hand_node_params_file = str(self.get_parameter('hand_node_params_file').value)
        self.hand_node_service_wait_timeout_sec = float(
            self.get_parameter('hand_node_service_wait_timeout_sec').value
        )
        self.hand_node_launch_delay_sec = float(
            self.get_parameter('hand_node_launch_delay_sec').value
        )

        # ===== Voice target state =====
        self.latest_voice_target = None
        self.voice_target_sub = self.create_subscription(
            String,
            self.voice_target_topic,
            self.voice_target_callback,
            10
        )

        # ===== Track launched hand node process =====
        self.hand_node_process: Optional[subprocess.Popen] = None

        # ===== Service clients =====
        self.move_to_p0_client = self.create_client(Trigger, 'move_to_p0')
        self.start_instrument_detection_client = self.create_client(
            Trigger, 'start_instrument_detection'
        )
        self.move_to_instrument_client = self.create_client(Trigger, 'move_to_instrument')
        self.lift_after_grasp_client = self.create_client(Trigger, 'lift_after_grasp')

        # Hand-related services: create now, but do not wait here
        self.start_hand_detection_client = self.create_client(Trigger, 'start_hand_detection')
        self.move_to_hand_client = self.create_client(Trigger, 'move_to_hand')

        self.wait_for_release_client = self.create_client(Trigger, 'wait_for_release')
        self.retreat_after_release_client = self.create_client(
            Trigger, 'retreat_after_release'
        )
        self.open_client = self.create_client(Trigger, 'open_gripper')
        self.close_client = self.create_client(Trigger, 'close_gripper')

        self.get_logger().info('Waiting for core services...')

        self._wait_for_service_or_raise(self.move_to_p0_client, 'move_to_p0')
        self._wait_for_service_or_raise(
            self.start_instrument_detection_client,
            'start_instrument_detection'
        )
        self._wait_for_service_or_raise(self.move_to_instrument_client, 'move_to_instrument')
        self._wait_for_service_or_raise(self.lift_after_grasp_client, 'lift_after_grasp')
        self._wait_for_service_or_raise(self.wait_for_release_client, 'wait_for_release')
        self._wait_for_service_or_raise(
            self.retreat_after_release_client,
            'retreat_after_release'
        )
        self._wait_for_service_or_raise(self.open_client, 'open_gripper')
        self._wait_for_service_or_raise(self.close_client, 'close_gripper')

        self.get_logger().info(
            'Core services are available. Hand services will be waited only after hand node is launched.'
        )

    def _wait_for_service_or_raise(self, client, service_name: str, timeout_sec: float = 1.0):
        while rclpy.ok():
            if client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().info(f'{service_name} service is available.')
                return
            self.get_logger().info(f'{service_name} service not available, waiting...')

        raise RuntimeError(f'Interrupted while waiting for service: {service_name}')

    def _wait_for_service_with_deadline(self, client, service_name: str, timeout_sec: float) -> bool:
        self.get_logger().info(
            f'Waiting for {service_name} service (timeout={timeout_sec:.1f}s)...'
        )
        start_time = time.time()

        while rclpy.ok():
            if client.wait_for_service(timeout_sec=0.5):
                self.get_logger().info(f'{service_name} service is available.')
                return True

            if (time.time() - start_time) > timeout_sec:
                self.get_logger().error(
                    f'Timeout while waiting for {service_name} service after {timeout_sec:.1f}s'
                )
                return False

        return False

    def _launch_hand_node_if_needed(self) -> bool:
        if not self.auto_launch_hand_node:
            self.get_logger().info(
                'auto_launch_hand_node=False, assuming zed_hand_node will be started externally.'
            )
            return True

        if self.hand_node_process is not None:
            poll_result = self.hand_node_process.poll()
            if poll_result is None:
                self.get_logger().info('Hand node process is already running.')
                return True

            self.get_logger().warn(
                f'Previous hand node process already exited with code {poll_result}. Relaunching...'
            )
            self.hand_node_process = None

        cmd = ['ros2', 'run', self.hand_node_package, self.hand_node_executable]
        if self.hand_node_params_file:
            cmd.extend(['--ros-args', '--params-file', self.hand_node_params_file])

        self.get_logger().info(f'Launching hand node: {" ".join(cmd)}')

        try:
            self.hand_node_process = subprocess.Popen(cmd)
        except Exception as e:
            self.get_logger().error(f'Failed to launch hand node: {e}')
            return False

        time.sleep(self.hand_node_launch_delay_sec)
        return True

    def _prepare_hand_detection_phase(self) -> bool:
        self.get_logger().info(
            'Preparing hand-detection phase: waiting for instrument node to release ZED...'
        )
        time.sleep(self.sleep_after_instrument_node_exit_sec)

        if not self._launch_hand_node_if_needed():
            return False

        if not self._wait_for_service_with_deadline(
            self.start_hand_detection_client,
            'start_hand_detection',
            self.hand_node_service_wait_timeout_sec
        ):
            return False

        if not self._wait_for_service_with_deadline(
            self.move_to_hand_client,
            'move_to_hand',
            self.hand_node_service_wait_timeout_sec
        ):
            return False

        return True

    def voice_target_callback(self, msg: String):
        self.latest_voice_target = msg.data.strip()
        self.get_logger().info(f'Received voice target: {self.latest_voice_target}')

    def clear_voice_target(self):
        self.latest_voice_target = None

    def wait_for_voice_target(self):
        self.get_logger().info(
            f'Waiting for voice target on {self.voice_target_topic} '
            f'(timeout={self.voice_wait_timeout_sec:.1f}s)...'
        )

        self.clear_voice_target()
        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=self.voice_poll_interval_sec)

            if self.latest_voice_target is not None:
                self.get_logger().info(f'Voice target received: {self.latest_voice_target}')
                return True

            if (time.time() - start_time) > self.voice_wait_timeout_sec:
                self.get_logger().error(
                    f'Voice target timeout after {self.voice_wait_timeout_sec:.1f}s'
                )
                return False

        return False

    def call_trigger_service(self, client, service_name: str) -> bool:
        request = Trigger.Request()
        future = client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f'{service_name} response: success={response.success}, '
                f'message="{response.message}"'
            )
            return response.success

        self.get_logger().error(f'Failed to call {service_name}')
        return False

    def run_demo(self):
        self.get_logger().info('===== DEMO START =====')

        # Step 1: move to P0
        self.get_logger().info('Step 1: move to P0')
        if not self.call_trigger_service(self.move_to_p0_client, '/move_to_p0'):
            self.get_logger().error('Demo aborted at Step 1.')
            return
        time.sleep(self.sleep_after_move_p0_sec)

        # Step 2: open gripper
        self.get_logger().info('Step 2: open gripper')
        if not self.call_trigger_service(self.open_client, '/open_gripper'):
            self.get_logger().error('Demo aborted at Step 2.')
            return
        time.sleep(self.sleep_after_open_gripper_sec)

        # Step 3: wait for voice command
        self.get_logger().info('Step 3: wait for voice command')
        if not self.wait_for_voice_target():
            self.get_logger().error('Demo aborted at Step 3.')
            return

        # Step 4: start instrument detection
        self.get_logger().info('Step 4: start instrument detection')
        if not self.call_trigger_service(
            self.start_instrument_detection_client,
            '/start_instrument_detection'
        ):
            self.get_logger().error('Demo aborted at Step 4.')
            return
        time.sleep(self.sleep_after_start_instrument_detection_sec)

        # Step 5: move to detected instrument grasp pose (with retry)
        self.get_logger().info('Step 5: move to instrument grasp pose')

        success = False
        for i in range(self.instrument_move_max_attempts):
            self.get_logger().info(
                f'Attempt {i + 1}/{self.instrument_move_max_attempts} to move to instrument'
            )
            if self.call_trigger_service(self.move_to_instrument_client, '/move_to_instrument'):
                success = True
                break
            time.sleep(self.instrument_move_retry_interval_sec)

        if not success:
            self.get_logger().error('Demo aborted at Step 5.')
            return

        time.sleep(self.sleep_after_move_to_instrument_sec)

        # Step 6: close gripper
        self.get_logger().info('Step 6: close gripper')
        if not self.call_trigger_service(self.close_client, '/close_gripper'):
            self.get_logger().error('Demo aborted at Step 6.')
            return
        time.sleep(self.sleep_after_close_gripper_sec)

        # Step 7: lift after grasp
        self.get_logger().info('Step 7: lift after grasp')
        if not self.call_trigger_service(self.lift_after_grasp_client, '/lift_after_grasp'):
            self.get_logger().error('Demo aborted at Step 7.')
            return
        time.sleep(self.sleep_after_lift_after_grasp_sec)

        # Step 8: wait instrument node exit, launch hand node, wait hand services
        self.get_logger().info('Step 8: prepare hand detection phase')
        if not self._prepare_hand_detection_phase():
            self.get_logger().error('Demo aborted at Step 8.')
            return

        # Step 9: start hand detection
        self.get_logger().info('Step 9: start hand detection')
        if not self.call_trigger_service(
            self.start_hand_detection_client,
            '/start_hand_detection'
        ):
            self.get_logger().error('Demo aborted at Step 9.')
            return
        time.sleep(self.sleep_after_start_hand_detection_sec)

        # Step 10: move to detected hand hover pose
        self.get_logger().info('Step 10: move to hand hover pose')

        success = False
        for i in range(self.hand_move_max_attempts):
            self.get_logger().info(
                f'Attempt {i + 1}/{self.hand_move_max_attempts} to move to hand'
            )
            if self.call_trigger_service(self.move_to_hand_client, '/move_to_hand'):
                success = True
                break
            time.sleep(self.hand_move_retry_interval_sec)

        if not success:
            self.get_logger().error('Demo aborted at Step 10.')
            return
        time.sleep(self.sleep_after_move_to_hand_sec)

        # Step 11: wait for release trigger
        self.get_logger().info('Step 11: wait for release trigger')
        if not self.call_trigger_service(self.wait_for_release_client, '/wait_for_release'):
            self.get_logger().error('Demo aborted at Step 11.')
            return
        time.sleep(self.sleep_after_wait_for_release_sec)

        # Step 12: open gripper to release
        self.get_logger().info('Step 12: open gripper to release')
        if not self.call_trigger_service(self.open_client, '/open_gripper'):
            self.get_logger().error('Demo aborted at Step 12.')
            return
        time.sleep(self.sleep_after_release_open_sec)

        # Step 13: retreat after release
        self.get_logger().info('Step 13: retreat after release')
        if not self.call_trigger_service(
            self.retreat_after_release_client,
            '/retreat_after_release'
        ):
            self.get_logger().error('Demo aborted at Step 13.')
            return
        time.sleep(self.sleep_after_retreat_sec)

        # Step 14: return to P0
        self.get_logger().info('Step 14: return to P0')
        if not self.call_trigger_service(self.move_to_p0_client, '/move_to_p0'):
            self.get_logger().error('Demo aborted at Step 14.')
            return
        time.sleep(self.sleep_after_final_return_p0_sec)

        self.get_logger().info('===== DEMO FINISHED SUCCESSFULLY =====')

    def destroy_node(self):
        self.get_logger().info('Shutting down demo_coordinator...')

        if self.hand_node_process is not None:
            try:
                poll_result = self.hand_node_process.poll()
                if poll_result is None:
                    self.get_logger().info('Hand node process is still running. Terminating it...')
                    self.hand_node_process.terminate()
                    try:
                        self.hand_node_process.wait(timeout=3.0)
                    except subprocess.TimeoutExpired:
                        self.get_logger().warn('Hand node did not terminate in time. Killing it...')
                        self.hand_node_process.kill()
                        self.hand_node_process.wait(timeout=3.0)
                else:
                    self.get_logger().info(
                        f'Hand node process already exited with code {poll_result}.'
                    )
            except Exception as e:
                self.get_logger().warn(f'Exception while cleaning hand node process: {e}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DemoCoordinator()

    try:
        node.run_demo()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
