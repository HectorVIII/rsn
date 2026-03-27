import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class DemoCoordinator(Node):
    def __init__(self):
        super().__init__('demo_coordinator')

        # Service clients
        self.move_to_p0_client = self.create_client(Trigger, 'move_to_p0')
        self.move_to_p1_client = self.create_client(Trigger, 'move_to_p1')
        self.start_hand_detection_client = self.create_client(Trigger, 'start_hand_detection')
        self.move_to_hand_client = self.create_client(Trigger, 'move_to_hand')
        self.wait_for_release_client = self.create_client(Trigger, 'wait_for_release')
        self.retreat_after_release_client = self.create_client(Trigger, 'retreat_after_release')
        self.open_client = self.create_client(Trigger, 'open_gripper')
        self.close_client = self.create_client(Trigger, 'close_gripper')

        self.get_logger().info('Waiting for required services...')

        while not self.move_to_p0_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('move_to_p0 service not available, waiting...')

        while not self.move_to_p1_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('move_to_p1 service not available, waiting...')

        while not self.start_hand_detection_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('start_hand_detection service not available, waiting...')

        while not self.move_to_hand_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('move_to_hand service not available, waiting...')

        while not self.wait_for_release_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('wait_for_release service not available, waiting...')

        while not self.retreat_after_release_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('retreat_after_release service not available, waiting...')

        while not self.open_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('open_gripper service not available, waiting...')

        while not self.close_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('close_gripper service not available, waiting...')

        self.get_logger().info('All required services are available.')

    def call_trigger_service(self, client, service_name):
        request = Trigger.Request()
        future = client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f'{service_name} response: success={response.success}, message="{response.message}"'
            )
            return response.success
        else:
            self.get_logger().error(f'Failed to call {service_name}')
            return False

    def run_demo(self):
        self.get_logger().info('===== DEMO START =====')

        # Step 1: move to P0
        self.get_logger().info('Step 1: move to P0')
        if not self.call_trigger_service(self.move_to_p0_client, '/move_to_p0'):
            self.get_logger().error('Demo aborted at Step 1.')
            return
        time.sleep(1.0)

        # Step 2: open gripper
        self.get_logger().info('Step 2: open gripper')
        if not self.call_trigger_service(self.open_client, '/open_gripper'):
            self.get_logger().error('Demo aborted at Step 2.')
            return
        time.sleep(1.0)

        # Step 3: move to P1
        self.get_logger().info('Step 3: move to P1')
        if not self.call_trigger_service(self.move_to_p1_client, '/move_to_p1'):
            self.get_logger().error('Demo aborted at Step 3.')
            return
        time.sleep(1.0)

        # Step 4: close gripper
        self.get_logger().info('Step 4: close gripper')
        if not self.call_trigger_service(self.close_client, '/close_gripper'):
            self.get_logger().error('Demo aborted at Step 4.')
            return
        time.sleep(1.0)

        # Step 5: move back to P0
        self.get_logger().info('Step 5: move back to P0')
        if not self.call_trigger_service(self.move_to_p0_client, '/move_to_p0'):
            self.get_logger().error('Demo aborted at Step 5.')
            return
        time.sleep(1.0)

        # Step 6: start hand detection
        self.get_logger().info('Step 6: start hand detection')
        if not self.call_trigger_service(self.start_hand_detection_client, '/start_hand_detection'):
            self.get_logger().error('Demo aborted at Step 6.')
            return
        time.sleep(0.5)

        # Step 7: move to detected hand hover pose
        self.get_logger().info('Step 7: move to hand hover pose')

        success = False
        for i in range(20):
            self.get_logger().info(f'Attempt {i+1}/20 to move to hand')
            if self.call_trigger_service(self.move_to_hand_client, '/move_to_hand'):
                success = True
                break
            time.sleep(1.0)

        if not success:
            self.get_logger().error('Demo aborted at Step 7.')
            return
        time.sleep(1.0)

        # Step 8: wait for release trigger
        self.get_logger().info('Step 8: wait for release trigger')
        if not self.call_trigger_service(self.wait_for_release_client, '/wait_for_release'):
            self.get_logger().error('Demo aborted at Step 8.')
            return
        time.sleep(0.5)

        # Step 9: open gripper to release
        self.get_logger().info('Step 9: open gripper to release')
        if not self.call_trigger_service(self.open_client, '/open_gripper'):
            self.get_logger().error('Demo aborted at Step 9.')
            return
        time.sleep(0.5)

        # Step 10: retreat after release
        self.get_logger().info('Step 10: retreat after release')
        if not self.call_trigger_service(self.retreat_after_release_client, '/retreat_after_release'):
            self.get_logger().error('Demo aborted at Step 10.')
            return
        time.sleep(0.5)

        # Step 11: return to P0
        self.get_logger().info('Step 11: return to P0')
        if not self.call_trigger_service(self.move_to_p0_client, '/move_to_p0'):
            self.get_logger().error('Demo aborted at Step 11.')
            return
        time.sleep(0.5)

        self.get_logger().info('===== DEMO FINISHED SUCCESSFULLY =====')


def main(args=None):
    rclpy.init(args=args)
    node = DemoCoordinator()

    try:
        node.run_demo()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()