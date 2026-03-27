import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from xarm.wrapper import XArmAPI


class XArmControllerNode(Node):
    def __init__(self):
        super().__init__('xarm_controller_node')

        # ===== Parameters =====
        self.declare_parameter('robot_ip', '192.168.1.225')

        self.declare_parameter('speed', 100.0)   # mm/s
        self.declare_parameter('acc', 500.0)     # mm/s^2
        self.declare_parameter('wait_for_finish', True)

        self.declare_parameter('p0', [452.3, 0.0, 75.6, 180.0, 0.0, 0.0])
        self.declare_parameter('p1', [452.3, 0.0, 13.7, 180.0, 0.0, 0.0])

        self.declare_parameter('open_position', 500)
        self.declare_parameter('close_position', 0)
        self.declare_parameter('gripper_speed', 2000)

        self.robot_ip = self.get_parameter('robot_ip').value
        self.speed = float(self.get_parameter('speed').value)
        self.acc = float(self.get_parameter('acc').value)
        self.wait_for_finish = bool(self.get_parameter('wait_for_finish').value)

        self.p0 = list(self.get_parameter('p0').value)
        self.p1 = list(self.get_parameter('p1').value)

        self.open_position = int(self.get_parameter('open_position').value)
        self.close_position = int(self.get_parameter('close_position').value)
        self.gripper_speed = int(self.get_parameter('gripper_speed').value)

        self.get_logger().info(f'Loaded P0: {self.p0}')
        self.get_logger().info(f'Loaded P1: {self.p1}')
        self.get_logger().info(f'Connecting to xArm at {self.robot_ip}...')

        # ===== Single hardware connection =====
        self.arm = XArmAPI(self.robot_ip)
        self.arm.connect()

        self._init_robot()

        # ===== Services =====
        self.move_p0_service = self.create_service(
            Trigger, 'move_to_p0', self.move_to_p0_callback
        )
        self.move_p1_service = self.create_service(
            Trigger, 'move_to_p1', self.move_to_p1_callback
        )
        self.open_service = self.create_service(
            Trigger, 'open_gripper', self.open_gripper_callback
        )
        self.close_service = self.create_service(
            Trigger, 'close_gripper', self.close_gripper_callback
        )

        self.get_logger().info(
            'Services ready: /move_to_p0, /move_to_p1, /open_gripper, /close_gripper'
        )

    def _init_robot(self):
        self.get_logger().info('Initializing robot and gripper...')

        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(0)

        code = self.arm.set_gripper_enable(True)
        self.get_logger().info(f'set_gripper_enable -> code={code}')

        code = self.arm.set_gripper_mode(0)
        self.get_logger().info(f'set_gripper_mode -> code={code}')

        self.arm.set_state(0)

        self.get_logger().info('Robot/gripper initialization done.')

    def _ensure_robot_ready(self):
        """
        Keep readiness logic separate from motion logic.
        This is cleaner than putting mode/state reset directly inside _move_to_pose().
        """
        if self.arm.error_code != 0:
            self.get_logger().warn(f'Clearing error_code={self.arm.error_code}')
            self.arm.clean_error()

        if self.arm.warn_code != 0:
            self.get_logger().warn(f'Clearing warn_code={self.arm.warn_code}')
            self.arm.clean_warn()

        # If robot is not in ready state, restore it
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(0)

        self.get_logger().info(
            f'Ready check: state={self.arm.state}, '
            f'error_code={self.arm.error_code}, warn_code={self.arm.warn_code}'
        )

    def _move_to_pose(self, pose):
        try:
            self._ensure_robot_ready()

            self.get_logger().info(
                f'Moving to pose={pose}, speed={self.speed}, '
                f'acc={self.acc}, wait={self.wait_for_finish}'
            )

            code = self.arm.set_position(
                x=pose[0],
                y=pose[1],
                z=pose[2],
                roll=pose[3],
                pitch=pose[4],
                yaw=pose[5],
                speed=self.speed,
                mvacc=self.acc,
                wait=self.wait_for_finish
            )

            if code != 0:
                return False, f'set_position failed, code={code}'

            return True, f'Moved to pose {pose}'

        except Exception as e:
            return False, f'Exception while moving arm: {e}'

    def _move_gripper(self, target_pos):
        try:
            self._ensure_robot_ready()

            self.get_logger().info(
                f'Moving gripper to position={target_pos}, '
                f'speed={self.gripper_speed}, wait={self.wait_for_finish}'
            )

            code = self.arm.set_gripper_position(
                target_pos,
                wait=self.wait_for_finish,
                speed=self.gripper_speed
            )

            if code != 0:
                return False, f'set_gripper_position failed, code={code}'

            return True, f'Gripper moved to position {target_pos}'

        except Exception as e:
            return False, f'Exception while moving gripper: {e}'

    def move_to_p0_callback(self, request, response):
        self.get_logger().info('MOVE_TO_P0 command received')
        ok, msg = self._move_to_pose(self.p0)
        response.success = ok
        response.message = msg
        if ok:
            self.get_logger().info(msg)
        else:
            self.get_logger().error(msg)
        return response

    def move_to_p1_callback(self, request, response):
        self.get_logger().info('MOVE_TO_P1 command received')
        ok, msg = self._move_to_pose(self.p1)
        response.success = ok
        response.message = msg
        if ok:
            self.get_logger().info(msg)
        else:
            self.get_logger().error(msg)
        return response

    def open_gripper_callback(self, request, response):
        self.get_logger().info('OPEN_GRIPPER command received')
        ok, msg = self._move_gripper(self.open_position)
        response.success = ok
        response.message = msg
        if ok:
            self.get_logger().info(msg)
        else:
            self.get_logger().error(msg)
        return response

    def close_gripper_callback(self, request, response):
        self.get_logger().info('CLOSE_GRIPPER command received')
        ok, msg = self._move_gripper(self.close_position)
        response.success = ok
        response.message = msg
        if ok:
            self.get_logger().info(msg)
        else:
            self.get_logger().error(msg)
        return response

    def destroy_node(self):
        self.get_logger().info('Shutting down xarm_controller_node...')
        try:
            if hasattr(self, 'arm'):
                self.arm.disconnect()
        except Exception as e:
            self.get_logger().warn(f'Exception during disconnect: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = XArmControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, exiting...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()