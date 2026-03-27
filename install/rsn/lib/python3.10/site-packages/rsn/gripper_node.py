
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from xarm.wrapper import XArmAPI

class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')

        # ===== Parameters =====
        self.declare_parameter('robot_ip', '192.168.1.225')
        self.declare_parameter('open_position', 500)
        self.declare_parameter('close_position', 0)
        self.declare_parameter('gripper_speed', 2000)
        self.declare_parameter('wait_for_finish', True)

        self.robot_ip = self.get_parameter('robot_ip').value
        self.open_position = self.get_parameter('open_position').value
        self.close_position = self.get_parameter('close_position').value
        self.gripper_speed = self.get_parameter('gripper_speed').value
        self.wait_for_finish = self.get_parameter('wait_for_finish').value

        self.get_logger().info(f'Connecting to xArm at {self.robot_ip}...')
        
        # =====Connect to xArm =====
        self.arm = XArmAPI(self.robot_ip)
        self.arm.connect()
        
        self._init_robot()

        #==== Services =====
        self.open_service = self.create_service(Trigger, 'open_gripper', self.open_gripper_callback)
        self.close_service = self.create_service(Trigger, 'close_gripper', self.close_gripper_callback)

        self.get_logger().info('Gripper node is ready.')
        self.get_logger().info('Services: open_gripper, close_gripper')
        
    def _init_robot(self):
        self.get_logger().info('Initializing robot and gripper...')
        
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)  # Position control mode
        self.arm.set_state(0)  # Enable the robot
        code = self.arm.set_gripper_enable(True)
        self.get_logger().info(f'set_gripper_enable -> code={code}')

        code = self.arm.set_gripper_mode(0)
        self.get_logger().info(f'set_gripper_mode -> code={code}')

        self.arm.set_state(0)

        self.get_logger().info('Robot/gripper initialization done.')

    def _move_gripper(self, target_pos: int):
        """
        Move xArm gripper to target position. Returns: (ok: bool, msg: str)
        """
        try:
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

    def open_gripper_callback(self, request, response):
        self.get_logger().info('Gripper OPEN command received')

        ok, msg = self._move_gripper(self.open_position)
        response.success = ok
        response.message = msg

        if ok:
            self.get_logger().info(msg)
        else:
            self.get_logger().error(msg)

        return response

    def close_gripper_callback(self, request, response):
        self.get_logger().info('Gripper CLOSE command received')

        ok, msg = self._move_gripper(self.close_position)
        response.success = ok
        response.message = msg

        if ok:
            self.get_logger().info(msg)
        else:
            self.get_logger().error(msg)

        return response
    
    def destroy_node(self):
        self.get_logger().info('Shutting down gripper node...')
        try:
            if hasattr(self, 'arm'):
                self.arm.disconnect()
        except Exception as e:
            self.get_logger().warn(f'Exception during disconnect: {e}')
        super().destroy_node()
    
def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, exiting...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()