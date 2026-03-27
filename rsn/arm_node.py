import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from xarm.wrapper import XArmAPI

class ArmNode(Node):
    def __init__(self):
        super().__init__('arm_node')

        # ===== Parameters =====
        self.declare_parameter('robot_ip', '192.168.1.225')
        self.declare_parameter('speed', 100)    # mm/s
        self.declare_parameter('acc', 500)    # mm/s^2
        self.declare_parameter('wait_for_finish', True)

        # Predefined poses
        self.declare_parameter('p0', [452.3, 0.0, 75.6, 180.0, 0.0, 0.0])  # Idle pose, 6D pose list: [x, y, z, roll, pitch, yaw] mm and degree
        self.declare_parameter('p1', [452.3, 0.0, 13.7, 180.0, 0.0, 0.0])   # Grasp pose

        self.robot_ip = self.get_parameter('robot_ip').value
        self.speed = float(self.get_parameter('speed').value)
        self.acc = float(self.get_parameter('acc').value)
        self.wait_for_finish = bool(self.get_parameter('wait_for_finish').value)
        self.p0 = list(self.get_parameter('p0').value)
        self.p1 = list(self.get_parameter('p1').value)

        self.get_logger().info(f'Loaded P0: {self.p0}')
        self.get_logger().info(f'Loaded P1: {self.p1}')

        self.get_logger().info(f'Connecting to xArm at {self.robot_ip}...')
        
        # =====Connect to xArm =====
        self.arm = XArmAPI(self.robot_ip)
        self.arm.connect()

        self._init_robot()

        self.move_p0_service = self.create_service(Trigger, 'move_to_p0', self.move_to_p0_callback)
        self.move_p1_service = self.create_service(Trigger, 'move_to_p1', self.move_to_p1_callback)

        self.get_logger().info('Arm node is ready.')
        self.get_logger().info('Services: move_to_p0, move_to_p1')

    def _init_robot(self):
        self.get_logger().info('Initializing robot...')

        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)  # Position control mode
        self.arm.set_state(0)  # Enable the robot

        self.get_logger().info('Robot initialization done.')
    
    def _move_to_pose(self, pose):
        try:
            self.get_logger().info(
            f'Before move: state={self.arm.state}, error_code={self.arm.error_code}, warn_code={self.arm.warn_code}'
        )
            
            self.arm.motion_enable(enable=True)
            self.arm.set_mode(0)
            self.arm.set_state(0)

            self.get_logger().info(f'Moving to pose: {pose}, speed={self.speed}, acc={self.acc}, wait_for_finish={self.wait_for_finish}')
            
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
        
    def move_to_p0_callback(self, request, response):
        self.get_logger().info('MOVE_TO_P0 command received.')
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
    
    def destroy_node(self):
        self.get_logger().info('Shutting down arm node...')
        try:
            if hasattr(self, 'arm'):
                self.arm.disconnect()
        except Exception as e:
                self.get_logger().warn(f'Error during disconnect: {e}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArmNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, exiting...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()