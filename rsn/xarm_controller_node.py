import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped

from xarm.wrapper import XArmAPI


class XArmControllerNode(Node):
    def __init__(self):
        super().__init__('xarm_controller_node')

        # ===== Parameters =====
        self.declare_parameter('robot_ip', '192.168.1.225')

        self.declare_parameter('speed', 50.0)   # mm/s
        self.declare_parameter('acc', 100.0)    # mm/s^2
        self.declare_parameter('wait_for_finish', True)

        self.declare_parameter('p0', [452.3, 0.0, 75.6, 180.0, 0.0, 0.0])
        self.declare_parameter('p1', [452.3, 0.0, 13.7, 180.0, 0.0, 0.0])

        self.declare_parameter('open_position', 500)
        self.declare_parameter('close_position', 0)
        self.declare_parameter('gripper_speed', 2000)

        # ===== Hand-guided motion parameters =====
        self.declare_parameter('hand_topic', '/right_hand_pose_base')
        self.declare_parameter('hand_hover_offset_mm', 50.0)
        self.declare_parameter('hand_roll', 180.0)
        self.declare_parameter('hand_pitch', 0.0)
        self.declare_parameter('hand_yaw', 90.0)

        # ===== Instrument-guided motion parameters =====
        self.declare_parameter('instrument_topic', '/instrument_grasp_pose_base')
        self.declare_parameter('instrument_hover_offset_mm', 25.0)
        self.declare_parameter('instrument_roll', 180.0)
        self.declare_parameter('instrument_pitch', 0.0)
        self.declare_parameter('instrument_yaw', 0.0)
        self.declare_parameter('instrument_fixed_x_mm', 410.0)
        self.declare_parameter('instrument_fixed_z_mm', 14.0)
        self.declare_parameter('lift_after_grasp_mm', 30.0)

        # ===== Release parameters =====
        self.declare_parameter('release_wait_seconds', 3.0)
        self.declare_parameter('retreat_z_offset_mm', 50.0)

        # ===== FT-based release parameters =====
        self.declare_parameter('release_force_threshold_n', 7.0)
        self.declare_parameter('release_hold_time_s', 0.2)
        self.declare_parameter('release_timeout_s', 10.0)
        self.declare_parameter('release_use_force_magnitude', True)
        self.declare_parameter('release_poll_dt_s', 0.05)

        self.robot_ip = self.get_parameter('robot_ip').value
        self.speed = float(self.get_parameter('speed').value)
        self.acc = float(self.get_parameter('acc').value)
        self.wait_for_finish = bool(self.get_parameter('wait_for_finish').value)

        self.p0 = list(self.get_parameter('p0').value)
        self.p1 = list(self.get_parameter('p1').value)

        self.open_position = int(self.get_parameter('open_position').value)
        self.close_position = int(self.get_parameter('close_position').value)
        self.gripper_speed = int(self.get_parameter('gripper_speed').value)

        self.hand_topic = str(self.get_parameter('hand_topic').value)
        self.hand_hover_offset_mm = float(self.get_parameter('hand_hover_offset_mm').value)
        self.hand_roll = float(self.get_parameter('hand_roll').value)
        self.hand_pitch = float(self.get_parameter('hand_pitch').value)
        self.hand_yaw = float(self.get_parameter('hand_yaw').value)

        self.instrument_topic = str(self.get_parameter('instrument_topic').value)
        self.instrument_hover_offset_mm = float(self.get_parameter('instrument_hover_offset_mm').value)
        self.instrument_roll = float(self.get_parameter('instrument_roll').value)
        self.instrument_pitch = float(self.get_parameter('instrument_pitch').value)
        self.instrument_yaw = float(self.get_parameter('instrument_yaw').value)
        self.instrument_fixed_x_mm = float(self.get_parameter('instrument_fixed_x_mm').value)
        self.instrument_fixed_z_mm = float(self.get_parameter('instrument_fixed_z_mm').value)
        self.lift_after_grasp_mm = float(self.get_parameter('lift_after_grasp_mm').value)

        self.release_wait_seconds = float(self.get_parameter('release_wait_seconds').value)
        self.retreat_z_offset_mm = float(self.get_parameter('retreat_z_offset_mm').value)
        self.release_force_threshold_n = float(self.get_parameter('release_force_threshold_n').value)
        self.release_hold_time_s = float(self.get_parameter('release_hold_time_s').value)
        self.release_timeout_s = float(self.get_parameter('release_timeout_s').value)
        self.release_use_force_magnitude = bool(self.get_parameter('release_use_force_magnitude').value)
        self.release_poll_dt_s = float(self.get_parameter('release_poll_dt_s').value)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # ===== Latest pose cache =====
        self.latest_hand_pose = None
        self.last_hand_hover_pose = None
        self.last_hand_target_pose = None

        self.latest_instrument_pose = None
        self.last_instrument_hover_pose = None
        self.last_instrument_target_pose = None

        self.get_logger().info(f'Loaded P0: {self.p0}')
        self.get_logger().info(f'Loaded P1: {self.p1}')
        self.get_logger().info(f'Hand topic: {self.hand_topic}')
        self.get_logger().info(f'Hand hover offset: {self.hand_hover_offset_mm} mm')
        self.get_logger().info(f'Instrument topic: {self.instrument_topic}')
        self.get_logger().info(f'Instrument hover offset: {self.instrument_hover_offset_mm} mm')
        self.get_logger().info(f'Release wait seconds: {self.release_wait_seconds}')
        self.get_logger().info(f'Retreat z offset: {self.retreat_z_offset_mm} mm')
        self.get_logger().info(f'Release force threshold: {self.release_force_threshold_n} N')
        self.get_logger().info(f'Release hold time: {self.release_hold_time_s} s')
        self.get_logger().info(f'Release timeout: {self.release_timeout_s} s')
        self.get_logger().info(f'Release use force magnitude: {self.release_use_force_magnitude}')
        self.get_logger().info(f'Release poll dt: {self.release_poll_dt_s} s')
        self.get_logger().info(f'Connecting to xArm at {self.robot_ip}...')

        # ===== Single hardware connection =====
        self.arm = XArmAPI(self.robot_ip)
        self.arm.connect()

        self._init_robot()

        # ===== Subscribers =====
        self.hand_pose_sub = self.create_subscription(
            PoseStamped,
            self.hand_topic,
            self.hand_pose_callback,
            10
        )

        self.instrument_pose_sub = self.create_subscription(
            PoseStamped,
            self.instrument_topic,
            self.instrument_pose_callback,
            10
        )

        # ===== Services =====
        self.move_p0_service = self.create_service(
            Trigger, 'move_to_p0', self.move_to_p0_callback
        )
        self.move_p1_service = self.create_service(
            Trigger, 'move_to_p1', self.move_to_p1_callback
        )
        self.move_to_hand_service = self.create_service(
            Trigger, 'move_to_hand', self.move_to_hand_callback
        )
        self.move_to_instrument_service = self.create_service(
            Trigger, 'move_to_instrument', self.move_to_instrument_callback
        )
        self.lift_after_grasp_service = self.create_service(
            Trigger, 'lift_after_grasp', self.lift_after_grasp_callback
        )
        self.return_instrument_to_source_service = self.create_service(
            Trigger,
            'return_instrument_to_source',
            self.return_instrument_to_source_callback
        )
        self.wait_for_release_service = self.create_service(
            Trigger, 'wait_for_release', self.wait_for_release_callback
        )
        self.retreat_after_release_service = self.create_service(
            Trigger, 'retreat_after_release', self.retreat_after_release_callback
        )
        self.open_service = self.create_service(
            Trigger, 'open_gripper', self.open_gripper_callback
        )
        self.close_service = self.create_service(
            Trigger, 'close_gripper', self.close_gripper_callback
        )

        self.get_logger().info(
            'Services ready: /move_to_p0, /move_to_p1, /move_to_hand, '
            '/move_to_instrument, /lift_after_grasp, '
            '/return_instrument_to_source, /open_gripper, /close_gripper, '
            '/wait_for_release, /retreat_after_release'
        )

    def _on_set_parameters(self, parameters):
        updates = {}

        for parameter in parameters:
            if parameter.name == 'speed':
                value = float(parameter.value)
                if value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='speed must be greater than 0.0'
                    )
                updates['speed'] = value

            elif parameter.name == 'acc':
                value = float(parameter.value)
                if value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='acc must be greater than 0.0'
                    )
                updates['acc'] = value

            elif parameter.name == 'wait_for_finish':
                updates['wait_for_finish'] = bool(parameter.value)

        if 'speed' in updates:
            self.speed = updates['speed']
            self.get_logger().info(f'Updated speed to {self.speed} mm/s')

        if 'acc' in updates:
            self.acc = updates['acc']
            self.get_logger().info(f'Updated acc to {self.acc} mm/s^2')

        if 'wait_for_finish' in updates:
            self.wait_for_finish = updates['wait_for_finish']
            self.get_logger().info(
                f'Updated wait_for_finish to {self.wait_for_finish}'
            )

        return SetParametersResult(successful=True)

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
        if self.arm.error_code != 0:
            self.get_logger().warn(f'Clearing error_code={self.arm.error_code}')
            self.arm.clean_error()

        if self.arm.warn_code != 0:
            self.get_logger().warn(f'Clearing warn_code={self.arm.warn_code}')
            self.arm.clean_warn()

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

    def hand_pose_callback(self, msg):
        self.latest_hand_pose = msg.pose
        self.get_logger().info(
            f'Received hand pose in {msg.header.frame_id}: '
            f'x={msg.pose.position.x:.4f} m, '
            f'y={msg.pose.position.y:.4f} m, '
            f'z={msg.pose.position.z:.4f} m'
        )

    def instrument_pose_callback(self, msg):
        self.latest_instrument_pose = msg.pose
        self.get_logger().info(
            f'Received instrument pose in {msg.header.frame_id}: '
            f'x={msg.pose.position.x:.4f} m, '
            f'y={msg.pose.position.y:.4f} m, '
            f'z={msg.pose.position.z:.4f} m'
        )

    def _build_hand_approach_poses(self):
        if self.latest_hand_pose is None:
            return None, None, 'No hand pose received yet.'

        x_mm = self.latest_hand_pose.position.x * 1000.0
        y_mm = self.latest_hand_pose.position.y * 1000.0
        z_mm = self.latest_hand_pose.position.z * 1000.0

        self.get_logger().info(
            f'[hand] raw pose from topic: x={self.latest_hand_pose.position.x:.4f} m, '
            f'y={self.latest_hand_pose.position.y:.4f} m, '
            f'z={self.latest_hand_pose.position.z:.4f} m'
        )

        hover_pose = [
            x_mm,
            y_mm,
            z_mm + self.hand_hover_offset_mm,
            self.hand_roll,
            self.hand_pitch,
            self.hand_yaw,
        ]

        target_pose = [
            x_mm,
            y_mm,
            z_mm,
            self.hand_roll,
            self.hand_pitch,
            self.hand_yaw,
        ]

        return hover_pose, target_pose, (
            f'Built approach poses from latest hand pose: hover={hover_pose}, target={target_pose}'
        )

    def _build_instrument_approach_poses(self):
        if self.latest_instrument_pose is None:
            return None, None, 'No instrument pose received yet.'

        x_mm = self.instrument_fixed_x_mm
        y_mm = self.latest_instrument_pose.position.y * 1000.0
        z_mm = self.instrument_fixed_z_mm

        self.get_logger().info(
            f'[instrument] raw pose from topic: x={self.latest_instrument_pose.position.x:.4f} m, '
            f'y={self.latest_instrument_pose.position.y:.4f} m, '
            f'z={self.latest_instrument_pose.position.z:.4f} m'
        )

        self.get_logger().info(
            f'[instrument] using fixed x_mm={x_mm:.1f}, '
            f'fixed z_mm={z_mm:.1f}, '
            f'while y_mm={y_mm:.1f}'
        )

        hover_pose = [
            x_mm,
            y_mm,
            z_mm + self.instrument_hover_offset_mm,
            self.instrument_roll,
            self.instrument_pitch,
            self.instrument_yaw,
        ]

        target_pose = [
            x_mm,
            y_mm,
            z_mm,
            self.instrument_roll,
            self.instrument_pitch,
            self.instrument_yaw,
        ]

        return hover_pose, target_pose, (
            f'Built approach poses from latest instrument pose: hover={hover_pose}, target={target_pose}'
        )

    def _build_lift_after_grasp_pose(self):
        if self.last_instrument_target_pose is None:
            return None, 'No last instrument target pose recorded yet.'
    
        pose = list(self.last_instrument_target_pose)
        pose[2] += self.lift_after_grasp_mm
    
        return pose, (f'Built lift after grasp pose from last instrument target pose: {pose}')
    
    def _build_return_instrument_poses(self):
        if self.last_instrument_hover_pose is None:
            return None, None, 'No last instrument hover pose recorded yet.'

        if self.last_instrument_target_pose is None:
            return None, None, 'No last instrument target pose recorded yet.'

        hover_pose = list(self.last_instrument_hover_pose)
        target_pose = list(self.last_instrument_target_pose)
        return hover_pose, target_pose, (
            'Built return-to-source poses from last instrument poses: '
            f'hover={hover_pose}, target={target_pose}'
        )

    def _build_retreat_pose(self):
        if self.last_hand_hover_pose is None:
            return None, 'No last hand hover pose recorded yet.'

        pose = list(self.last_hand_hover_pose)
        pose[2] += self.retreat_z_offset_mm
        return pose, f'Built retreat pose from last hand hover pose: {pose}'

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

    def move_to_hand_callback(self, request, response):
        self.get_logger().info('MOVE_TO_HAND command received')

        hover_pose, target_pose, build_msg = self._build_hand_approach_poses()
        self.get_logger().info(build_msg)

        if hover_pose is None or target_pose is None:
            response.success = False
            response.message = build_msg
            self.get_logger().error(build_msg)
            return response

        self.get_logger().info('Stage 1/2: move to hand upper hover pose')
        ok_hover, msg_hover = self._move_to_pose(hover_pose)
        if not ok_hover:
            response.success = False
            response.message = f'Failed at hover stage: {msg_hover}'
            self.get_logger().error(response.message)
            return response

        self.last_hand_hover_pose = list(hover_pose)
        self.get_logger().info(msg_hover)

        self.get_logger().info('Stage 2/2: descend to hand target pose')
        ok_target, msg_target = self._move_to_pose(target_pose)
        response.success = ok_target
        response.message = msg_target

        if ok_target:
            self.last_hand_target_pose = list(target_pose)
            self.get_logger().info(msg_target)
        else:
            self.get_logger().error(msg_target)

        return response

    def move_to_instrument_callback(self, request, response):
        self.get_logger().info('MOVE_TO_INSTRUMENT command received')

        hover_pose, target_pose, build_msg = self._build_instrument_approach_poses()
        self.get_logger().info(build_msg)

        if hover_pose is None or target_pose is None:
            response.success = False
            response.message = build_msg
            self.get_logger().error(build_msg)
            return response

        self.get_logger().info('Stage 1/2: move to instrument upper hover pose')
        ok_hover, msg_hover = self._move_to_pose(hover_pose)
        if not ok_hover:
            response.success = False
            response.message = f'Failed at hover stage: {msg_hover}'
            self.get_logger().error(response.message)
            return response

        self.last_instrument_hover_pose = list(hover_pose)
        self.get_logger().info(msg_hover)

        self.get_logger().info('Stage 2/2: descend to instrument target pose')
        ok_target, msg_target = self._move_to_pose(target_pose)
        response.success = ok_target
        response.message = msg_target

        if ok_target:
            self.last_instrument_target_pose = list(target_pose)
            self.get_logger().info(msg_target)
        else:
            self.get_logger().error(msg_target)

        return response
    
    def lift_after_grasp_callback(self, request, response):
        self.get_logger().info('LIFT_AFTER_GRASP command received')

        pose, build_msg = self._build_lift_after_grasp_pose()
        self.get_logger().info(build_msg)

        if pose is None:
            response.success = False
            response.message = build_msg
            self.get_logger().error(build_msg)
            return response

        ok, msg = self._move_to_pose(pose)
        response.success = ok
        response.message = msg

        if ok:
            self.get_logger().info(msg)
        else:
            self.get_logger().error(msg)

        return response

    def return_instrument_to_source_callback(self, request, response):
        self.get_logger().info('RETURN_INSTRUMENT_TO_SOURCE command received')

        hover_pose, target_pose, build_msg = self._build_return_instrument_poses()
        self.get_logger().info(build_msg)

        if hover_pose is None or target_pose is None:
            response.success = False
            response.message = build_msg
            self.get_logger().error(build_msg)
            return response

        steps = []

        if self.last_hand_hover_pose is not None:
            hand_hover_pose = list(self.last_hand_hover_pose)
            steps.append(
                ('move to hand hover pose', lambda: self._move_to_pose(hand_hover_pose))
            )

        steps.extend([
            ('move to instrument hover pose', lambda: self._move_to_pose(hover_pose)),
            ('descend to instrument source pose', lambda: self._move_to_pose(target_pose)),
            ('open gripper at source pose', lambda: self._move_gripper(self.open_position)),
            ('retreat to instrument hover pose', lambda: self._move_to_pose(hover_pose)),
            ('return to P0 after placing instrument', lambda: self._move_to_pose(self.p0)),
        ])

        for label, step in steps:
            self.get_logger().info(f'Return instrument step: {label}')
            ok, msg = step()
            if not ok:
                response.success = False
                response.message = f'Failed to {label}: {msg}'
                self.get_logger().error(response.message)
                return response
            self.get_logger().info(msg)

        response.success = True
        response.message = 'Returned instrument to source pose and moved to P0.'
        self.get_logger().info(response.message)
        return response

    def wait_for_release_callback(self, request, response):
        self.get_logger().info('WAIT_FOR_RELEASE command received')

        try:
            code = self.arm.set_ft_sensor_enable(1)
            self.get_logger().info(f'set_ft_sensor_enable(1) -> code={code}')
            time.sleep(0.2)

            code = self.arm.set_ft_sensor_zero()
            self.get_logger().info(f'set_ft_sensor_zero() -> code={code}')
            time.sleep(0.2)

            start_time = time.time()
            trigger_start_time = None

            while time.time() - start_time < self.release_timeout_s:
                code, ft = self.arm.get_ft_sensor_data()

                if code != 0 or ft is None or len(ft) < 6:
                    self.get_logger().warn(f'Failed to read FT data, code={code}, ft={ft}')
                    time.sleep(self.release_poll_dt_s)
                    continue

                fx = float(ft[0])
                fy = float(ft[1])
                fz = float(ft[2])
                tx = float(ft[3])
                ty = float(ft[4])
                tz = float(ft[5])

                if self.release_use_force_magnitude:
                    release_signal = (fx**2 + fy**2 + fz**2) ** 0.5
                    signal_name = '|F|'
                else:
                    release_signal = (fx**2 + fy**2 + fz**2) ** 0.5
                    signal_name = '|F|'

                self.get_logger().info(
                    f'FT: fx={fx:.2f}, fy={fy:.2f}, fz={fz:.2f}, '
                    f'tx={tx:.2f}, ty={ty:.2f}, tz={tz:.2f}, '
                    f'{signal_name}={release_signal:.2f}'
                )

                if release_signal >= self.release_force_threshold_n:
                    if trigger_start_time is None:
                        trigger_start_time = time.time()
                        self.get_logger().info(
                            f'Release threshold reached: {signal_name}={release_signal:.2f} >= '
                            f'{self.release_force_threshold_n:.2f}. Starting hold timer...'
                        )
                    else:
                        held_time = time.time() - trigger_start_time
                        if held_time >= self.release_hold_time_s:
                            response.success = True
                            response.message = (
                                f'Release triggered: {signal_name}={release_signal:.2f} '
                                f'>= {self.release_force_threshold_n:.2f} '
                                f'for {held_time:.2f} s'
                            )
                            self.get_logger().info(response.message)
                            return response
                else:
                    if trigger_start_time is not None:
                        self.get_logger().info(
                            'Release signal dropped below threshold. Resetting hold timer.'
                        )
                    trigger_start_time = None

                time.sleep(self.release_poll_dt_s)

            response.success = False
            response.message = (
                f'Release timeout after {self.release_timeout_s:.2f} s'
            )
            self.get_logger().warn(response.message)
            return response

        except Exception as e:
            response.success = False
            response.message = f'Exception while waiting for release: {e}'
            self.get_logger().error(response.message)
            return response

    def retreat_after_release_callback(self, request, response):
        self.get_logger().info('RETREAT_AFTER_RELEASE command received')

        pose, build_msg = self._build_retreat_pose()
        self.get_logger().info(build_msg)

        if pose is None:
            response.success = False
            response.message = build_msg
            self.get_logger().error(build_msg)
            return response

        ok, msg = self._move_to_pose(pose)
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


def main(args=None):
    rclpy.init(args=args)
    node = XArmControllerNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
