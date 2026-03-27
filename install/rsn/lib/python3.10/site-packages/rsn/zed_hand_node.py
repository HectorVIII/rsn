import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import pyzed.sl as sl


class ZedHandNode(Node):
    def __init__(self):
        super().__init__('zed_hand_node')

        # ===== Parameters =====
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('stability_duration', 2.0)      # seconds
        self.declare_parameter('stability_threshold', 0.03)    # meters
        self.declare_parameter('publish_topic', '/right_hand_pose_base')
        self.declare_parameter('exit_delay_after_publish', 1.0)  # seconds
        self.declare_parameter('show_viewer', True)

        self.camera_fps = int(self.get_parameter('camera_fps').value)
        self.stability_duration = float(self.get_parameter('stability_duration').value)
        self.stability_threshold = float(self.get_parameter('stability_threshold').value)
        self.publish_topic = str(self.get_parameter('publish_topic').value)
        self.exit_delay_after_publish = float(self.get_parameter('exit_delay_after_publish').value)
        self.show_viewer = bool(self.get_parameter('show_viewer').value)

        # ===== Fixed transform: camera -> robot base =====
        self.T_cam2base = np.array([
            [-0.152816883,  0.733177385, -0.662644642,  0.694418397],
            [ 0.988175165,  0.104867217, -0.111860223,  0.431523428],
            [-0.012523686, -0.671903110, -0.740533165,  0.596290570],
            [ 0.0,          0.0,          0.0,          1.0        ],
        ], dtype=np.float64)

        # ===== Publisher =====
        self.pose_pub = self.create_publisher(PoseStamped, self.publish_topic, 10)

        # ===== ZED objects =====
        self.zed = sl.Camera()
        self.runtime_params = sl.RuntimeParameters()
        self.bodies = sl.Bodies()
        self.image = sl.Mat()

        # ===== Stability state =====
        self.stable_start_time = None
        self.stable_ref_cam = None

        # ===== One-shot state =====
        self.published_once = False

        # ===== Init camera =====
        self._init_zed()

        # ===== Timer =====
        timer_period = 1.0 / float(self.camera_fps)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('zed_hand_node started.')
        self.get_logger().info(f'Publish topic: {self.publish_topic}')
        self.get_logger().info(f'Stability duration: {self.stability_duration:.2f} s')
        self.get_logger().info(f'Stability threshold: {self.stability_threshold:.3f} m')
        self.get_logger().info(f'Show viewer: {self.show_viewer}')
        self.get_logger().info('Mode: publish once after stable right hand, then exit.')

    def _init_zed(self):
        self.get_logger().info('Opening ZED camera...')

        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = self.camera_fps
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE

        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Failed to open ZED camera: {repr(status)}')

        self.get_logger().info('ZED camera opened successfully.')

        positional_tracking_params = sl.PositionalTrackingParameters()
        tracking_status = self.zed.enable_positional_tracking(positional_tracking_params)
        if tracking_status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Failed to enable positional tracking: {repr(tracking_status)}')

        self.get_logger().info('Positional tracking enabled.')

        body_params = sl.BodyTrackingParameters()
        body_params.enable_tracking = True
        body_params.enable_body_fitting = True
        body_params.body_format = sl.BODY_FORMAT.BODY_34
        body_params.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_ACCURATE

        body_status = self.zed.enable_body_tracking(body_params)
        if body_status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Failed to enable body tracking: {repr(body_status)}')

        self.get_logger().info('Body tracking enabled with BODY_34.')

    def timer_callback(self):
        if self.published_once:
            return

        if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
            return

        # Get bodies
        self.zed.retrieve_bodies(self.bodies)
        self.get_logger().info(f'bodies detected: {len(self.bodies.body_list)}')
        for i, body in enumerate(self.bodies.body_list):
            self.get_logger().info(f'body {i}, num keypoints: {len(body.keypoint)}')
            if len(body.keypoint) > 14:
                self.get_logger().info(f'kp14 = {body.keypoint[14]}')
            if len(body.keypoint) > 15:
                self.get_logger().info(f'kp15 = {body.keypoint[15]}')

        # Get left image for viewer
        frame = None
        if self.show_viewer:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            frame = self.image.get_data().copy()

            # ZED image may be BGRA, convert to BGR for OpenCV drawing
            if frame is not None and len(frame.shape) == 3 and frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        hand_cam, hand_uv = self._get_right_hand_position_and_uv()

        status_text = 'NO HAND'
        stable_elapsed = 0.0

        if hand_cam is None:
            if self.stable_start_time is not None:
                self.get_logger().info('Right hand lost. Resetting stability state.')
            self._reset_stability()
        else:
            now = time.time()

            if self.stable_start_time is None:
                self.stable_start_time = now
                self.stable_ref_cam = hand_cam.copy()
                self.get_logger().info(
                    f'Right hand detected. Start timing. '
                    f'cam = [{hand_cam[0]:.3f}, {hand_cam[1]:.3f}, {hand_cam[2]:.3f}] m'
                )
                status_text = 'TRACKING'
            else:
                dist = np.linalg.norm(hand_cam - self.stable_ref_cam)

                if dist > self.stability_threshold:
                    self.get_logger().info(
                        f'Hand moved too much ({dist:.3f} m > {self.stability_threshold:.3f} m). Restarting timer.'
                    )
                    self.stable_start_time = now
                    self.stable_ref_cam = hand_cam.copy()
                    stable_elapsed = 0.0
                    status_text = 'TRACKING'
                else:
                    stable_elapsed = now - self.stable_start_time
                    status_text = 'TRACKING'

                    if stable_elapsed >= self.stability_duration:
                        hand_base = self._transform_cam_to_base(hand_cam)
                        self._publish_pose(hand_base)

                        self.published_once = True
                        status_text = 'STABLE'

                        self.get_logger().info(
                            'Stable right hand confirmed and published once. '
                            f'cam = [{hand_cam[0]:.3f}, {hand_cam[1]:.3f}, {hand_cam[2]:.3f}] m, '
                            f'base = [{hand_base[0]:.3f}, {hand_base[1]:.3f}, {hand_base[2]:.3f}] m'
                        )

                        if frame is not None:
                            self._draw_overlay(
                                frame, hand_uv, hand_cam, status_text, stable_elapsed
                            )
                            cv2.imshow('ZED Right Hand Viewer', frame)
                            cv2.waitKey(1)

                        time.sleep(self.exit_delay_after_publish)
                        self.get_logger().info('Publish completed. Exiting node...')
                        self.destroy_node()
                        rclpy.shutdown()
                        return

        if frame is not None:
            self._draw_overlay(frame, hand_uv, hand_cam, status_text, stable_elapsed)
            cv2.imshow('ZED Right Hand Viewer', frame)

            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                self.get_logger().info('Viewer closed by user. Exiting node...')
                self.destroy_node()
                rclpy.shutdown()

    def _get_right_hand_position_and_uv(self):
        """
        Use BODY_34 keypoint index 15 as requested.
        Return:
            best_point_3d: np.array([x, y, z]) in camera frame, meters
            best_point_2d: np.array([u, v]) in image pixel coordinates
        """
        if self.bodies is None or len(self.bodies.body_list) == 0:
            return None, None

        best_point_3d = None
        best_point_2d = None
        best_dist = float('inf')

        for body in self.bodies.body_list:
            if body.keypoint is None or body.keypoint_2d is None:
                continue

            if len(body.keypoint) <= 15 or len(body.keypoint_2d) <= 15:
                continue

            kp3d = body.keypoint[15]
            kp2d = body.keypoint_2d[15]

            if kp3d is None or kp2d is None:
                continue

            x, y, z = float(kp3d[0]), float(kp3d[1]), float(kp3d[2])
            u, v = float(kp2d[0]), float(kp2d[1])

            if not np.isfinite(x) or not np.isfinite(y) or not np.isfinite(z):
                continue
            if not np.isfinite(u) or not np.isfinite(v):
                continue
            # Reject only obviously invalid 3D points
            if np.linalg.norm([x, y, z]) < 1e-6:
                continue

            dist = np.linalg.norm([x, y, z])
            if dist < best_dist:
                best_dist = dist
                best_point_3d = np.array([x, y, z], dtype=np.float64)
                best_point_2d = np.array([u, v], dtype=np.float64)

        return best_point_3d, best_point_2d

    def _transform_cam_to_base(self, p_cam):
        p_cam_h = np.array([p_cam[0], p_cam[1], p_cam[2], 1.0], dtype=np.float64)
        p_base_h = self.T_cam2base @ p_cam_h
        return p_base_h[:3]

    def _publish_pose(self, p_base):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'xarm_base'

        msg.pose.position.x = float(p_base[0])
        msg.pose.position.y = float(p_base[1])
        msg.pose.position.z = float(p_base[2])

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.pose_pub.publish(msg)

    def _draw_overlay(self, frame, hand_uv, hand_cam, status_text, stable_elapsed):
        h, w = frame.shape[:2]

        # Draw keypoint
        if hand_uv is not None:
            u, v = int(hand_uv[0]), int(hand_uv[1])
            if 0 <= u < w and 0 <= v < h:
                cv2.circle(frame, (u, v), 8, (0, 0, 255), -1)
                cv2.circle(frame, (u, v), 16, (0, 255, 255), 2)
                cv2.putText(
                    frame, 'KP15',
                    (u + 10, v - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
                )

        # Draw state text
        cv2.putText(
            frame,
            f'State: {status_text}',
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 0) if status_text == 'STABLE' else (0, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f'Stable: {stable_elapsed:.2f} / {self.stability_duration:.2f} s',
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 0),
            2
        )

        if hand_cam is not None:
            cv2.putText(
                frame,
                f'Cam XYZ: [{hand_cam[0]:.3f}, {hand_cam[1]:.3f}, {hand_cam[2]:.3f}] m',
                (20, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

        cv2.putText(
            frame,
            'Press q or ESC to quit',
            (20, h - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (200, 200, 200),
            2
        )

    def _reset_stability(self):
        self.stable_start_time = None
        self.stable_ref_cam = None

    def destroy_node(self):
        self.get_logger().info('Shutting down zed_hand_node...')
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        try:
            if hasattr(self, 'zed'):
                self.zed.disable_body_tracking()
                self.zed.disable_positional_tracking()
                self.zed.close()
        except Exception as e:
            self.get_logger().warn(f'Exception during ZED shutdown: {e}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ZedHandNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'[zed_hand_node] Fatal error: {e}')
    finally:
        if rclpy.ok():
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()