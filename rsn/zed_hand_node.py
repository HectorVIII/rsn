import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

import pyzed.sl as sl

try:
    import mediapipe as mp
except ImportError:
    mp = None


class ZedHandNode(Node):
    def __init__(self):
        super().__init__('zed_hand_node')

        if mp is None:
            raise RuntimeError(
                'mediapipe is not installed. Install it before running zed_hand_node.'
            )

        # ===== Parameters =====
        self.declare_parameter('camera_fps', 30)

        # Stability logic: EMA + stable frame count
        self.declare_parameter('ema_alpha', 0.4)                    # EMA alpha for smoothing the detected hand position over time (0.0 = no smoothing, 1.0 = max smoothing)
        self.declare_parameter('position_tolerance', 0.01)          # meters, max allowed EMA diff to count as stable
        self.declare_parameter('stable_frames_required', 45)        # 45 frames at 30 fps ~= 1.5 s

        # MediaPipe hand detection parameters
        self.declare_parameter('max_num_hands', 2)
        self.declare_parameter('min_detection_confidence', 0.6)     # Minimum confidence for MediaPipe to consider a hand detected in the image
        self.declare_parameter('min_tracking_confidence', 0.6)      # Minimum confidence for MediaPipe to consider the hand tracking valid in subsequent frames
        self.declare_parameter('target_hand_label', 'Left')         # For current ZED view, Left selects real right hand

        # 3D point extraction parameters
        self.declare_parameter('depth_window_radius', 2)            # Pixel radius around the target fingertip to consider for depth median calculation (e.g. 2 means a 5x5 window)
        self.declare_parameter('depth_min_valid_points', 5)         # Minimum number of valid depth points required in the window to consider the 3D point valid    

        self.declare_parameter('publish_topic', '/right_hand_pose_base')
        self.declare_parameter('exit_delay_after_publish', 3.0)     # Seconds to wait after publishing before exiting, to ensure message is sent before shutdown
        self.declare_parameter('show_viewer', True)

        self.camera_fps = int(self.get_parameter('camera_fps').value)
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.stable_frames_required = int(self.get_parameter('stable_frames_required').value)

        self.max_num_hands = int(self.get_parameter('max_num_hands').value)
        self.min_detection_confidence = float(self.get_parameter('min_detection_confidence').value)
        self.min_tracking_confidence = float(self.get_parameter('min_tracking_confidence').value)
        self.target_hand_label = str(self.get_parameter('target_hand_label').value)

        self.depth_window_radius = int(self.get_parameter('depth_window_radius').value)
        self.depth_min_valid_points = int(self.get_parameter('depth_min_valid_points').value)

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

        # ===== Service =====
        self.start_detection_service = self.create_service(
            Trigger,
            'start_hand_detection',
            self.start_hand_detection_callback
        )

        # ===== ZED objects =====
        self.zed = sl.Camera()
        self.runtime_params = sl.RuntimeParameters()
        self.image = sl.Mat()
        self.point_cloud = sl.Mat()

        # ===== MediaPipe =====
        self.mp_hands = mp.solutions.hands                      # MediaPipe Hands solution for hand landmark detection and tracking
        self.mp_drawing = mp.solutions.drawing_utils            # Utility for drawing hand landmarks and connections on images
        self.mp_drawing_styles = mp.solutions.drawing_styles    # Predefined styles for drawing hand landmarks and connections
        
        # Create MediaPipe Hands object with specified parameters
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,                                    # Set to False for video stream input, enabling tracking across frames
            max_num_hands=self.max_num_hands,
            min_detection_confidence=self.min_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence,
            model_complexity=1,                                     # 0, 1, or 2. Higher complexity may improve accuracy but reduce speed. Default is 1.
        )

        # ===== EMA + stable frame state =====
        self.ema = None
        self.last_ema = None
        self.stable_frames = 0

        # ===== One-shot state =====
        self.published_once = False
        self.detection_enabled = False

        # ===== Debug / viewer state =====
        self.last_hand_uv = None
        self.last_hand_score = 0.0
        self.last_point_count = 0

        # ===== Init camera =====
        self._init_zed()

        # ===== Timer =====
        timer_period = 1.0 / float(self.camera_fps)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('zed_hand_node started.')
        self.get_logger().info(f'Publish topic: {self.publish_topic}')
        self.get_logger().info(f'EMA alpha: {self.ema_alpha:.3f}')
        self.get_logger().info(f'Position tolerance: {self.position_tolerance:.4f} m')
        self.get_logger().info(f'Stable frames required: {self.stable_frames_required}')
        self.get_logger().info(f'Target hand label: {self.target_hand_label}')
        self.get_logger().info('Target point: index fingertip (landmark 8)')
        self.get_logger().info(f'Depth window radius: {self.depth_window_radius}')
        self.get_logger().info(f'Show viewer: {self.show_viewer}')
        self.get_logger().info('Mode: wait for /start_hand_detection, then publish once and exit.')

    def _init_zed(self):
        self.get_logger().info('Opening ZED camera...')

        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = self.camera_fps
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL

        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Failed to open ZED camera: {repr(status)}')

        self.get_logger().info('ZED camera opened successfully.')

    def start_hand_detection_callback(self, request, response):
        """Enable hand detection when the start service is called."""
        self.get_logger().info('START_HAND_DETECTION command received')
        self.detection_enabled = True
        self._reset_stability()
        response.success = True
        response.message = 'Hand detection enabled.'
        return response

    def timer_callback(self):
        """Process each frame, detect the hand, and publish once when stable."""

        if self.published_once:
            return

        if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
            return

        self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
        frame = self.image.get_data().copy()
        if frame is not None and len(frame.shape) == 3 and frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        # Idle mode: wait for explicit start signal
        if not self.detection_enabled:
            if self.show_viewer and frame is not None:
                self._draw_idle_overlay(frame)
                cv2.imshow('ZED Right Hand Viewer', frame)
                key = cv2.waitKey(1) & 0xFF
                if key == 27 or key == ord('q'):
                    self.get_logger().info('Viewer closed by user. Exiting node...')
                    self.destroy_node()
                    rclpy.shutdown()
            return

        self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)

        hand_cam, hand_uv, hand_score, annotated_frame = self._detect_target_hand(frame)
        self.last_hand_uv = hand_uv
        self.last_hand_score = hand_score

        status_text = 'NO HAND'

        if hand_cam is None:
            if self.stable_frames > 0:
                self.get_logger().info('Target hand lost or depth invalid. Resetting EMA/stability state.')
            self._reset_stability()
        else:
            if self.ema is None:
                self.ema = hand_cam.copy()
            else:
                self.ema = self.ema_alpha * hand_cam + (1.0 - self.ema_alpha) * self.ema

            if self.last_ema is None:
                self.last_ema = self.ema.copy()
                self.stable_frames = 1
                status_text = 'TRACKING'
                self.get_logger().info(
                    f'Target hand detected. Start stable counting. '
                    f'raw cam = [{hand_cam[0]:.3f}, {hand_cam[1]:.3f}, {hand_cam[2]:.3f}] m, '
                    f'score = {hand_score:.3f}, valid_pts = {self.last_point_count}'
                )
            else:
                diff = float(np.linalg.norm(self.ema - self.last_ema))
                self.last_ema = self.ema.copy()

                if diff <= self.position_tolerance:
                    self.stable_frames += 1
                else:
                    self.get_logger().info(
                        f'Hand moved too much (EMA diff {diff:.4f} m > '
                        f'{self.position_tolerance:.4f} m). Restarting stable count.'
                    )
                    self.stable_frames = 1

                status_text = 'TRACKING'

            if self.stable_frames >= self.stable_frames_required:
                hand_base = self._transform_cam_to_base(self.ema)
                self._publish_pose(hand_base)

                self.published_once = True
                status_text = 'STABLE'

                self.get_logger().info(
                    'Stable target hand confirmed and published once. '
                    f'raw cam = [{hand_cam[0]:.3f}, {hand_cam[1]:.3f}, {hand_cam[2]:.3f}] m, '
                    f'ema cam = [{self.ema[0]:.3f}, {self.ema[1]:.3f}, {self.ema[2]:.3f}] m, '
                    f'base = [{hand_base[0]:.3f}, {hand_base[1]:.3f}, {hand_base[2]:.3f}] m, '
                    f'score = {hand_score:.3f}, valid_pts = {self.last_point_count}'
                )

                if self.show_viewer and annotated_frame is not None:
                    self._draw_overlay(annotated_frame, hand_cam, status_text)
                    cv2.imshow('ZED Right Hand Viewer', annotated_frame)
                    cv2.waitKey(1)

                time.sleep(self.exit_delay_after_publish)
                self.get_logger().info('Publish completed. Exiting node...')
                self.destroy_node()
                rclpy.shutdown()
                return

        if self.show_viewer and annotated_frame is not None:
            self._draw_overlay(annotated_frame, hand_cam, status_text)
            cv2.imshow('ZED Right Hand Viewer', annotated_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                self.get_logger().info('Viewer closed by user. Exiting node...')
                self.destroy_node()
                rclpy.shutdown()

    def _detect_target_hand(self, frame_bgr):
        """
        Detect target hand in 2D with MediaPipe, then obtain 3D camera-frame point
        from ZED point cloud around the selected fingertip pixel.

        Return:
            hand_3d: np.array([x, y, z]) in camera frame, meters
            hand_2d: np.array([u, v]) in image pixel coordinates
            score: handedness classification score
            annotated_frame: BGR image for viewer
        """
        if frame_bgr is None:
            return None, None, 0.0, None

        annotated = frame_bgr.copy()
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        if not results.multi_hand_landmarks or not results.multi_handedness:
            self.last_point_count = 0
            return None, None, 0.0, annotated

        h, w = frame_bgr.shape[:2]
        best = None

        for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
            if len(handedness.classification) == 0:
                continue

            cls = handedness.classification[0]
            label = cls.label
            score = float(cls.score)

            self.mp_drawing.draw_landmarks(
                annotated,
                hand_landmarks,
                self.mp_hands.HAND_CONNECTIONS,
                self.mp_drawing_styles.get_default_hand_landmarks_style(),
                self.mp_drawing_styles.get_default_hand_connections_style(),
            )

            if label != self.target_hand_label:
                continue

            u, v = self._compute_target_pixel(hand_landmarks, w, h)
            p_cam, valid_count = self._get_3d_point_from_window(u, v)

            if p_cam is None:
                continue

            candidate = {
                'p_cam': p_cam,
                'uv': np.array([u, v], dtype=np.float64),
                'score': score,
                'valid_count': valid_count,
            }

            if best is None or candidate['score'] > best['score']:
                best = candidate

        if best is None:
            self.last_point_count = 0
            return None, None, 0.0, annotated

        self.last_point_count = best['valid_count']
        return best['p_cam'], best['uv'], best['score'], annotated

    def _compute_target_pixel(self, hand_landmarks, img_w, img_h):
        """
        Choose handover target in image plane.
        Use index fingertip = landmark 8.
        """
        lm = hand_landmarks.landmark

        x = float(lm[8].x)   # INDEX_FINGER_TIP
        y = float(lm[8].y)

        u = int(np.clip(round(x * img_w), 0, img_w - 1))    # Convert normalized [0,1] to pixel coordinates and clip to image bounds
        v = int(np.clip(round(y * img_h), 0, img_h - 1))
        return u, v

    def _get_3d_point_from_window(self, u, v):
        """
        Robustly estimate 3D camera-frame point from a pixel neighborhood.
        Median is used to reduce outliers from noisy depth.
        """
        points = []
        r = self.depth_window_radius

        for dv in range(-r, r + 1):
            for du in range(-r, r + 1):
                uu = int(u + du)
                vv = int(v + dv)
                if uu < 0 or vv < 0:
                    continue

                err, value = self.point_cloud.get_value(uu, vv)
                if err != sl.ERROR_CODE.SUCCESS:
                    continue
                if value is None or len(value) < 3:
                    continue

                x, y, z = float(value[0]), float(value[1]), float(value[2])
                if not np.isfinite(x) or not np.isfinite(y) or not np.isfinite(z):
                    continue
                if np.linalg.norm([x, y, z]) < 1e-6:
                    continue

                points.append([x, y, z])

        if len(points) < self.depth_min_valid_points:
            return None, 0

        pts = np.array(points, dtype=np.float64)
        p_cam = np.median(pts, axis=0)
        return p_cam, len(points)

    def _transform_cam_to_base(self, p_cam):
        """Transform a 3D point from camera frame to robot base frame."""

        p_cam_h = np.array([p_cam[0], p_cam[1], p_cam[2], 1.0], dtype=np.float64)
        p_base_h = self.T_cam2base @ p_cam_h
        return p_base_h[:3]

    def _publish_pose(self, p_base):
        """Publish the detected hand position as a PoseStamped message."""

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

    def _draw_idle_overlay(self, frame):
        """Draw viewer text when the node is waiting for the start command."""

        h, _ = frame.shape[:2]
        cv2.putText(
            frame,
            'WAITING FOR /start_hand_detection',
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2
        )
        cv2.putText(
            frame,
            'MediaPipe Hands + ZED depth',
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )
        cv2.putText(
            frame,
            'Target: index fingertip',
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

    def _draw_overlay(self, frame, hand_cam, status_text):
        """Draw real-time debug information and hand tracking results on the viewer."""

        h, w = frame.shape[:2]

        if self.last_hand_uv is not None:
            u, v = int(self.last_hand_uv[0]), int(self.last_hand_uv[1])
            if 0 <= u < w and 0 <= v < h:
                cv2.circle(frame, (u, v), 8, (0, 0, 255), -1)
                cv2.circle(frame, (u, v), 16, (0, 255, 255), 2)
                cv2.putText(
                    frame,
                    'INDEX_TIP',
                    (u + 10, v - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2
                )

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
            f'StableFrames: {self.stable_frames} / {self.stable_frames_required}',
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 0),
            2
        )

        cv2.putText(
            frame,
            f'Hand score: {self.last_hand_score:.3f}',
            (20, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 200, 0),
            2
        )

        cv2.putText(
            frame,
            f'Valid depth pts: {self.last_point_count}',
            (20, 160),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 200, 0),
            2
        )

        if hand_cam is not None:
            cv2.putText(
                frame,
                f'Raw Cam XYZ: [{hand_cam[0]:.3f}, {hand_cam[1]:.3f}, {hand_cam[2]:.3f}] m',
                (20, 200),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

        if self.ema is not None:
            cv2.putText(
                frame,
                f'EMA Cam XYZ: [{self.ema[0]:.3f}, {self.ema[1]:.3f}, {self.ema[2]:.3f}] m',
                (20, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (180, 255, 180),
                2
            )

        cv2.putText(
            frame,
            f'Target hand: {self.target_hand_label}',
            (20, 280),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (200, 200, 255),
            2
        )

        cv2.putText(
            frame,
            'Target point: index fingertip',
            (20, 320),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (200, 255, 200),
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
        """Reset all stability-related tracking state."""
        self.ema = None
        self.last_ema = None
        self.stable_frames = 0
        self.last_hand_uv = None
        self.last_hand_score = 0.0
        self.last_point_count = 0

    def destroy_node(self):
        """Clean up OpenCV, MediaPipe, and ZED resources before shutdown."""
        self.get_logger().info('Shutting down zed_hand_node...')
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        try:
            if hasattr(self, 'hands') and self.hands is not None:
                self.hands.close()
        except Exception as e:
            self.get_logger().warn(f'Exception during MediaPipe shutdown: {e}')

        try:
            if hasattr(self, 'zed'):
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
