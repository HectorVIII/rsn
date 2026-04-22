import math
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import pyzed.sl as sl

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped

from ultralytics import YOLO


class InstrumentDetectionNode(Node):
    def __init__(self):
        super().__init__('instrument_detection_node')

        # ============================================================
        # Parameters
        # ============================================================
        self.declare_parameter(
            'model_path',
            '/home/huitao/MT/yolo_runs/black_tray_finetune/weights/best.pt'
        )
        self.declare_parameter('camera_serial', 27204693)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('img_size', 1280)
        self.declare_parameter('conf_threshold', 0.85)
        self.declare_parameter('min_mask_area', 500)

        self.declare_parameter('voice_target_topic', '/voice_target_instrument')
        self.declare_parameter('publish_topic', '/instrument_grasp_pose_base')

        self.declare_parameter('mask_threshold', 0.50)
        self.declare_parameter('close_kernel', 7)
        self.declare_parameter('open_kernel', 3)

        self.declare_parameter('center_ema_alpha', 0.18)
        self.declare_parameter('track_max_dist', 80.0)
        self.declare_parameter('track_max_missed', 20)

        self.declare_parameter('axis_percentile_low', 8.0)
        self.declare_parameter('axis_percentile_high', 92.0)

        self.declare_parameter('grasp_z_base_m', 0.0129)
        self.declare_parameter('x_offset_m', 0.028)
        self.declare_parameter('y_offset_m', -0.015)
        self.declare_parameter('hover_offset_m', 0.025)

        self.declare_parameter('show_viewer', True)
        self.declare_parameter('publish_once_then_stop', True)
        self.declare_parameter('target_stable_frames', 8)
        self.declare_parameter('exit_delay_after_publish', 0.5)

        self.model_path = str(self.get_parameter('model_path').value)
        self.camera_serial = int(self.get_parameter('camera_serial').value)
        self.camera_fps = int(self.get_parameter('camera_fps').value)
        self.img_size = int(self.get_parameter('img_size').value)
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.min_mask_area = float(self.get_parameter('min_mask_area').value)

        self.voice_target_topic = str(self.get_parameter('voice_target_topic').value)
        self.publish_topic = str(self.get_parameter('publish_topic').value)

        self.mask_threshold = float(self.get_parameter('mask_threshold').value)
        self.close_kernel = int(self.get_parameter('close_kernel').value)
        self.open_kernel = int(self.get_parameter('open_kernel').value)

        self.center_ema_alpha = float(self.get_parameter('center_ema_alpha').value)
        self.track_max_dist = float(self.get_parameter('track_max_dist').value)
        self.track_max_missed = int(self.get_parameter('track_max_missed').value)

        self.axis_percentile_low = float(self.get_parameter('axis_percentile_low').value)
        self.axis_percentile_high = float(self.get_parameter('axis_percentile_high').value)

        self.grasp_z_base_m = float(self.get_parameter('grasp_z_base_m').value)
        self.x_offset_m = float(self.get_parameter('x_offset_m').value)
        self.y_offset_m = float(self.get_parameter('y_offset_m').value)
        self.hover_offset_m = float(self.get_parameter('hover_offset_m').value)

        self.show_viewer = bool(self.get_parameter('show_viewer').value)
        self.publish_once_then_stop = bool(self.get_parameter('publish_once_then_stop').value)
        self.target_stable_frames = int(self.get_parameter('target_stable_frames').value)
        self.exit_delay_after_publish = float(self.get_parameter('exit_delay_after_publish').value)

        # ============================================================
        # Fixed transform camera -> xarm base
        # ============================================================
        self.T_cam2base = np.array([
            [-0.152816883,  0.733177385, -0.662644642,  0.694418397],
            [ 0.988175165,  0.104867217, -0.111860223,  0.431523428],
            [-0.012523686, -0.671903110, -0.740533165,  0.596290570],
            [ 0.0,          0.0,          0.0,          1.0        ],
        ], dtype=np.float64)

        # ============================================================
        # ROS interfaces
        # ============================================================
        self.target_sub = self.create_subscription(
            String,
            self.voice_target_topic,
            self.voice_target_callback,
            10
        )

        self.pose_pub = self.create_publisher(PoseStamped, self.publish_topic, 10)

        self.start_detection_service = self.create_service(
            Trigger,
            'start_instrument_detection',
            self.start_detection_callback
        )

        # ============================================================
        # Runtime state
        # ============================================================
        self.current_target_class: Optional[str] = None
        self.detection_enabled = False
        self.published_once = False
        self.shutdown_requested = False

        self.last_center_xy: Optional[np.ndarray] = None
        self.last_angle_deg: Optional[float] = None
        self.target_stable_count = 0

        # ============================================================
        # Model + camera
        # ============================================================
        self.model = YOLO(self.model_path)

        self.zed = sl.Camera()
        self.runtime = sl.RuntimeParameters()
        self.image_zed = sl.Mat()

        self._open_zed()
        self.fx, self.fy, self.cx, self.cy = self._get_left_camera_intrinsics()

        timer_period = 1.0 / float(self.camera_fps)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('instrument_detection_node started.')
        self.get_logger().info(f'Model path: {self.model_path}')
        self.get_logger().info(f'Voice target topic: {self.voice_target_topic}')
        self.get_logger().info(f'Publish topic: {self.publish_topic}')
        self.get_logger().info('Mode: wait for target + /start_instrument_detection, then publish grasp pose and exit.')

    # ============================================================
    # ROS callbacks
    # ============================================================
    def voice_target_callback(self, msg: String):
        self.current_target_class = msg.data.strip().upper()
        self.get_logger().info(f'Received voice target: {self.current_target_class}')
        self._reset_target_tracking()

    def start_detection_callback(self, request, response):
        if self.current_target_class is None:
            response.success = False
            response.message = 'No voice target received yet.'
            self.get_logger().warn(response.message)
            return response

        self.detection_enabled = True
        self.published_once = False
        self.shutdown_requested = False
        self._reset_target_tracking()

        response.success = True
        response.message = f'Instrument detection enabled for target: {self.current_target_class}'
        self.get_logger().info(response.message)
        return response

    # ============================================================
    # Main loop
    # ============================================================
    def timer_callback(self):
        if self.shutdown_requested:
            return

        if self.published_once and self.publish_once_then_stop:
            return

        if self.zed.grab(self.runtime) != sl.ERROR_CODE.SUCCESS:
            return

        self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT)
        frame_bgra = self.image_zed.get_data()
        frame_bgr = cv2.cvtColor(frame_bgra, cv2.COLOR_BGRA2BGR)

        display = frame_bgr.copy()

        if not self.detection_enabled:
            if self.show_viewer:
                self._draw_idle_overlay(display)
                cv2.imshow('Instrument Detection Viewer', display)
                self._handle_key()
            return

        if self.current_target_class is None:
            if self.show_viewer:
                cv2.putText(
                    display,
                    'NO TARGET CLASS',
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 0, 255),
                    2
                )
                cv2.imshow('Instrument Detection Viewer', display)
                self._handle_key()
            return

        result = self._detect_target(frame_bgr)

        if result is None:
            self.target_stable_count = 0
            self.last_center_xy = None
            self.last_angle_deg = None

            if self.show_viewer:
                cv2.putText(
                    display,
                    f'Target: {self.current_target_class} | NOT FOUND',
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2
                )
                cv2.imshow('Instrument Detection Viewer', display)
                self._handle_key()
            return

        center_xy, angle_deg, contour = result

        center_xy_np = np.array(center_xy, dtype=np.float64)

        if self.last_center_xy is None:
            self.last_center_xy = center_xy_np
            self.last_angle_deg = angle_deg
            self.target_stable_count = 1
        else:
            self.last_center_xy = (
                self.center_ema_alpha * center_xy_np
                + (1.0 - self.center_ema_alpha) * self.last_center_xy
            )
            self.last_angle_deg = angle_deg
            self.target_stable_count += 1

        center_xy_smooth = (
            int(round(self.last_center_xy[0])),
            int(round(self.last_center_xy[1]))
        )

        base_point = self._pixel_to_base_plane_xy(
            u=center_xy_smooth[0],
            v=center_xy_smooth[1],
        )

        if base_point is None:
            self.get_logger().warn('Base point projection failed.')
            self.target_stable_count = 0
            return

        bx, by, bz = base_point
        bx += self.x_offset_m
        by += self.y_offset_m

        if self.show_viewer:
            self._draw_target_overlay(
                display,
                contour,
                center_xy_smooth,
                angle_deg,
                (bx, by, bz)
            )
            cv2.imshow('Instrument Detection Viewer', display)
            self._handle_key()

        if self.target_stable_count >= self.target_stable_frames:
            self._publish_pose(
                x=bx,
                y=by,
                z=bz,
                yaw_deg=0.0
            )
            self.get_logger().info(
                f'Published grasp pose for {self.current_target_class}: '
                f'x={bx:.4f}, y={by:.4f}, z={bz:.4f}'
            )

            self.published_once = True
            self.detection_enabled = False
            self.shutdown_requested = True

            self.get_logger().info(
                f'Waiting {self.exit_delay_after_publish:.2f}s before shutdown to ensure pose is delivered...'
            )
            time.sleep(self.exit_delay_after_publish)

            self.get_logger().info('Detection finished. Releasing ZED camera and exiting node...')
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            return

    # ============================================================
    # Detection helpers
    # ============================================================
    def _detect_target(self, frame_bgr) -> Optional[Tuple[Tuple[int, int], float, np.ndarray]]:
        results = self.model.predict(
            frame_bgr,
            imgsz=self.img_size,
            conf=self.conf_threshold,
            verbose=False,
            retina_masks=True
        )

        if len(results) == 0:
            return None

        result = results[0]
        if result.masks is None or result.boxes is None:
            return None

        masks = result.masks.data
        h, w = frame_bgr.shape[:2]

        best_candidate = None
        best_conf = -1.0

        for i in range(int(masks.shape[0])):
            cls_id = int(result.boxes.cls[i].item())
            cls_name = str(result.names.get(cls_id, str(cls_id))).upper()
            conf = float(result.boxes.conf[i].item())

            if cls_name != self.current_target_class:
                continue

            mask_prob = masks[i].detach().cpu().numpy()
            if mask_prob.shape[:2] != (h, w):
                mask_prob = cv2.resize(mask_prob, (w, h), interpolation=cv2.INTER_NEAREST)

            mask = self._preprocess_mask(mask_prob)
            contour = self._largest_contour(mask)
            if contour is None:
                continue

            area = float(cv2.contourArea(contour))
            if area < self.min_mask_area:
                continue

            center_info = self._robust_oriented_center_from_contour(contour)
            if center_info is None:
                continue

            center_xy, angle_deg, _, _ = center_info

            if conf > best_conf:
                best_conf = conf
                best_candidate = (
                    (int(round(center_xy[0])), int(round(center_xy[1]))),
                    float(angle_deg),
                    contour
                )

        return best_candidate

    def _build_kernel(self, size: int) -> np.ndarray:
        size = max(1, int(size))
        if size % 2 == 0:
            size += 1
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))

    def _preprocess_mask(self, mask_prob: np.ndarray) -> np.ndarray:
        mask = (mask_prob > self.mask_threshold).astype(np.uint8) * 255
        if self.close_kernel > 1:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._build_kernel(self.close_kernel))
        if self.open_kernel > 1:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._build_kernel(self.open_kernel))
        return mask

    def _largest_contour(self, mask: np.ndarray) -> Optional[np.ndarray]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            return None
        return max(contours, key=cv2.contourArea)

    def _normalize(self, v: np.ndarray) -> np.ndarray:
        n = np.linalg.norm(v)
        if n < 1e-8:
            return v
        return v / n

    def _robust_oriented_center_from_contour(self, contour: np.ndarray):
        if contour is None or len(contour) < 20:
            return None

        pts = contour.reshape(-1, 2).astype(np.float32)
        mean = pts.mean(axis=0)
        centered = pts - mean

        cov = np.cov(centered.T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        order = np.argsort(eigvals)[::-1]
        eigvecs = eigvecs[:, order]

        major_axis = self._normalize(eigvecs[:, 0].astype(np.float32))
        minor_axis = np.array([-major_axis[1], major_axis[0]], dtype=np.float32)

        proj_major = centered @ major_axis
        proj_minor = centered @ minor_axis

        lo_major = np.percentile(proj_major, self.axis_percentile_low)
        hi_major = np.percentile(proj_major, self.axis_percentile_high)
        lo_minor = np.percentile(proj_minor, self.axis_percentile_low)
        hi_minor = np.percentile(proj_minor, self.axis_percentile_high)

        center_major = 0.5 * (lo_major + hi_major)
        center_minor = 0.5 * (lo_minor + hi_minor)

        center_xy = mean + center_major * major_axis + center_minor * minor_axis
        major_len = float(hi_major - lo_major)
        minor_len = float(hi_minor - lo_minor)
        angle_deg = math.degrees(math.atan2(float(major_axis[1]), float(major_axis[0])))

        if angle_deg < -90.0:
            angle_deg += 180.0
        elif angle_deg > 90.0:
            angle_deg -= 180.0

        return (float(center_xy[0]), float(center_xy[1])), float(angle_deg), major_len, minor_len

    # ============================================================
    # Geometry helpers
    # ============================================================
    def _open_zed(self):
        init = sl.InitParameters()
        init.set_from_serial_number(self.camera_serial)
        init.camera_resolution = sl.RESOLUTION.HD720
        init.camera_fps = self.camera_fps
        init.coordinate_units = sl.UNIT.METER
        init.depth_mode = sl.DEPTH_MODE.NONE
        init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE

        status = self.zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Failed to open ZED camera: {status}')

        self.get_logger().info('ZED camera opened successfully.')

    def _get_left_camera_intrinsics(self):
        info = self.zed.get_camera_information()
        calib = info.camera_configuration.calibration_parameters.left_cam
        return float(calib.fx), float(calib.fy), float(calib.cx), float(calib.cy)

    def _pixel_to_base_plane_xy(self, u: float, v: float):
        x_n = (float(u) - self.cx) / self.fx
        y_n = (float(v) - self.cy) / self.fy

        ray_cam = np.array([x_n, y_n, 1.0], dtype=np.float64)
        ray_cam /= np.linalg.norm(ray_cam)

        R = self.T_cam2base[:3, :3]
        t = self.T_cam2base[:3, 3]
        ray_base = R @ ray_cam
        cam_origin_base = t.copy()

        if abs(ray_base[2]) < 1e-9:
            return None

        scale = (self.grasp_z_base_m - cam_origin_base[2]) / ray_base[2]
        if scale <= 0.0:
            return None

        p_base = cam_origin_base + scale * ray_base
        return float(p_base[0]), float(p_base[1]), float(p_base[2])

    # ============================================================
    # Publish / draw / reset
    # ============================================================
    def _publish_pose(self, x: float, y: float, z: float, yaw_deg: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'xarm_base'

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.pose_pub.publish(msg)

    def _draw_idle_overlay(self, frame):
        cv2.putText(
            frame,
            'WAITING FOR /start_instrument_detection',
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2
        )

        if self.current_target_class is None:
            target_text = 'Voice target: NONE'
        else:
            target_text = f'Voice target: {self.current_target_class}'

        cv2.putText(
            frame,
            target_text,
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

    def _draw_target_overlay(self, frame, contour, center_xy, angle_deg, base_point):
        cv2.drawContours(frame, [contour], -1, (0, 255, 255), 2)

        x, y = center_xy
        cv2.circle(frame, (x, y), 6, (0, 0, 255), -1)

        theta = math.radians(angle_deg)
        dx = int(math.cos(theta) * 40)
        dy = int(math.sin(theta) * 40)
        cv2.line(frame, (x - dx, y - dy), (x + dx, y + dy), (255, 255, 0), 2)

        bx, by, bz = base_point
        cv2.putText(
            frame,
            f'Target: {self.current_target_class}',
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2
        )
        cv2.putText(
            frame,
            f'Center: ({x}, {y})  Stable: {self.target_stable_count}/{self.target_stable_frames}',
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )
        cv2.putText(
            frame,
            f'Base XYZ: [{bx:.3f}, {by:.3f}, {bz:.3f}] m',
            (20, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (180, 255, 180),
            2
        )

    def _handle_key(self):
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            self.get_logger().info('Viewer closed by user. Exiting...')
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def _reset_target_tracking(self):
        self.last_center_xy = None
        self.last_angle_deg = None
        self.target_stable_count = 0

    def destroy_node(self):
        self.get_logger().info('Shutting down instrument_detection_node...')
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

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
        node = InstrumentDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()