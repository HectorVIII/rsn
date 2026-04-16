import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np
import pyzed.sl as sl
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_srvs.srv import Trigger
from ultralytics import YOLO


# ============================================================
# DATA STRUCTURES
# ============================================================
@dataclass
class Detection:
    cls_id: int
    cls_name: str
    conf: float
    contour: np.ndarray
    mask: np.ndarray
    area: float
    center_xy: Tuple[float, float]
    angle_deg: float
    major_len: float
    minor_len: float


@dataclass
class Track:
    track_id: int
    cls_id: int
    cls_name: str
    center_xy: np.ndarray
    angle_deg: float
    major_len: float
    minor_len: float
    conf: float
    missed: int = 0


# ============================================================
# NODE
# ============================================================
class InstrumentDetectionNode(Node):
    def __init__(self):
        super().__init__('instrument_detection_node')

        # ----------------------------------------------------
        # Parameters
        # ----------------------------------------------------
        self.declare_parameter(
            'model_path',
            '/home/huitao/MT/Surgical-Tool-Segmentation-main/runs/segment/runs/segment/black_tray_ft_v1/weights/best.pt'
        )
        self.declare_parameter('camera_serial', 27204693)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('img_size', 1280)
        self.declare_parameter('conf_threshold', 0.85)
        self.declare_parameter('min_mask_area', 500)
        self.declare_parameter('target_class_names', [])

        # Camera-to-base transform
        self.declare_parameter(
            't_cam2base',
            [
                -0.152816883, 0.733177385, -0.662644642, 0.694418397,
                 0.988175165, 0.104867217, -0.111860223, 0.431523428,
                -0.012523686, -0.671903110, -0.740533165, 0.596290570,
                 0.0,         0.0,          0.0,          1.0,
            ]
        )

        # Fixed grasp plane height in xArm base frame
        self.declare_parameter('grasp_z_base_m', 0.0129)

        # Fixed XY compensation in xArm base frame
        self.declare_parameter('x_offset_m', 0.028)
        self.declare_parameter('y_offset_m', -0.015)

        # Hover target
        self.declare_parameter('hover_offset_m', 0.025)

        # Mask processing
        self.declare_parameter('mask_threshold', 0.50)
        self.declare_parameter('close_kernel', 7)
        self.declare_parameter('open_kernel', 3)

        # Tracking / smoothing
        self.declare_parameter('center_ema_alpha', 0.18)
        self.declare_parameter('angle_ema_alpha', 0.20)
        self.declare_parameter('track_max_dist', 80.0)
        self.declare_parameter('track_max_missed', 20)
        self.declare_parameter('axis_percentile_low', 8.0)
        self.declare_parameter('axis_percentile_high', 92.0)

        # Stability
        self.declare_parameter('stable_frames_required', 10)

        # Topics / services
        self.declare_parameter('grasp_pose_topic', '/instrument_grasp_pose_base')
        self.declare_parameter('hover_pose_topic', '/instrument_hover_pose_base')
        self.declare_parameter('start_service_name', 'start_instrument_detection')

        # Viewer
        self.declare_parameter('show_viewer', True)
        self.declare_parameter('window_name', 'Instrument Detection Viewer')
        self.declare_parameter('exit_delay_after_publish', 1.0)

        # ----------------------------------------------------
        # Read parameters
        # ----------------------------------------------------
        self.model_path = str(self.get_parameter('model_path').value)
        self.camera_serial = int(self.get_parameter('camera_serial').value)
        self.camera_fps = int(self.get_parameter('camera_fps').value)
        self.img_size = int(self.get_parameter('img_size').value)
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.min_mask_area = int(self.get_parameter('min_mask_area').value)

        target_class_names_param = list(self.get_parameter('target_class_names').value)
        if len(target_class_names_param) == 0:
            self.target_class_names = None
        else:
            self.target_class_names = {str(x).upper() for x in target_class_names_param}

        t_list = list(self.get_parameter('t_cam2base').value)
        self.T_cam2base = np.array(t_list, dtype=np.float64).reshape(4, 4)

        self.grasp_z_base_m = float(self.get_parameter('grasp_z_base_m').value)
        self.x_offset_m = float(self.get_parameter('x_offset_m').value)
        self.y_offset_m = float(self.get_parameter('y_offset_m').value)
        self.hover_offset_m = float(self.get_parameter('hover_offset_m').value)

        self.mask_threshold = float(self.get_parameter('mask_threshold').value)
        self.close_kernel = int(self.get_parameter('close_kernel').value)
        self.open_kernel = int(self.get_parameter('open_kernel').value)

        self.center_ema_alpha = float(self.get_parameter('center_ema_alpha').value)
        self.angle_ema_alpha = float(self.get_parameter('angle_ema_alpha').value)
        self.track_max_dist = float(self.get_parameter('track_max_dist').value)
        self.track_max_missed = int(self.get_parameter('track_max_missed').value)
        self.axis_percentile_low = float(self.get_parameter('axis_percentile_low').value)
        self.axis_percentile_high = float(self.get_parameter('axis_percentile_high').value)

        self.stable_frames_required = int(self.get_parameter('stable_frames_required').value)

        self.grasp_pose_topic = str(self.get_parameter('grasp_pose_topic').value)
        self.hover_pose_topic = str(self.get_parameter('hover_pose_topic').value)
        self.start_service_name = str(self.get_parameter('start_service_name').value)

        self.show_viewer = bool(self.get_parameter('show_viewer').value)
        self.window_name = str(self.get_parameter('window_name').value)
        self.exit_delay_after_publish = float(self.get_parameter('exit_delay_after_publish').value)

        # ----------------------------------------------------
        # Publishers / service
        # ----------------------------------------------------
        self.grasp_pose_pub = self.create_publisher(PoseStamped, self.grasp_pose_topic, 10)
        self.hover_pose_pub = self.create_publisher(PoseStamped, self.hover_pose_topic, 10)

        self.start_service = self.create_service(
            Trigger,
            self.start_service_name,
            self.start_detection_callback
        )

        # ----------------------------------------------------
        # State
        # ----------------------------------------------------
        self.model = YOLO(self.model_path)

        self.zed = sl.Camera()
        self.runtime = sl.RuntimeParameters()
        self.image_zed = sl.Mat()

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.tracks: List[Track] = []
        self.next_track_id = 1

        self.detection_enabled = False
        self.published_once = False
        self.last_valid_track_id = None
        self.last_stable_frames = 0
        self.latest_grasp_pose = None
        self.latest_hover_pose = None

        self.last_detection_info = ''

        # ----------------------------------------------------
        # Init camera
        # ----------------------------------------------------
        self._init_zed()
        self.fx, self.fy, self.cx, self.cy = self._get_left_camera_intrinsics()

        timer_period = 1.0 / float(self.camera_fps)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('instrument_detection_node started.')
        self.get_logger().info(f'Model path: {self.model_path}')
        self.get_logger().info(f'Camera serial: {self.camera_serial}')
        self.get_logger().info(f'Grasp pose topic: {self.grasp_pose_topic}')
        self.get_logger().info(f'Hover pose topic: {self.hover_pose_topic}')
        self.get_logger().info(f'Start service: /{self.start_service_name}')
        self.get_logger().info(f'Stable frames required: {self.stable_frames_required}')
        self.get_logger().info('Mode: wait for /start_instrument_detection, then publish once and exit.')

    # ========================================================
    # INIT / CALLBACKS
    # ========================================================
    def _init_zed(self) -> None:
        self.get_logger().info('Opening ZED camera...')

        init = sl.InitParameters()
        init.set_from_serial_number(self.camera_serial)
        init.camera_resolution = sl.RESOLUTION.HD720
        init.camera_fps = self.camera_fps
        init.coordinate_units = sl.UNIT.METER
        init.depth_mode = sl.DEPTH_MODE.NONE
        init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE

        status = self.zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Failed to open ZED camera: {repr(status)}')

        self.get_logger().info('ZED camera opened successfully.')

    def _get_left_camera_intrinsics(self) -> Tuple[float, float, float, float]:
        info = self.zed.get_camera_information()
        calib = info.camera_configuration.calibration_parameters.left_cam
        return float(calib.fx), float(calib.fy), float(calib.cx), float(calib.cy)

    def start_detection_callback(self, request, response):
        self.get_logger().info('START_INSTRUMENT_DETECTION command received')
        self.detection_enabled = True
        self.published_once = False
        self.last_valid_track_id = None
        self.last_stable_frames = 0
        self.latest_grasp_pose = None
        self.latest_hover_pose = None
        self.tracks = []
        self.next_track_id = 1
        response.success = True
        response.message = 'Instrument detection enabled.'
        return response

    def timer_callback(self) -> None:
        if self.published_once:
            return

        if self.zed.grab(self.runtime) != sl.ERROR_CODE.SUCCESS:
            return

        self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT)
        frame_bgra = self.image_zed.get_data()
        frame = cv2.cvtColor(frame_bgra, cv2.COLOR_BGRA2BGR)

        if not self.detection_enabled:
            if self.show_viewer:
                idle_frame = frame.copy()
                self._draw_idle_overlay(idle_frame)
                cv2.imshow(self.window_name, idle_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == 27 or key == ord('q'):
                    self.get_logger().info('Viewer closed by user. Exiting node...')
                    self.destroy_node()
                    rclpy.shutdown()
            return

        display = frame.copy()

        detections = self.detect_instruments(frame)
        self.next_track_id = self.update_tracks(self.tracks, detections, self.next_track_id)

        best_result = self._select_best_detection(detections)

        if best_result is None:
            self.last_valid_track_id = None
            self.last_stable_frames = 0
            self.latest_grasp_pose = None
            self.latest_hover_pose = None
            self.last_detection_info = 'NO VALID INSTRUMENT'
        else:
            det, tr, center_xy, angle_deg, major_len, base_point, hover_point = best_result

            track_id = tr.track_id if tr is not None else -1

            if self.last_valid_track_id == track_id:
                self.last_stable_frames += 1
            else:
                self.last_valid_track_id = track_id
                self.last_stable_frames = 1

            self.latest_grasp_pose = base_point
            self.latest_hover_pose = hover_point

            self.last_detection_info = (
                f'{det.cls_name} id={track_id} '
                f'px=({center_xy[0]}, {center_xy[1]}) '
                f'base=({base_point[0]:.4f}, {base_point[1]:.4f}, {base_point[2]:.4f}) '
                f'hover=({hover_point[0]:.4f}, {hover_point[1]:.4f}, {hover_point[2]:.4f}) '
                f'angle={angle_deg:.2f} stable={self.last_stable_frames}/{self.stable_frames_required}'
            )

            self.get_logger().info(self.last_detection_info)

            self._draw_detection_overlay(
                display=display,
                det=det,
                track=tr,
                center_xy=center_xy,
                angle_deg=angle_deg,
                major_len=major_len,
                base_point=base_point,
                hover_point=hover_point,
            )

            if self.last_stable_frames >= self.stable_frames_required:
                self._publish_pose(self.grasp_pose_pub, base_point, 'xarm_base')
                self._publish_pose(self.hover_pose_pub, hover_point, 'xarm_base')

                self.get_logger().info('Stable instrument grasp target confirmed and published once.')
                self.get_logger().info(
                    f'Published grasp pose: x={base_point[0]:.4f}, y={base_point[1]:.4f}, z={base_point[2]:.4f}'
                )
                self.get_logger().info(
                    f'Published hover pose: x={hover_point[0]:.4f}, y={hover_point[1]:.4f}, z={hover_point[2]:.4f}'
                )

                self.published_once = True

                if self.show_viewer:
                    cv2.imshow(self.window_name, display)
                    cv2.waitKey(1)

                time.sleep(self.exit_delay_after_publish)
                self.get_logger().info('Publish completed. Exiting node...')
                self.destroy_node()
                rclpy.shutdown()
                return

        if self.show_viewer:
            self._draw_status_overlay(display)
            cv2.imshow(self.window_name, display)
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                self.get_logger().info('Viewer closed by user. Exiting node...')
                self.destroy_node()
                rclpy.shutdown()

    # ========================================================
    # DETECTION LOGIC
    # ========================================================
    def normalize(self, v: np.ndarray) -> np.ndarray:
        n = np.linalg.norm(v)
        if n < 1e-8:
            return v
        return v / n

    def angle_wrap_deg(self, angle: float) -> float:
        while angle <= -180.0:
            angle += 360.0
        while angle > 180.0:
            angle -= 360.0
        return angle

    def angle_blend_deg(self, prev: float, new: float, alpha: float) -> float:
        diff = self.angle_wrap_deg(new - prev)
        return self.angle_wrap_deg(prev + alpha * diff)

    def build_kernel(self, size: int) -> np.ndarray:
        size = max(1, int(size))
        if size % 2 == 0:
            size += 1
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))

    def preprocess_mask(self, mask_prob: np.ndarray) -> np.ndarray:
        mask = (mask_prob > self.mask_threshold).astype(np.uint8) * 255
        if self.close_kernel > 1:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.build_kernel(self.close_kernel))
        if self.open_kernel > 1:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.build_kernel(self.open_kernel))
        return mask

    def largest_contour(self, mask: np.ndarray) -> Optional[np.ndarray]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            return None
        return max(contours, key=cv2.contourArea)

    def robust_oriented_center_from_contour(
        self,
        contour: np.ndarray
    ) -> Optional[Tuple[Tuple[float, float], float, float, float]]:
        if contour is None or len(contour) < 20:
            return None

        pts = contour.reshape(-1, 2).astype(np.float32)
        mean = pts.mean(axis=0)
        centered = pts - mean

        cov = np.cov(centered.T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        order = np.argsort(eigvals)[::-1]
        eigvecs = eigvecs[:, order]

        major_axis = self.normalize(eigvecs[:, 0].astype(np.float32))
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

    def detect_instruments(self, frame_bgr: np.ndarray) -> List[Detection]:
        results = self.model.predict(
            frame_bgr,
            imgsz=self.img_size,
            conf=self.conf_threshold,
            verbose=False,
            retina_masks=True
        )
        result = results[0]

        detections: List[Detection] = []
        if result.masks is None or result.boxes is None:
            return detections

        masks = result.masks.data
        h, w = frame_bgr.shape[:2]

        for i in range(int(masks.shape[0])):
            cls_id = int(result.boxes.cls[i].item())
            cls_name = str(result.names.get(cls_id, str(cls_id))).upper()
            conf = float(result.boxes.conf[i].item())

            if self.target_class_names is not None and cls_name not in self.target_class_names:
                continue

            mask_prob = masks[i].detach().cpu().numpy()
            if mask_prob.shape[:2] != (h, w):
                mask_prob = cv2.resize(mask_prob, (w, h), interpolation=cv2.INTER_NEAREST)

            mask = self.preprocess_mask(mask_prob)
            contour = self.largest_contour(mask)
            if contour is None:
                continue

            area = float(cv2.contourArea(contour))
            if area < self.min_mask_area:
                continue

            center_info = self.robust_oriented_center_from_contour(contour)
            if center_info is None:
                continue

            center_xy, angle_deg, major_len, minor_len = center_info
            detections.append(
                Detection(
                    cls_id=cls_id,
                    cls_name=cls_name,
                    conf=conf,
                    contour=contour,
                    mask=mask,
                    area=area,
                    center_xy=center_xy,
                    angle_deg=angle_deg,
                    major_len=major_len,
                    minor_len=minor_len,
                )
            )

        return detections

    def match_tracks(
        self,
        tracks: List[Track],
        detections: List[Detection]
    ) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
        pairs = []
        for ti, tr in enumerate(tracks):
            for di, det in enumerate(detections):
                if tr.cls_id != det.cls_id:
                    continue
                dx = tr.center_xy[0] - det.center_xy[0]
                dy = tr.center_xy[1] - det.center_xy[1]
                dist = math.hypot(dx, dy)
                if dist <= self.track_max_dist:
                    pairs.append((dist, ti, di))

        pairs.sort(key=lambda x: x[0])
        matched_t = set()
        matched_d = set()
        matches = []

        for _, ti, di in pairs:
            if ti in matched_t or di in matched_d:
                continue
            matched_t.add(ti)
            matched_d.add(di)
            matches.append((ti, di))

        unmatched_tracks = [i for i in range(len(tracks)) if i not in matched_t]
        unmatched_dets = [i for i in range(len(detections)) if i not in matched_d]
        return matches, unmatched_tracks, unmatched_dets

    def update_tracks(
        self,
        tracks: List[Track],
        detections: List[Detection],
        next_track_id: int
    ) -> int:
        matches, _, unmatched_dets = self.match_tracks(tracks, detections)

        for tr in tracks:
            tr.missed += 1

        for ti, di in matches:
            tr = tracks[ti]
            det = detections[di]
            tr.missed = 0
            tr.center_xy = (
                (1.0 - self.center_ema_alpha) * tr.center_xy
                + self.center_ema_alpha * np.array(det.center_xy, dtype=np.float32)
            )
            tr.angle_deg = self.angle_blend_deg(tr.angle_deg, det.angle_deg, self.angle_ema_alpha)
            tr.major_len = (
                (1.0 - self.center_ema_alpha) * tr.major_len
                + self.center_ema_alpha * det.major_len
            )
            tr.minor_len = (
                (1.0 - self.center_ema_alpha) * tr.minor_len
                + self.center_ema_alpha * det.minor_len
            )
            tr.conf = det.conf

        for di in unmatched_dets:
            det = detections[di]
            tracks.append(
                Track(
                    track_id=next_track_id,
                    cls_id=det.cls_id,
                    cls_name=det.cls_name,
                    center_xy=np.array(det.center_xy, dtype=np.float32),
                    angle_deg=det.angle_deg,
                    major_len=det.major_len,
                    minor_len=det.minor_len,
                    conf=det.conf,
                    missed=0,
                )
            )
            next_track_id += 1

        tracks[:] = [tr for tr in tracks if tr.missed <= self.track_max_missed]
        return next_track_id

    def find_track_for_detection(self, tracks: List[Track], det: Detection) -> Optional[Track]:
        best_track = None
        best_dist = float('inf')

        for tr in tracks:
            if tr.cls_id != det.cls_id:
                continue
            dx = tr.center_xy[0] - det.center_xy[0]
            dy = tr.center_xy[1] - det.center_xy[1]
            dist = math.hypot(dx, dy)
            if dist < best_dist and dist <= self.track_max_dist:
                best_dist = dist
                best_track = tr

        return best_track

    def pixel_to_base_plane_xy(
        self,
        u: float,
        v: float,
        z_base_plane: float
    ) -> Optional[Tuple[float, float, float]]:
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

        scale = (z_base_plane - cam_origin_base[2]) / ray_base[2]
        if scale <= 0.0:
            return None

        p_base = cam_origin_base + scale * ray_base
        return float(p_base[0]), float(p_base[1]), float(p_base[2])

    def _select_best_detection(self, detections: List[Detection]):
        best_result = None
        best_score = -1.0

        for det in detections:
            tr = self.find_track_for_detection(self.tracks, det)

            if tr is None:
                center_xy = (int(det.center_xy[0]), int(det.center_xy[1]))
                angle_deg = det.angle_deg
                major_len = det.major_len
                score = det.conf
            else:
                center_xy = (int(tr.center_xy[0]), int(tr.center_xy[1]))
                angle_deg = tr.angle_deg
                major_len = tr.major_len
                score = tr.conf

            base_point_raw = self.pixel_to_base_plane_xy(
                u=center_xy[0],
                v=center_xy[1],
                z_base_plane=self.grasp_z_base_m,
            )

            if base_point_raw is None:
                continue

            bx_raw, by_raw, bz_raw = base_point_raw
            base_point = (
                bx_raw + self.x_offset_m,
                by_raw + self.y_offset_m,
                bz_raw,
            )
            hover_point = (
                base_point[0],
                base_point[1],
                base_point[2] + self.hover_offset_m,
            )

            if score > best_score:
                best_score = score
                best_result = (det, tr, center_xy, angle_deg, major_len, base_point, hover_point)

        return best_result

    # ========================================================
    # PUBLISH / DRAW
    # ========================================================
    def _publish_pose(self, publisher, p_base: Tuple[float, float, float], frame_id: str) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.pose.position.x = float(p_base[0])
        msg.pose.position.y = float(p_base[1])
        msg.pose.position.z = float(p_base[2])

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        publisher.publish(msg)

    def draw_crosshair(self, img: np.ndarray, xy: Tuple[int, int], color=(0, 255, 255)) -> None:
        x, y = xy
        cv2.circle(img, (x, y), 6, color, 2, cv2.LINE_AA)
        cv2.line(img, (x - 12, y), (x + 12, y), (255, 255, 255), 1, cv2.LINE_AA)
        cv2.line(img, (x, y - 12), (x, y + 12), (255, 255, 255), 1, cv2.LINE_AA)

    def draw_axis(
        self,
        img: np.ndarray,
        center_xy: Tuple[int, int],
        angle_deg: float,
        major_len: float,
        color=(0, 255, 255)
    ) -> None:
        x, y = center_xy
        theta = math.radians(angle_deg)
        dx = int(math.cos(theta) * 0.5 * major_len)
        dy = int(math.sin(theta) * 0.5 * major_len)
        cv2.line(img, (x - dx, y - dy), (x + dx, y + dy), color, 2, cv2.LINE_AA)

    def _draw_idle_overlay(self, frame: np.ndarray) -> None:
        h, _ = frame.shape[:2]
        cv2.putText(
            frame,
            'WAITING FOR /start_instrument_detection',
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2
        )
        cv2.putText(
            frame,
            'YOLO segmentation + fixed grasp plane projection',
            (20, 80),
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

    def _draw_detection_overlay(
        self,
        display: np.ndarray,
        det: Detection,
        track: Optional[Track],
        center_xy: Tuple[int, int],
        angle_deg: float,
        major_len: float,
        base_point: Tuple[float, float, float],
        hover_point: Tuple[float, float, float],
    ) -> None:
        overlay = np.zeros_like(display)
        cv2.drawContours(overlay, [det.contour], -1, (255, 255, 0), -1)
        cv2.addWeighted(display, 1.0, overlay, 0.12, 0, dst=display)

        rect = cv2.minAreaRect(det.contour)
        box = cv2.boxPoints(rect).astype(np.int32)
        cv2.polylines(display, [box], True, (0, 180, 255), 2, cv2.LINE_AA)

        self.draw_crosshair(display, center_xy)
        self.draw_axis(display, center_xy, angle_deg, major_len)

        track_id = track.track_id if track is not None else -1

        text1 = f'{det.cls_name} id={track_id} conf={det.conf:.2f}'
        text2 = f'grasp_x_px={center_xy[0]} grasp_y_px={center_xy[1]} angle={angle_deg:.1f}'
        text3 = f'base_x={base_point[0]:.4f}m base_y={base_point[1]:.4f}m base_z={base_point[2]:.4f}m'
        text4 = f'hover_x={hover_point[0]:.4f}m hover_y={hover_point[1]:.4f}m hover_z={hover_point[2]:.4f}m'

        cv2.putText(display, text1, (center_xy[0] + 12, center_xy[1] - 14),
                    cv2.FONT_HERSHEY_DUPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(display, text2, (center_xy[0] + 12, center_xy[1] + 8),
                    cv2.FONT_HERSHEY_DUPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(display, text3, (center_xy[0] + 12, center_xy[1] + 30),
                    cv2.FONT_HERSHEY_DUPLEX, 0.55, (0, 220, 120), 1, cv2.LINE_AA)
        cv2.putText(display, text4, (center_xy[0] + 12, center_xy[1] + 52),
                    cv2.FONT_HERSHEY_DUPLEX, 0.55, (0, 180, 255), 1, cv2.LINE_AA)

    def _draw_status_overlay(self, display: np.ndarray) -> None:
        h, _ = display.shape[:2]

        cv2.putText(
            display,
            f'StableFrames: {self.last_stable_frames} / {self.stable_frames_required}',
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 0),
            2
        )

        cv2.putText(
            display,
            self.last_detection_info[:120],
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

        cv2.putText(
            display,
            'Press q or ESC to quit',
            (20, h - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (200, 200, 200),
            2
        )

    # ========================================================
    # CLEANUP
    # ========================================================
    def destroy_node(self):
        self.get_logger().info('Shutting down instrument_detection_node...')

        try:
            if self.zed is not None:
                self.zed.close()
        except Exception:
            pass

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        super().destroy_node()


# ============================================================
# MAIN
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = InstrumentDetectionNode()

    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()