"""
Classical Computer Vision Engine for the Raspberry Pi Zero W.
- Single-frame on-demand V4L2 capture (never runs background capture loops during motion)
- ArUco DICT_4X4_50 fiducial detection for Phase 2 Auto-Scale Calibration
- HSV thresholding, morphological cleanup, cv2.findContours(), and cv2.minAreaRect()
- Synthetic scene renderer for --mock-hardware mode so the full OpenCV pipeline
  can be verified on a laptop or headless Pi before physical camera mounting.
"""

from dataclasses import dataclass
import base64
import logging
import math
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from .config import GantryConfig
from .kinematics import EyeInHandKinematics

logger = logging.getLogger("gantry.vision")


@dataclass
class DetectedBlock:
    """Block detection in image pixel coordinates."""
    color: str
    u_px: float
    v_px: float
    angle_deg: float
    area_px: float
    box_pts: np.ndarray


@dataclass
class SimulatedCourseBlock:
    """Ground-truth virtual 20x20 mm block used to render synthetic camera frames in mock mode."""
    id: int
    color: str
    x_mm: float
    y_mm: float
    angle_deg: float
    mass_g: float
    size_mm: float = 20.0
    picked: bool = False


class VisionPipeline:
    """Classical OpenCV ArUco + HSV block detection pipeline."""

    def __init__(
        self,
        config: GantryConfig,
        kinematics: EyeInHandKinematics,
        mock_hardware: bool = False,
    ):
        self.config = config
        self.kinematics = kinematics
        self.mock_hardware = mock_hardware
        self._cap: Optional[cv2.VideoCapture] = None
        self.last_annotated_jpeg_b64: Optional[str] = None

        # Setup ArUco DICT_4X4_50 detector (compatible across OpenCV 4.5 - 4.10+)
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, "DetectorParameters"):
            self._aruco_params = cv2.aruco.DetectorParameters()
        else:
            self._aruco_params = cv2.aruco.DetectorParameters_create()

        # Pre-generate the 15x15 mm ArUco ID 0 marker image (60x60 px @ 0.25 mm/px)
        self._synthetic_marker_img = self._generate_aruco_patch(
            self.config.aruco_marker_id, side_px=48
        )

        # Ground-truth blocks for --mock-hardware mode
        self.simulated_blocks: List[SimulatedCourseBlock] = []
        self.reset_simulated_course()

    def reset_simulated_course(self) -> None:
        """Populates 20x20 mm blocks across the 4 mass classes inside the reachable course."""
        self.simulated_blocks = [
            SimulatedCourseBlock(1, "red", 80.0, 75.0, 28.0, 21.6),     # Aluminum (~21.6 g)
            SimulatedCourseBlock(2, "green", 155.0, 82.0, 64.0, 15.1),  # High infill (~15 g)
            SimulatedCourseBlock(3, "blue", 150.0, 125.0, 15.0, 10.2),  # Med infill (~10 g)
            SimulatedCourseBlock(4, "yellow", 88.0, 122.0, 42.0, 5.1),  # Low infill (~5 g)
        ]

    def open_camera(self) -> None:
        if self.mock_hardware:
            logger.info("VisionPipeline initialized in --mock-hardware mode (synthetic OpenCV renderer).")
            return

        logger.info("Opening UVC USB camera index %d...", self.config.camera_index)
        self._cap = cv2.VideoCapture(self.config.camera_index, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            self._cap = cv2.VideoCapture(self.config.camera_index)
        if not self._cap.isOpened():
            raise RuntimeError(f"Could not open UVC camera index {self.config.camera_index}")

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.frame_width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.frame_height)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def close_camera(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def capture_stationary_frame(
        self,
        camera_x_mm: float,
        camera_y_mm: float,
        camera_z_mm: float,
    ) -> np.ndarray:
        """
        Stop-and-Capture Rule:
        Captures a single static frame ONLY after the gantry has come to a complete
        mechanical stop. Flushes queued V4L2 frames so no motion-blurred buffer is read.
        """
        if self.mock_hardware:
            return self._render_synthetic_frame(camera_x_mm, camera_y_mm, camera_z_mm)

        if self._cap is None or not self._cap.isOpened():
            self.open_camera()

        assert self._cap is not None
        # Flush 3 stale frames from V4L2 ring buffer
        for _ in range(3):
            self._cap.grab()
        ok, frame = self._cap.read()
        if not ok or frame is None:
            raise RuntimeError("Failed to capture stationary frame from UVC camera.")
        return frame

    def detect_aruco_fiducial(
        self, frame: np.ndarray, target_id: Optional[int] = None
    ) -> Optional[Tuple[float, float, float]]:
        """
        Detects the calibration ArUco marker (DICT_4X4_50) and returns
        (center_u_px, center_v_px, angle_deg).
        """
        if target_id is None:
            target_id = self.config.aruco_marker_id

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if hasattr(cv2.aruco, "ArucoDetector"):
            detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._aruco_params)
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._aruco_dict, parameters=self._aruco_params
            )

        annotated = frame.copy()
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                if int(marker_id) == int(target_id):
                    pts = marker_corners[0]  # shape (4, 2)
                    u_px = float(np.mean(pts[:, 0]))
                    v_px = float(np.mean(pts[:, 1]))
                    dx = float(pts[1, 0] - pts[0, 0])
                    dy = float(pts[1, 1] - pts[0, 1])
                    angle_deg = math.degrees(math.atan2(dy, dx))
                    cv2.circle(annotated, (int(u_px), int(v_px)), 6, (0, 255, 0), -1)
                    self._encode_annotated_preview(annotated, f"ArUco ID {target_id}: ({u_px:.1f}, {v_px:.1f})")
                    return u_px, v_px, angle_deg

        self._encode_annotated_preview(annotated, "ArUco search: no marker in view")
        return None

    def detect_blocks_hsv(self, frame: np.ndarray) -> List[DetectedBlock]:
        """
        Classical HSV color space thresholding + cv2.findContours() + cv2.minAreaRect()
        to extract block centroids (u, v) and rotation angles (theta).
        """
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        detections: List[DetectedBlock] = []
        annotated = frame.copy()

        # Draw optical crosshair
        h, w = frame.shape[:2]
        cv2.drawMarker(
            annotated, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1
        )

        for color_name, hsv_cfg in self.config.hsv_ranges.items():
            mask = cv2.inRange(
                hsv,
                np.array(hsv_cfg.lower1, dtype=np.uint8),
                np.array(hsv_cfg.upper1, dtype=np.uint8),
            )
            if hsv_cfg.lower2 is not None and hsv_cfg.upper2 is not None:
                mask2 = cv2.inRange(
                    hsv,
                    np.array(hsv_cfg.lower2, dtype=np.uint8),
                    np.array(hsv_cfg.upper2, dtype=np.uint8),
                )
                mask = cv2.bitwise_or(mask, mask2)

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for cnt in contours:
                area = float(cv2.contourArea(cnt))
                if not (self.config.min_contour_area_px <= area <= self.config.max_contour_area_px):
                    continue

                rect = cv2.minAreaRect(cnt)
                (u_px, v_px), (rw, rh), angle = rect
                if rw < rh:
                    angle = angle + 90.0
                angle_deg = float(angle % 180.0)

                box = cv2.boxPoints(rect)
                box_int = np.int32(box)

                det = DetectedBlock(
                    color=color_name,
                    u_px=float(u_px),
                    v_px=float(v_px),
                    angle_deg=angle_deg,
                    area_px=area,
                    box_pts=box_int,
                )
                detections.append(det)

                cv2.drawContours(annotated, [box_int], 0, (0, 255, 0), 2)
                cv2.circle(annotated, (int(u_px), int(v_px)), 4, (255, 255, 255), -1)
                cv2.putText(
                    annotated,
                    f"{color_name.upper()} {angle_deg:.0f}deg",
                    (int(u_px) - 35, int(v_px) - 12),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

        self._encode_annotated_preview(annotated, f"HSV detections: {len(detections)}")
        return detections

    def _encode_annotated_preview(self, img: np.ndarray, label: str) -> None:
        cv2.putText(
            img,
            label,
            (10, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if ok:
            self.last_annotated_jpeg_b64 = base64.b64encode(buf.tobytes()).decode("ascii")

    # ------------------------------------------------------------------
    # Synthetic Camera Frame Renderer (--mock-hardware)
    # ------------------------------------------------------------------
    def _generate_aruco_patch(self, marker_id: int, side_px: int = 80) -> np.ndarray:
        if hasattr(cv2.aruco, "generateImageMarker"):
            patch = cv2.aruco.generateImageMarker(self._aruco_dict, marker_id, side_px)
        else:
            patch = np.zeros((side_px, side_px), dtype=np.uint8)
            cv2.aruco.drawMarker(self._aruco_dict, marker_id, side_px, patch, 1)
        # Add white quiet zone border required by ArUco detector
        bordered = cv2.copyMakeBorder(
            patch, 12, 12, 12, 12, cv2.BORDER_CONSTANT, value=255
        )
        return cv2.cvtColor(bordered, cv2.COLOR_GRAY2BGR)

    def _render_synthetic_frame(
        self, camera_x_mm: float, camera_y_mm: float, camera_z_mm: float
    ) -> np.ndarray:
        """
        Renders a top-down 640x480 BGR image of the course bed from the camera's
        current (camera_x_mm, camera_y_mm, camera_z_mm) viewpoint, including the
        ArUco calibration fiducial and all unpicked blocks.
        """
        w, h = self.config.frame_width, self.config.frame_height
        # Matte dark gray course surface
        frame = np.full((h, w, 3), (38, 42, 46), dtype=np.uint8)

        # Draw subtle 20mm grid lines on course floor
        for gx in range(0, int(self.config.workspace_x_mm) + 1, 20):
            u, _ = self.kinematics.world_to_pixel(
                gx, camera_y_mm, camera_x_mm, camera_y_mm, camera_z_mm, self.config.z_scan_mm
            )
            if 0 <= int(u) < w:
                cv2.line(frame, (int(u), 0), (int(u), h), (48, 52, 58), 1)
        for gy in range(0, int(self.config.workspace_y_mm) + 1, 20):
            _, v = self.kinematics.world_to_pixel(
                camera_x_mm, gy, camera_x_mm, camera_y_mm, camera_z_mm, self.config.z_scan_mm
            )
            if 0 <= int(v) < h:
                cv2.line(frame, (0, int(v)), (w, int(v)), (48, 52, 58), 1)

        # 1. Draw the 4 corner ArUco calibration fiducials (15x15 mm squares on 284x167.4 mm border)
        mh, mw = self._synthetic_marker_img.shape[:2]
        for fx_mm, fy_mm in self.config.corner_fiducials_mm:
            fu, fv = self.kinematics.world_to_pixel(
                fx_mm,
                fy_mm,
                camera_x_mm,
                camera_y_mm,
                camera_z_mm,
                self.config.z_scan_mm,
            )
            x0, y0 = int(round(fu - mw / 2)), int(round(fv - mh / 2))
            if 0 <= x0 and x0 + mw <= w and 0 <= y0 and y0 + mh <= h:
                frame[y0 : y0 + mh, x0 : x0 + mw] = self._synthetic_marker_img

        # 2. Draw scattered colored blocks in BGR matching our HSV ranges
        bgr_map: Dict[str, Tuple[int, int, int]] = {
            "red": (35, 35, 225),
            "green": (50, 205, 50),
            "blue": (230, 110, 25),
            "yellow": (20, 215, 235),
        }

        height_ratio = self.kinematics._height_ratio(camera_z_mm, self.config.z_scan_mm)
        px_per_mm = 1.0 / (self.kinematics.kx_mm_per_px * height_ratio)

        for blk in self.simulated_blocks:
            if blk.picked:
                continue
            bu, bv = self.kinematics.world_to_pixel(
                blk.x_mm,
                blk.y_mm,
                camera_x_mm,
                camera_y_mm,
                camera_z_mm,
                self.config.z_scan_mm,
            )
            side_px = blk.size_mm * px_per_mm
            if -side_px <= bu <= w + side_px and -side_px <= bv <= h + side_px:
                rect = ((float(bu), float(bv)), (float(side_px), float(side_px * 0.88)), float(blk.angle_deg))
                box = np.int32(cv2.boxPoints(rect))
                color_bgr = bgr_map.get(blk.color, (200, 200, 200))
                cv2.fillPoly(frame, [box], color_bgr)
                cv2.polylines(frame, [box], True, (20, 20, 20), 2, cv2.LINE_AA)

        return frame
