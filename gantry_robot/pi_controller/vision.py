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
import threading
import time
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

    @property
    def contour_area_px(self) -> float:
        """Alias for area_px to preserve backwards compatibility."""
        return self.area_px

    def to_dict(self) -> dict:
        return {
            "color": self.color,
            "u_px": round(self.u_px, 1),
            "v_px": round(self.v_px, 1),
            "angle_deg": round(self.angle_deg, 1),
            "area_px": round(self.area_px, 1),
        }


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
        self._cam_lock = threading.Lock()
        self.last_annotated_jpeg_b64: Optional[str] = None
        self.frame_seq: int = 0

        # Setup ArUco DICT_4X4_50 detector (compatible across OpenCV 4.5 - 4.10+)
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, "DetectorParameters"):
            self._aruco_params = cv2.aruco.DetectorParameters()
        else:
            self._aruco_params = cv2.aruco.DetectorParameters_create()

        # Pre-generate 15x15 mm ArUco markers for all configured corner fiducial IDs
        self._synthetic_markers: Dict[int, np.ndarray] = {
            int(mid): self._generate_aruco_patch(int(mid), side_px=60)
            for mid in self.config.corner_fiducials_by_id
        }
        self._synthetic_marker_img = self._synthetic_markers.get(
            self.config.aruco_marker_id,
            self._generate_aruco_patch(self.config.aruco_marker_id, side_px=60),
        )
        # Optional separate physical ground-truth kinematics for mock calibration tests
        self.mock_ground_truth_kinematics: Optional[EyeInHandKinematics] = None

        # Ground-truth blocks for --mock-hardware mode
        self.simulated_blocks: List[SimulatedCourseBlock] = []
        self.reset_simulated_course()

    def reset_simulated_course(self) -> None:
        """Populates 20x20 mm blocks across the 4 mass classes inside the reachable course."""
        self.simulated_blocks = [
            SimulatedCourseBlock(1, "white", 80.0, 75.0, 28.0, 21.6),  # Aluminum (~21.6 g)
            SimulatedCourseBlock(2, "green", 155.0, 82.0, 64.0, 9.75), # High infill (~9.75 g)
            SimulatedCourseBlock(3, "blue", 150.0, 125.0, 15.0, 7.05), # Med infill (~7.05 g)
            SimulatedCourseBlock(4, "red", 88.0, 122.0, 42.0, 2.46),   # Low infill (~2.46 g)
        ]

    def open_camera(self) -> None:
        if self.mock_hardware:
            logger.info("VisionPipeline initialized in --mock-hardware mode (synthetic OpenCV renderer).")
            return

        indices = [self.config.camera_index]
        for alt in (0, 1, 2, 4):
            if alt not in indices:
                indices.append(alt)

        for idx in indices:
            logger.info("Attempting to open UVC camera index %d...", idx)
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    self._cap = cap
                    logger.info("Successfully opened UVC camera at index %d", idx)
                    break
                cap.release()

        if self._cap is None or not self._cap.isOpened():
            raise RuntimeError(f"Could not open any UVC camera (tried indices {indices})")

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.frame_width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.frame_height)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # Disable backlight compensation to prevent dynamic exposure pumping on stationary metal
        try:
            import subprocess
            subprocess.run(
                ["v4l2-ctl", "-d", f"/dev/video{idx}", "-c", "backlight_compensation=0"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=1.0,
            )
        except Exception:
            pass

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

        with self._cam_lock:
            if self._cap is None or not self._cap.isOpened():
                self.open_camera()

            assert self._cap is not None
            # Let gantry vibrations settle before capturing to prevent motion blur
            settle_s = max(0.0, float(getattr(self.config, "scan_settle_s", 0.35)))
            if settle_s > 0.0:
                time.sleep(settle_s)
            # Flush stale frames from V4L2 ring buffer
            for _ in range(4):
                self._cap.grab()
            ok, frame = self._cap.read()
            if not ok or frame is None:
                raise RuntimeError("Failed to capture stationary frame from UVC camera.")
            return frame

    def capture_live_frame(
        self,
        camera_x_mm: Optional[float] = None,
        camera_y_mm: Optional[float] = None,
        camera_z_mm: Optional[float] = None,
    ) -> Optional[np.ndarray]:
        """
        Fast non-blocking grab for semi-live streaming.
        Flushes stale buffer so the preview is fresh without lag.
        """
        if self.mock_hardware:
            cx = camera_x_mm if camera_x_mm is not None else self.config.cam_homed_plate_xyz[0]
            cy = camera_y_mm if camera_y_mm is not None else self.config.cam_homed_plate_xyz[1]
            cz = camera_z_mm if camera_z_mm is not None else self.config.cam_homed_plate_xyz[2]
            return self._render_synthetic_frame(cx, cy, cz)

        with self._cam_lock:
            if self._cap is None or not self._cap.isOpened():
                try:
                    self.open_camera()
                except Exception:
                    return None

            if self._cap is None or not self._cap.isOpened():
                return None

            # Fast grab to discard any stale buffer frame
            self._cap.grab()
            ok, frame = self._cap.read()
            if not ok or frame is None:
                return None
            return frame

    def encode_fast_preview(self, frame: np.ndarray) -> None:
        """
        Lightweight fast preview encoding for live streaming in IDLE/manual mode.
        Draws the optical center crosshair in-place and compresses to JPEG without
        running heavy HSV morphology/ArUco loops or copying the 921 KB frame buffer.
        """
        h, w = frame.shape[:2]
        cx_img, cy_img = w // 2, h // 2
        cv2.line(frame, (cx_img - 20, cy_img), (cx_img + 20, cy_img), (200, 200, 200), 1)
        cv2.line(frame, (cx_img, cy_img - 20), (cx_img, cy_img + 20), (200, 200, 200), 1)
        cv2.circle(frame, (cx_img, cy_img), 3, (255, 255, 255), -1)

        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 52])
        if ok:
            self.last_annotated_jpeg_b64 = base64.b64encode(buf).decode("ascii")
            self.frame_seq += 1

    def _detect_raw_aruco(
        self, frame: np.ndarray
    ) -> Tuple[tuple, Optional[np.ndarray]]:
        """Detects DICT_4X4_50 markers in grayscale, falling back to CLAHE contrast boost in shadow."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if hasattr(cv2.aruco, "ArucoDetector"):
            detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._aruco_params)
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is None or len(ids) == 0:
                enhanced = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(gray)
                corners, ids, _ = detector.detectMarkers(enhanced)
            return corners, ids

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self._aruco_dict, parameters=self._aruco_params
        )
        if ids is None or len(ids) == 0:
            enhanced = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(gray)
            corners, ids, _ = cv2.aruco.detectMarkers(
                enhanced, self._aruco_dict, parameters=self._aruco_params
            )
        return corners, ids

    def _detect_aruco_markers_and_mask(
        self, frame: np.ndarray
    ) -> Tuple[tuple, Optional[np.ndarray], np.ndarray]:
        """Detects DICT_4X4_50 markers and builds an expanded binary exclusion mask."""
        h, w = frame.shape[:2]
        corners, ids = self._detect_raw_aruco(frame)

        exclusion_mask = np.zeros((h, w), dtype=np.uint8)
        if ids is not None and len(ids) > 0:
            for c in corners:
                pts = c[0]
                center = pts.mean(axis=0)
                expanded = center + 1.5 * (pts - center)
                cv2.fillPoly(exclusion_mask, [np.int32(expanded)], 255)
        return corners, ids, exclusion_mask

    def _extract_hsv_detections(
        self, frame: np.ndarray, exclusion_mask: Optional[np.ndarray] = None
    ) -> List[DetectedBlock]:
        """Runs HSV thresholding + contour geometry filtering, excluding ArUco marker regions."""
        h, w = frame.shape[:2]
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        detections: List[DetectedBlock] = []
        k_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        k_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        has_exclusion = exclusion_mask is not None and np.any(exclusion_mask)
        inv_exclusion = cv2.bitwise_not(exclusion_mask) if has_exclusion else None

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

            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_open)
            if inv_exclusion is not None:
                mask = cv2.bitwise_and(mask, inv_exclusion)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for cnt in contours:
                area = float(cv2.contourArea(cnt))
                if not (self.config.min_contour_area_px <= area <= self.config.max_contour_area_px):
                    continue

                bx, by, bw, bh = cv2.boundingRect(cnt)
                if bx <= 3 or by <= 3 or bx + bw >= w - 3 or by + bh >= h - 3:
                    continue

                rect = cv2.minAreaRect(cnt)
                (u_px, v_px), (rw, rh), angle = rect
                aspect = max(rw, rh) / max(1e-3, min(rw, rh))
                if aspect > 1.65:
                    continue

                solidity = area / max(1e-3, (rw * rh))
                if solidity < 0.60:
                    continue

                if rw < rh:
                    angle = angle + 90.0
                angle_deg = round(float(angle % 180.0), 1)
                box_int = np.int32(cv2.boxPoints(rect))

                detections.append(
                    DetectedBlock(
                        color=color_name,
                        u_px=float(u_px),
                        v_px=float(v_px),
                        angle_deg=angle_deg,
                        area_px=area,
                        box_pts=box_int,
                    )
                )
        return detections

    def process_live_frame(
        self, frame: np.ndarray
    ) -> Tuple[List[dict], List[DetectedBlock]]:
        """
        Processes a live video frame by simultaneously detecting all ArUco fiducials
        and HSV color blocks, rendering high-visibility angle vectors, centroids,
        and HUD overlays, and encoding the preview for the dashboard.
        """
        annotated = frame.copy()
        h, w = frame.shape[:2]

        # 1. Optical center crosshair with tick marks at center (w//2, h//2)
        cx_img, cy_img = w // 2, h // 2
        cv2.line(annotated, (cx_img - 20, cy_img), (cx_img + 20, cy_img), (200, 200, 200), 1)
        cv2.line(annotated, (cx_img, cy_img - 20), (cx_img, cy_img + 20), (200, 200, 200), 1)
        cv2.circle(annotated, (cx_img, cy_img), 3, (255, 255, 255), -1)

        # 2. Detect all standard ArUco Fiducials (DICT_4X4_50) and exclusion mask
        corners, ids, aruco_exclusion_mask = self._detect_aruco_markers_and_mask(frame)
        detected_fiducials: List[dict] = []

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                m_id = int(marker_id)
                pts = marker_corners[0]
                u_px = float(np.mean(pts[:, 0]))
                v_px = float(np.mean(pts[:, 1]))
                dx = float(pts[1, 0] - pts[0, 0])
                dy = float(pts[1, 1] - pts[0, 1])
                angle_deg = round(math.degrees(math.atan2(dy, dx)), 1)
                detected_fiducials.append({
                    "id": m_id,
                    "u_px": round(u_px, 1),
                    "v_px": round(v_px, 1),
                    "angle_deg": angle_deg,
                })
                rad = math.radians(angle_deg)
                arrow_len = 35
                ax = int(u_px + arrow_len * math.cos(rad))
                ay = int(v_px + arrow_len * math.sin(rad))
                cv2.arrowedLine(annotated, (int(u_px), int(v_px)), (ax, ay), (255, 128, 0), 2, tipLength=0.25)
                cv2.circle(annotated, (int(u_px), int(v_px)), 5, (0, 255, 0), -1)

                lbl = f"ARUCO #{m_id} {angle_deg:+.1f}deg"
                (lw, lh), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                lx, ly = max(5, int(u_px) - lw // 2), max(lh + 5, int(v_px) - 20)
                cv2.rectangle(annotated, (lx - 2, ly - lh - 2), (lx + lw + 2, ly + 4), (0, 0, 0), -1)
                cv2.putText(annotated, lbl, (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

        # 3. Detect all HSV Color Blocks (excluding ArUco fiducial regions)
        color_bgr_map = {
            "red": (0, 0, 255),
            "green": (0, 255, 0),
            "blue": (255, 120, 0),
            "yellow": (0, 230, 255),
            "white": (255, 255, 255),
        }

        detected_blocks = self._extract_hsv_detections(frame, aruco_exclusion_mask)
        for det in detected_blocks:
            bgr = color_bgr_map.get(det.color, (255, 255, 255))
            cv2.drawContours(annotated, [det.box_pts], 0, bgr, 2)
            cv2.circle(annotated, (int(det.u_px), int(det.v_px)), 4, (255, 255, 255), -1)

            rad = math.radians(det.angle_deg)
            arrow_len = 30
            ax = int(det.u_px + arrow_len * math.cos(rad))
            ay = int(det.v_px + arrow_len * math.sin(rad))
            cv2.arrowedLine(annotated, (int(det.u_px), int(det.v_px)), (ax, ay), (0, 255, 0), 2, tipLength=0.3)

            lbl = f"{det.color.upper()} {det.angle_deg:.1f}deg"
            (lw, lh), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            lx, ly = max(5, int(det.u_px) - lw // 2), max(lh + 5, int(det.v_px) - 14)
            cv2.rectangle(annotated, (lx - 2, ly - lh - 2), (lx + lw + 2, ly + 4), (0, 0, 0), -1)
            cv2.putText(annotated, lbl, (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        # 4. Top HUD Header Banner
        hud_text = f"LIVE CAM (~3 FPS) | Fiducials: {len(detected_fiducials)} | Blocks: {len(detected_blocks)}"
        cv2.rectangle(annotated, (0, 0), (w, 24), (20, 25, 35), -1)
        cv2.circle(annotated, (12, 12), 5, (0, 255, 0), -1)
        cv2.putText(annotated, hud_text, (26, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (240, 240, 240), 1, cv2.LINE_AA)

        # 5. JPEG encode at quality 70 for responsive network transmission
        ok, buf = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if ok:
            self.last_annotated_jpeg_b64 = base64.b64encode(buf.tobytes()).decode("ascii")
            self.frame_seq += 1

        return detected_fiducials, detected_blocks

    def detect_all_aruco_fiducials(
        self, frame: np.ndarray
    ) -> List[Tuple[int, float, float, float]]:
        """
        Detects all DICT_4X4_50 ArUco markers in `frame` and returns a list of
        (marker_id, center_u_px, center_v_px, angle_deg), sorted closest to center first.
        """
        corners, ids = self._detect_raw_aruco(frame)

        annotated = frame.copy()
        results: List[Tuple[float, int, float, float, float]] = []
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            cx, cy = self.config.frame_width / 2.0, self.config.frame_height / 2.0
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                m_id = int(marker_id)
                pts = marker_corners[0]
                u_px = float(np.mean(pts[:, 0]))
                v_px = float(np.mean(pts[:, 1]))
                dx = float(pts[1, 0] - pts[0, 0])
                dy = float(pts[1, 1] - pts[0, 1])
                angle_deg = math.degrees(math.atan2(dy, dx))
                dist_center = (u_px - cx) ** 2 + (v_px - cy) ** 2
                results.append((dist_center, m_id, u_px, v_px, angle_deg))
                cv2.circle(annotated, (int(u_px), int(v_px)), 6, (0, 255, 0), -1)

        if results:
            results.sort(key=lambda c: c[0])
            summary = ", ".join(f"#{r[1]}({r[2]:.0f},{r[3]:.0f})" for r in results)
            self._encode_annotated_preview(annotated, f"ArUco: {summary}")
        else:
            self._encode_annotated_preview(annotated, "ArUco search: no marker in view")

        return [(r[1], r[2], r[3], r[4]) for r in results]

    def detect_aruco_fiducial(
        self, frame: np.ndarray, target_id: Optional[int] = None
    ) -> Optional[Tuple[int, float, float, float]]:
        """
        Detects an ArUco calibration marker (DICT_4X4_50).
        If target_id is specified, looks for that specific marker ID.
        If target_id is None, detects any corner fiducial from config (defaulting to
        the marker closest to image center) and returns (marker_id, center_u_px, center_v_px, angle_deg).
        """
        all_markers = self.detect_all_aruco_fiducials(frame)
        for m_id, u_px, v_px, angle_deg in all_markers:
            if target_id is None or m_id == int(target_id):
                return m_id, u_px, v_px, angle_deg
        return None

    def detect_blocks_hsv(self, frame: np.ndarray) -> List[DetectedBlock]:
        """
        Classical HSV color space thresholding + cv2.findContours() + cv2.minAreaRect()
        to extract block centroids (u, v) and rotation angles (theta).
        """
        annotated = frame.copy()
        color_bgr_map = {
            "red": (0, 0, 255),
            "green": (0, 255, 0),
            "blue": (255, 120, 0),
            "yellow": (0, 230, 255),
            "white": (255, 255, 255),
        }

        h, w = frame.shape[:2]
        cv2.drawMarker(
            annotated, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1
        )

        _, _, aruco_exclusion_mask = self._detect_aruco_markers_and_mask(frame)
        detections = self._extract_hsv_detections(frame, aruco_exclusion_mask)

        for det in detections:
            bgr = color_bgr_map.get(det.color, (255, 255, 255))
            cv2.drawContours(annotated, [det.box_pts], 0, bgr, 2)
            cv2.circle(annotated, (int(det.u_px), int(det.v_px)), 4, (255, 255, 255), -1)

            lbl = f"{det.color.upper()} {det.angle_deg:.0f}deg"
            (lw, lh), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            lx, ly = max(5, int(det.u_px) - lw // 2), max(lh + 5, int(det.v_px) - 14)
            cv2.rectangle(annotated, (lx - 2, ly - lh - 2), (lx + lw + 2, ly + 4), (0, 0, 0), -1)
            cv2.putText(
                annotated,
                lbl,
                (lx, ly),
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
            self.frame_seq += 1

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
        current (camera_x_mm, camera_y_mm, camera_z_mm) viewpoint, including all 4
        corner ArUco calibration fiducials and all unpicked blocks.
        """
        w, h = self.config.frame_width, self.config.frame_height
        render_kin = self.mock_ground_truth_kinematics or self.kinematics
        # Matte dark gray course surface
        frame = np.full((h, w, 3), (38, 42, 46), dtype=np.uint8)

        # Draw subtle 20mm grid lines on course floor
        for gx in range(0, int(self.config.workspace_x_mm) + 1, 20):
            u, _ = render_kin.world_to_pixel(
                gx, camera_y_mm, camera_x_mm, camera_y_mm, camera_z_mm, self.config.z_scan_mm
            )
            if 0 <= int(u) < w:
                cv2.line(frame, (int(u), 0), (int(u), h), (48, 52, 58), 1)
        for gy in range(0, int(self.config.workspace_y_mm) + 1, 20):
            _, v = render_kin.world_to_pixel(
                camera_x_mm, gy, camera_x_mm, camera_y_mm, camera_z_mm, self.config.z_scan_mm
            )
            if 0 <= int(v) < h:
                cv2.line(frame, (0, int(v)), (w, int(v)), (48, 52, 58), 1)

        # 1. Draw the 4 corner ArUco calibration fiducials (15x15 mm + 3mm quiet border = 21x21 mm)
        half_plate_mm = self.config.fiducial_size_mm / 2.0 + 3.0
        for mid, (fx_mm, fy_mm) in self.config.corner_fiducials_by_id.items():
            patch = self._synthetic_markers.get(mid, self._synthetic_marker_img)
            ph, pw = patch.shape[:2]
            dst_quad = np.array(
                [
                    render_kin.world_to_pixel(
                        fx_mm - half_plate_mm, fy_mm + half_plate_mm,
                        camera_x_mm, camera_y_mm, camera_z_mm, self.config.z_scan_mm,
                    ),
                    render_kin.world_to_pixel(
                        fx_mm + half_plate_mm, fy_mm + half_plate_mm,
                        camera_x_mm, camera_y_mm, camera_z_mm, self.config.z_scan_mm,
                    ),
                    render_kin.world_to_pixel(
                        fx_mm + half_plate_mm, fy_mm - half_plate_mm,
                        camera_x_mm, camera_y_mm, camera_z_mm, self.config.z_scan_mm,
                    ),
                    render_kin.world_to_pixel(
                        fx_mm - half_plate_mm, fy_mm - half_plate_mm,
                        camera_x_mm, camera_y_mm, camera_z_mm, self.config.z_scan_mm,
                    ),
                ],
                dtype=np.float32,
            )
            min_x = float(np.min(dst_quad[:, 0]))
            max_x = float(np.max(dst_quad[:, 0]))
            min_y = float(np.min(dst_quad[:, 1]))
            max_y = float(np.max(dst_quad[:, 1]))
            if max_x < 0 or min_x >= w or max_y < 0 or min_y >= h:
                continue
            src_quad = np.array(
                [[0, 0], [pw, 0], [pw, ph], [0, ph]], dtype=np.float32
            )
            M = cv2.getPerspectiveTransform(src_quad, dst_quad)
            warped = cv2.warpPerspective(patch, M, (w, h), flags=cv2.INTER_LINEAR)
            mask_poly = np.int32(dst_quad)
            cv2.fillConvexPoly(frame, mask_poly, (0, 0, 0))
            mask = np.zeros((h, w), dtype=np.uint8)
            cv2.fillConvexPoly(mask, mask_poly, 255)
            frame[mask > 0] = warped[mask > 0]

        # 2. Draw scattered colored blocks in BGR matching our HSV ranges
        bgr_map: Dict[str, Tuple[int, int, int]] = {
            "red": (35, 35, 225),
            "green": (50, 205, 50),
            "blue": (230, 110, 25),
            "yellow": (20, 215, 235),
            "white": (245, 245, 245),
        }

        height_ratio = render_kin._height_ratio(
            camera_z_mm, self.config.z_scan_mm, self.config.block_height_mm
        )
        px_per_mm = 1.0 / (render_kin.kx_mm_per_px * height_ratio)

        for blk in self.simulated_blocks:
            if blk.picked:
                continue
            bu, bv = render_kin.world_to_pixel(
                blk.x_mm,
                blk.y_mm,
                camera_x_mm,
                camera_y_mm,
                camera_z_mm,
                self.config.z_scan_mm,
                target_height_above_surface_mm=self.config.block_top_height_mm_at(
                    blk.x_mm, blk.y_mm
                ),
            )
            side_px = blk.size_mm * px_per_mm
            if -side_px <= bu <= w + side_px and -side_px <= bv <= h + side_px:
                rect_angle = float(blk.angle_deg) + float(render_kin.cam_skew_deg)
                rect = ((float(bu), float(bv)), (float(side_px), float(side_px * 0.88)), rect_angle)
                box = np.int32(cv2.boxPoints(rect))
                color_bgr = bgr_map.get(blk.color, (200, 200, 200))
                cv2.fillPoly(frame, [box], color_bgr)
                cv2.polylines(frame, [box], True, (20, 20, 20), 2, cv2.LINE_AA)

        return frame
