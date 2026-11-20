"""Eye-in-Hand coordinate transformations, block deduplication, and path planning."""

from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Set, Tuple

import numpy as np


@dataclass
class BlockRecord:
    """Cataloged 20x20 mm block in the virtual environment model."""
    id: int
    color: str
    x_mm: float
    y_mm: float
    angle_deg: float
    mass_g: float = 0.0
    mass_class: str = "unknown"  # aluminum, high_infill, med_infill, low_infill
    status: str = "DETECTED"     # DETECTED, QUEUED, PICKING, WEIGHED, STACKED
    confidence_views: int = 1

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "color": self.color,
            "x_mm": round(self.x_mm, 2),
            "y_mm": round(self.y_mm, 2),
            "angle_deg": round(self.angle_deg, 1),
            "mass_g": round(self.mass_g, 2),
            "mass_class": self.mass_class,
            "status": self.status,
        }


class EyeInHandKinematics:
    """
    Handles conversions between:
      - Motor travel coordinates (motor_x in [0..227], motor_y in [0..128], motor_z_down in [0..70.36])
      - Course/Fiducial Plate coordinates (plate_x in [0..284], plate_y in [0..167.4], plate_z above bed)
      - Camera pixel coordinates (u_px, v_px)
    """

    def __init__(
        self,
        frame_width: int = 640,
        frame_height: int = 480,
        kx_mm_per_px: float = 0.25,
        ky_mm_per_px: float = 0.25,
        cam_skew_deg: float = 0.0,
        offset_x_mm: float = -1.5,
        offset_y_mm: float = -41.6,
        cam_homed_z_above_plate_mm: float = 135.4,
        course_elevation_mm: float = 6.35,
        cam_origin_x_mm: float = 32.5,
        cam_origin_y_mm: float = 33.1,
        corner_fiducials: Optional[Dict[int, Tuple[float, float]]] = None,
    ):
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.cx_px = frame_width / 2.0
        self.cy_px = frame_height / 2.0
        self.kx_mm_per_px = kx_mm_per_px
        self.ky_mm_per_px = ky_mm_per_px
        self.cam_skew_deg = cam_skew_deg
        self.offset_x_mm = offset_x_mm
        self.offset_y_mm = offset_y_mm
        self.cam_homed_z_above_plate_mm = cam_homed_z_above_plate_mm
        self.course_elevation_mm = course_elevation_mm
        self.cam_origin_x_mm = cam_origin_x_mm
        self.cam_origin_y_mm = cam_origin_y_mm
        self.corner_fiducials = corner_fiducials or {
            0: (7.5, 7.5),
            1: (7.5, 159.9),
            2: (276.5, 159.9),
            3: (276.5, 7.5),
        }
        self.calibrated = False
        self.origin_calibrated = False

        # 4-Corner Ground-Truth Plate Transform state (Motor-Frame Camera <-> Plate)
        self.cam_motor_skew_deg: float = cam_skew_deg
        self.plate_yaw_deg: float = 0.0
        self.plate_scale_x: float = 1.0
        self.plate_scale_y: float = 1.0
        self.calibration_mode: str = "nominal"
        self.markers_detected: List[int] = []
        self.residual_rms_mm: Optional[float] = None
        self.residual_max_mm: Optional[float] = None
        self.corner_residuals_mm: Dict[int, float] = {}
        self._plate_A: Optional[np.ndarray] = None      # 2x2 matrix: P = A @ C + t
        self._plate_t: Optional[np.ndarray] = None      # 2-vector
        self._plate_A_inv: Optional[np.ndarray] = None  # 2x2 inverse
        self._plate_t_inv: Optional[np.ndarray] = None  # 2-vector inverse
        self.drift_x_mm: float = 0.0
        self.drift_y_mm: float = 0.0

    @staticmethod
    def _wrap_deg(angle_deg: float) -> float:
        return ((angle_deg + 180.0) % 360.0) - 180.0

    @staticmethod
    def _rotate_2d(x: float, y: float, angle_deg: float) -> Tuple[float, float]:
        if abs(angle_deg) <= 1e-4:
            return x, y
        rad = math.radians(angle_deg)
        c, s = math.cos(rad), math.sin(rad)
        return x * c - y * s, x * s + y * c

    @staticmethod
    def _fit_similarity_2d(src_pts: np.ndarray, dst_pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Closed-form 2D Procrustes similarity fit: dst = A @ src + t (rotation + uniform scale + translation)."""
        src_mean = src_pts.mean(axis=0)
        dst_mean = dst_pts.mean(axis=0)
        ds = src_pts - src_mean
        dd = dst_pts - dst_mean
        denom = float(np.sum(ds[:, 0] ** 2 + ds[:, 1] ** 2))
        if denom < 1e-6:
            raise ValueError("Degenerate fiducial geometry (collocated points).")
        a = float(np.sum(ds[:, 0] * dd[:, 0] + ds[:, 1] * dd[:, 1])) / denom
        b = float(np.sum(ds[:, 0] * dd[:, 1] - ds[:, 1] * dd[:, 0])) / denom
        A = np.array([[a, -b], [b, a]], dtype=float)
        t = dst_mean - A @ src_mean
        return A, t

    @staticmethod
    def _fit_affine_2d(src_pts: np.ndarray, dst_pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Least-squares 6-DOF 2D affine fit: dst = A @ src + t."""
        n = src_pts.shape[0]
        design = np.column_stack([src_pts, np.ones(n, dtype=float)])
        px, _, _, _ = np.linalg.lstsq(design, dst_pts[:, 0], rcond=None)
        py, _, _, _ = np.linalg.lstsq(design, dst_pts[:, 1], rcond=None)
        A = np.array([[px[0], px[1]], [py[0], py[1]]], dtype=float)
        t = np.array([px[2], py[2]], dtype=float)
        return A, t

    @staticmethod
    def _is_valid_plate_affine(A: np.ndarray) -> bool:
        det = float(np.linalg.det(A))
        if det <= 0.0:
            return False
        sx = math.hypot(float(A[0, 0]), float(A[1, 0]))
        sy = math.hypot(float(A[0, 1]), float(A[1, 1]))
        if not (0.95 <= sx <= 1.05 and 0.95 <= sy <= 1.05):
            return False
        # Verify axes remain nearly orthogonal (within +/- 5 deg of 90 deg)
        dot = float(A[0, 0] * A[0, 1] + A[1, 0] * A[1, 1])
        cos_ortho = abs(dot) / max(1e-6, sx * sy)
        return cos_ortho <= math.sin(math.radians(5.0))

    def _height_ratio(
        self,
        z_down_mm: float,
        z_scan_down_mm: float = 0.0,
        target_height_above_surface_mm: float = 0.0,
    ) -> float:
        """
        Since Z=0 is at the TOP when homed (camera 135.4 mm above plate, 129.05 mm above elevated course)
        and +Z_down moves DOWNWARD, camera distance to the target plane (e.g. +20.0 mm top face of a cube)
        is (cam_homed_z - course_elevation - z_down_mm - target_height_above_surface_mm).
        """
        cam_above_course = self.cam_homed_z_above_plate_mm - self.course_elevation_mm
        cam_h_now = max(25.0, cam_above_course - z_down_mm - target_height_above_surface_mm)
        cam_h_scan = max(25.0, cam_above_course - z_scan_down_mm)
        return cam_h_now / cam_h_scan

    def compute_auto_scale(
        self,
        u1_px: float,
        v1_px: float,
        u2_px: float,
        v2_px: float,
        delta_x_mm: float,
        delta_y_mm: float,
    ) -> Tuple[float, float]:
        # Use rotation-invariant Euclidean pixel shift so camera housing tilt (cam_skew_deg)
        # does not inflate Kx/Ky by 1/cos(skew).
        disp_px = math.hypot(u2_px - u1_px, v2_px - v1_px)
        step_mm = math.hypot(delta_x_mm, delta_y_mm)
        if disp_px < 5.0:
            raise ValueError(
                f"Fiducial pixel shift too small for calibration (disp={disp_px:.1f} px)."
            )
        if step_mm > 0.0:
            scale = step_mm / disp_px
            self.kx_mm_per_px = scale
            self.ky_mm_per_px = scale
        self.calibrated = True
        return self.kx_mm_per_px, self.ky_mm_per_px

    def calibrate_origin(
        self,
        marker_id: int,
        u_px: float,
        v_px: float,
        motor_x_mm: float,
        motor_y_mm: float,
        fid_angle_deg: Optional[float] = None,
    ) -> Tuple[float, float]:
        """
        Single-marker fallback: calibrates camera plate origin from one detected fiducial
        and sets camera housing skew from the marker's orientation angle
        (nominal is 180.0 deg for marker top-edge pointing in world +X direction).
        """
        if marker_id not in self.corner_fiducials:
            raise ValueError(f"Unknown ArUco marker ID {marker_id} for origin calibration.")

        if fid_angle_deg is not None:
            self.cam_skew_deg = round(self._wrap_deg(fid_angle_deg - 180.0), 2)

        fx, fy = self.corner_fiducials[marker_id]

        # Rotate camera sensor displacement by camera skew angle
        # Inverted image X axis: u > cx is to physical -X direction
        # Image Y axis: v > cy is to physical +Y (forward) direction
        u_prime = -(u_px - self.cx_px) * self.kx_mm_per_px
        v_prime = (v_px - self.cy_px) * self.ky_mm_per_px
        dx_mm, dy_mm = self._rotate_2d(u_prime, v_prime, self.cam_skew_deg)

        self.cam_origin_x_mm = round(fx - (motor_x_mm + dx_mm), 2)
        self.cam_origin_y_mm = round(fy - (motor_y_mm + dy_mm), 2)
        self.cam_motor_skew_deg = self.cam_skew_deg
        self.plate_yaw_deg = 0.0
        self.plate_scale_x = 1.0
        self.plate_scale_y = 1.0
        self._plate_A = None
        self._plate_t = None
        self._plate_A_inv = None
        self._plate_t_inv = None
        self.calibration_mode = "single_marker"
        self.markers_detected = [int(marker_id)]
        self.residual_rms_mm = 0.0
        self.residual_max_mm = 0.0
        self.corner_residuals_mm = {int(marker_id): 0.0}
        self.origin_calibrated = True
        self.drift_x_mm = 0.0
        self.drift_y_mm = 0.0
        return self.cam_origin_x_mm, self.cam_origin_y_mm

    def calibrate_from_fiducials(
        self,
        observations: List[Tuple[int, float, float, float, float, Optional[float]]],
        step_observation: Optional[Tuple[float, float, float, float, float, float, float, float]] = None,
    ) -> dict:
        """
        Multi-corner ArUco Ground-Truth Calibration (supports 1..4 corner markers).
        Each observation is (marker_id, u_px, v_px, motor_x_mm, motor_y_mm, fid_angle_deg).
        Optional step_observation is (u1, v1, mx1, my1, u2, v2, mx2, my2) from a known motor step
        to decouple camera housing twist (cam_motor_skew_deg) from course plate yaw (plate_yaw_deg).

        Solves the global Motor-Frame Camera <-> Plate transform:
          P = A @ C + t
        where C = (motor_x + dx_motor, motor_y + dy_motor) and P is the known ground-truth
        corner coordinate in corner_fiducials.
        """
        best_by_id: Dict[int, Tuple[int, float, float, float, float, Optional[float], float]] = {}
        for mid, u_px, v_px, mx, my, ang in observations:
            m_id = int(mid)
            if m_id not in self.corner_fiducials:
                continue
            dist2 = (u_px - self.cx_px) ** 2 + (v_px - self.cy_px) ** 2
            if m_id not in best_by_id or dist2 < best_by_id[m_id][6]:
                best_by_id[m_id] = (m_id, float(u_px), float(v_px), float(mx), float(my), ang, dist2)

        if not best_by_id:
            raise ValueError("No valid corner fiducial observations supplied.")

        ordered_ids = sorted(best_by_id.keys())
        if len(ordered_ids) == 1:
            m_id, u_px, v_px, mx, my, ang, _ = best_by_id[ordered_ids[0]]
            ox, oy = self.calibrate_origin(m_id, u_px, v_px, mx, my, ang)
            return {
                "mode": self.calibration_mode,
                "markers_detected": list(self.markers_detected),
                "cam_origin_x_mm": ox,
                "cam_origin_y_mm": oy,
                "cam_skew_deg": self.cam_skew_deg,
                "cam_motor_skew_deg": self.cam_motor_skew_deg,
                "plate_yaw_deg": self.plate_yaw_deg,
                "scale_x": self.plate_scale_x,
                "scale_y": self.plate_scale_y,
                "residual_rms_mm": 0.0,
                "residual_max_mm": 0.0,
                "corner_residuals_mm": dict(self.corner_residuals_mm),
            }

        # Circular mean of camera-to-plate skew across all detected markers
        skew_samples = [
            self._wrap_deg(best_by_id[mid][5] - 180.0)
            for mid in ordered_ids
            if best_by_id[mid][5] is not None
        ]
        if skew_samples:
            sin_sum = sum(math.sin(math.radians(a)) for a in skew_samples)
            cos_sum = sum(math.cos(math.radians(a)) for a in skew_samples)
            cam_in_plate_deg = self._wrap_deg(math.degrees(math.atan2(sin_sum, cos_sum)))
        else:
            cam_in_plate_deg = self.cam_skew_deg

        cam_in_motor_deg = cam_in_plate_deg
        if step_observation is not None:
            u1, v1, mx1, my1, u2, v2, mx2, my2 = step_observation
            if math.hypot(mx2 - mx1, my2 - my1) >= 5.0 and math.hypot(u2 - u1, v2 - v1) >= 5.0:
                du_p = -(u2 - u1) * self.kx_mm_per_px
                dv_p = (v2 - v1) * self.ky_mm_per_px
                motor_ang = math.degrees(math.atan2(my2 - my1, mx2 - mx1))
                sensor_ang = math.degrees(math.atan2(-dv_p, -du_p))
                cam_in_motor_deg = self._wrap_deg(motor_ang - sensor_ang)

        P_arr = np.array([self.corner_fiducials[mid] for mid in ordered_ids], dtype=float)
        A = np.eye(2, dtype=float)
        t = np.zeros(2, dtype=float)
        mode = "multi_corner_similarity"

        for _ in range(3):
            C_list = []
            for mid in ordered_ids:
                _m, u_px, v_px, mx, my, _ang, _d2 = best_by_id[mid]
                u_prime = -(u_px - self.cx_px) * self.kx_mm_per_px
                v_prime = (v_px - self.cy_px) * self.ky_mm_per_px
                dx_m, dy_m = self._rotate_2d(u_prime, v_prime, cam_in_motor_deg)
                C_list.append([mx + dx_m, my + dy_m])
            C_arr = np.array(C_list, dtype=float)

            if len(ordered_ids) >= 3:
                cand_A, cand_t = self._fit_affine_2d(C_arr, P_arr)
                if self._is_valid_plate_affine(cand_A):
                    A, t = cand_A, cand_t
                    mode = f"{len(ordered_ids)}_corner_affine"
                else:
                    A, t = self._fit_similarity_2d(C_arr, P_arr)
                    mode = f"{len(ordered_ids)}_corner_similarity"
            else:
                A, t = self._fit_similarity_2d(C_arr, P_arr)
                mode = "2_corner_similarity"

            # P = A @ C + t maps Motor -> Plate, so plate rotation wrt motor is -atan2(A[1,0], A[0,0])
            plate_yaw_deg = -math.degrees(math.atan2(float(A[1, 0]), float(A[0, 0])))
            cam_in_motor_deg = self._wrap_deg(plate_yaw_deg + cam_in_plate_deg)

        # Finalize residuals with converged cam_in_motor_deg
        C_final = []
        for mid in ordered_ids:
            _m, u_px, v_px, mx, my, _ang, _d2 = best_by_id[mid]
            u_prime = -(u_px - self.cx_px) * self.kx_mm_per_px
            v_prime = (v_px - self.cy_px) * self.ky_mm_per_px
            dx_m, dy_m = self._rotate_2d(u_prime, v_prime, cam_in_motor_deg)
            C_final.append([mx + dx_m, my + dy_m])
        C_arr = np.array(C_final, dtype=float)

        if len(ordered_ids) >= 3 and "affine" in mode:
            cand_A, cand_t = self._fit_affine_2d(C_arr, P_arr)
            if self._is_valid_plate_affine(cand_A):
                A, t = cand_A, cand_t
            else:
                A, t = self._fit_similarity_2d(C_arr, P_arr)
                mode = f"{len(ordered_ids)}_corner_similarity"
        else:
            A, t = self._fit_similarity_2d(C_arr, P_arr)

        pred = (C_arr @ A.T) + t
        errs = np.linalg.norm(pred - P_arr, axis=1)
        residuals = {mid: round(float(err), 3) for mid, err in zip(ordered_ids, errs)}
        rms_mm = round(float(math.sqrt(float(np.mean(errs ** 2)))), 3)
        max_mm = round(float(np.max(errs)), 3)

        A_inv = np.linalg.inv(A)
        t_inv = -A_inv @ t

        self._plate_A = A
        self._plate_t = t
        self._plate_A_inv = A_inv
        self._plate_t_inv = t_inv

        self.cam_origin_x_mm = round(float(t[0]), 2)
        self.cam_origin_y_mm = round(float(t[1]), 2)
        self.cam_skew_deg = round(float(cam_in_plate_deg), 2)
        self.cam_motor_skew_deg = round(float(cam_in_motor_deg), 2)
        self.plate_yaw_deg = round(-math.degrees(math.atan2(float(A[1, 0]), float(A[0, 0]))), 3)
        self.plate_scale_x = round(float(math.hypot(float(A[0, 0]), float(A[1, 0]))), 4)
        self.plate_scale_y = round(float(math.hypot(float(A[0, 1]), float(A[1, 1]))), 4)
        self.calibration_mode = mode
        self.markers_detected = ordered_ids
        self.residual_rms_mm = rms_mm
        self.residual_max_mm = max_mm
        self.corner_residuals_mm = residuals
        self.calibrated = True
        self.origin_calibrated = True
        self.drift_x_mm = 0.0
        self.drift_y_mm = 0.0

        return {
            "mode": self.calibration_mode,
            "markers_detected": list(self.markers_detected),
            "cam_origin_x_mm": self.cam_origin_x_mm,
            "cam_origin_y_mm": self.cam_origin_y_mm,
            "cam_skew_deg": self.cam_skew_deg,
            "cam_motor_skew_deg": self.cam_motor_skew_deg,
            "plate_yaw_deg": self.plate_yaw_deg,
            "scale_x": self.plate_scale_x,
            "scale_y": self.plate_scale_y,
            "residual_rms_mm": self.residual_rms_mm,
            "residual_max_mm": self.residual_max_mm,
            "corner_residuals_mm": dict(self.corner_residuals_mm),
        }

    def plate_transform_dict(self) -> Optional[dict]:
        """Serializable affine transform for UI digital twin rendering."""
        if self._plate_A is None or self._plate_t is None:
            return None
        return {
            "a00": round(float(self._plate_A[0, 0]), 6),
            "a01": round(float(self._plate_A[0, 1]), 6),
            "tx": round(float(self._plate_t[0]), 3),
            "a10": round(float(self._plate_A[1, 0]), 6),
            "a11": round(float(self._plate_A[1, 1]), 6),
            "ty": round(float(self._plate_t[1]), 3),
        }

    def pixel_to_world_mm(
        self,
        u_px: float,
        v_px: float,
        motor_x_mm: float,
        motor_y_mm: float,
        z_current_mm: float = 0.0,
        z_scan_mm: float = 0.0,
        target_height_above_surface_mm: float = 0.0,
    ) -> Tuple[float, float]:
        """
        Converts pixel coordinates (u, v) captured at motor position (motor_x, motor_y)
        into true Plate Coordinates (mm wrt fiducial plate origin 0,0).
        Uses the 4-corner calibrated affine transform when available, otherwise falls back
        to single-marker origin + skew rotation.
        """
        height_ratio = self._height_ratio(
            z_current_mm, z_scan_mm, target_height_above_surface_mm
        )
        kx = self.kx_mm_per_px * height_ratio
        ky = self.ky_mm_per_px * height_ratio

        # Inverted image X axis: u > cx is to physical -X direction
        # Image Y axis: v > cy is to physical +Y (forward) direction
        u_prime = -(u_px - self.cx_px) * kx
        v_prime = (v_px - self.cy_px) * ky

        if self._plate_A is not None and self._plate_t is not None:
            dx_m, dy_m = self._rotate_2d(u_prime, v_prime, self.cam_motor_skew_deg)
            cx_m = motor_x_mm + dx_m
            cy_m = motor_y_mm + dy_m
            plate_x = float(self._plate_A[0, 0]) * cx_m + float(self._plate_A[0, 1]) * cy_m + float(self._plate_t[0])
            plate_y = float(self._plate_A[1, 0]) * cx_m + float(self._plate_A[1, 1]) * cy_m + float(self._plate_t[1])
            return plate_x, plate_y

        dx_mm, dy_mm = self._rotate_2d(u_prime, v_prime, self.cam_skew_deg)
        plate_x = self.cam_origin_x_mm + motor_x_mm + dx_mm
        plate_y = self.cam_origin_y_mm + motor_y_mm + dy_mm
        return plate_x, plate_y

    def world_to_pixel(
        self,
        plate_x_mm: float,
        plate_y_mm: float,
        motor_x_mm: float,
        motor_y_mm: float,
        z_current_mm: float = 0.0,
        z_scan_mm: float = 0.0,
        target_height_above_surface_mm: float = 0.0,
    ) -> Tuple[float, float]:
        """
        Projects Plate Coordinates (mm) back to pixel coordinates (u, v)
        for a given motor carriage position, inverting the plate transform and camera skew.
        """
        height_ratio = self._height_ratio(
            z_current_mm, z_scan_mm, target_height_above_surface_mm
        )
        kx = self.kx_mm_per_px * height_ratio
        ky = self.ky_mm_per_px * height_ratio

        if self._plate_A_inv is not None and self._plate_t_inv is not None:
            cx_m = float(self._plate_A_inv[0, 0]) * plate_x_mm + float(self._plate_A_inv[0, 1]) * plate_y_mm + float(self._plate_t_inv[0])
            cy_m = float(self._plate_A_inv[1, 0]) * plate_x_mm + float(self._plate_A_inv[1, 1]) * plate_y_mm + float(self._plate_t_inv[1])
            dx_m = cx_m - motor_x_mm
            dy_m = cy_m - motor_y_mm
            u_prime, v_prime = self._rotate_2d(dx_m, dy_m, -self.cam_motor_skew_deg)
        else:
            cam_x = self.cam_origin_x_mm + motor_x_mm
            cam_y = self.cam_origin_y_mm + motor_y_mm
            dx_mm = plate_x_mm - cam_x
            dy_mm = plate_y_mm - cam_y
            u_prime, v_prime = self._rotate_2d(dx_mm, dy_mm, -self.cam_skew_deg)

        u_px = self.cx_px - u_prime / kx
        v_px = self.cy_px + v_prime / ky
        return u_px, v_px

    def compensate_block_angle(self, raw_angle_deg: float) -> float:
        """
        Compensates raw vision detection angle for camera-to-plate skew,
        mapping to physical plate coordinate orientation (0-180 deg).
        """
        return round((raw_angle_deg - self.cam_skew_deg) % 180.0, 1)

    def servo_angle_for_plate_angle(self, plate_angle_deg: float) -> float:
        """
        Maps a desired plate-frame grasp/place orientation (deg) into the gantry's
        motor-frame wrist servo angle [0..90 deg) for symmetric square blocks.
        """
        return round((plate_angle_deg + self.plate_yaw_deg) % 90.0, 1)

    def plate_to_camera_motor(self, plate_x_mm: float, plate_y_mm: float) -> Tuple[float, float]:
        """Calculates motor coordinates to position camera optical center directly over a plate location."""
        if self._plate_A_inv is not None and self._plate_t_inv is not None:
            return (
                float(self._plate_A_inv[0, 0]) * plate_x_mm + float(self._plate_A_inv[0, 1]) * plate_y_mm + float(self._plate_t_inv[0]),
                float(self._plate_A_inv[1, 0]) * plate_x_mm + float(self._plate_A_inv[1, 1]) * plate_y_mm + float(self._plate_t_inv[1]),
            )
        return (
            plate_x_mm - self.cam_origin_x_mm,
            plate_y_mm - self.cam_origin_y_mm,
        )

    def plate_to_gripper_motor(self, plate_x_mm: float, plate_y_mm: float) -> Tuple[float, float]:
        """
        Calculates motor coordinates to position gripper jaws directly over a plate location.
        Positions camera over the plate location via the calibrated plate transform, then
        subtracts the rigid motor-frame camera-to-gripper offset.
        """
        cam_mx, cam_my = self.plate_to_camera_motor(plate_x_mm, plate_y_mm)
        return (
            cam_mx - self.offset_x_mm,
            cam_my - self.offset_y_mm,
        )

    def gripper_target_for_block(self, block_plate_x: float, block_plate_y: float) -> Tuple[float, float]:
        """Convenience alias for plate_to_gripper_motor."""
        return self.plate_to_gripper_motor(block_plate_x, block_plate_y)

    def record_fine_alignment_drift(
        self,
        coarse_x_mm: float,
        coarse_y_mm: float,
        refined_x_mm: float,
        refined_y_mm: float,
    ) -> Tuple[float, float]:
        """
        Tracks systematic motor-to-plate drift observed during Phase 5A fine camera alignment
        so subsequent Phase 6 stack placements stay centered on the same physical stack.
        """
        dx = float(refined_x_mm - coarse_x_mm)
        dy = float(refined_y_mm - coarse_y_mm)
        if math.hypot(dx, dy) <= 10.0:
            if abs(dy) > 1.75:
                self.drift_y_mm = max(-10.0, min(10.0, 0.6 * self.drift_y_mm + 0.4 * dy))
            if abs(dx) > 1.75:
                self.drift_x_mm = max(-10.0, min(10.0, 0.6 * self.drift_x_mm + 0.4 * dx))
        return self.drift_x_mm, self.drift_y_mm

    def plate_to_drop_gripper_motor(
        self, drop_plate_x_mm: float, drop_plate_y_mm: float
    ) -> Tuple[float, float]:
        """
        Converts Phase 6 target stack plate coordinates to gripper motor coordinates,
        compensating for any systematic motor drift observed during fine pick alignments.
        """
        return self.plate_to_gripper_motor(
            drop_plate_x_mm + self.drift_x_mm,
            drop_plate_y_mm + self.drift_y_mm,
        )


class WorldCatalogAndPlanner:
    """Maintains the deduplicated virtual block catalog and computes pick/stack plans."""

    def __init__(self, dedup_radius_mm: float = 18.0, block_height_mm: float = 20.0):
        self.dedup_radius_mm = dedup_radius_mm
        self.block_height_mm = block_height_mm
        self.blocks: List[BlockRecord] = []
        self._next_id = 1
        self.stack_counts: Dict[str, int] = {}

    def clear(self) -> None:
        self.blocks.clear()
        self._next_id = 1
        self.stack_counts.clear()

    def merge_detection(self, color: str, x_mm: float, y_mm: float, angle_deg: float) -> BlockRecord:
        for b in self.blocks:
            if b.status == "STACKED":
                continue
            dist = math.hypot(b.x_mm - x_mm, b.y_mm - y_mm)
            if dist <= self.dedup_radius_mm and b.color == color:
                n = b.confidence_views
                b.x_mm = (b.x_mm * n + x_mm) / (n + 1)
                b.y_mm = (b.y_mm * n + y_mm) / (n + 1)
                # Angle wrap-around handling for 90-degree symmetric square blocks
                # Prevents averaging 28 deg and 118 deg (same cube) into a 73 deg diagonal error
                angle_diff = (angle_deg - b.angle_deg + 45.0) % 90.0 - 45.0
                b.angle_deg = (b.angle_deg + angle_diff / (n + 1)) % 180.0
                b.confidence_views = n + 1
                return b

        new_block = BlockRecord(
            id=self._next_id,
            color=color,
            x_mm=x_mm,
            y_mm=y_mm,
            angle_deg=angle_deg % 180.0,
        )
        self._next_id += 1
        self.blocks.append(new_block)
        return new_block

    def compute_pick_sequence(
        self,
        current_x_mm: float,
        current_y_mm: float,
        stack_zones_mm: Optional[Dict[str, Tuple[float, float, float]]] = None,
        zone_clear_radius_mm: float = 38.0,
        priority_block_id: Optional[int] = None,
        deferred_block_ids: Optional[Set[int]] = None,
    ) -> List[BlockRecord]:
        """
        Orders detected/queued blocks:
        1) First clear any block sitting inside/on top of a laydown area so later
           placements never stack onto an obstructing block (preferring a requested
           priority blocker first, then blockers whose nominal color zone is free).
        2) Then pick remaining blocks sequentially by nearest distance (respecting
           neighbor clearance and pushing recently staged blocks after blockers).
        """
        candidates = [b for b in self.blocks if b.status in ("DETECTED", "QUEUED")]
        if not candidates:
            return []

        zones = stack_zones_mm or {}
        deferred = deferred_block_ids or set()

        def obstructing_zone(target: BlockRecord) -> Optional[str]:
            for zk, (zx, zy, _zz) in zones.items():
                if math.hypot(target.x_mm - zx, target.y_mm - zy) <= zone_clear_radius_mm:
                    return zk
            return None

        def min_neighbor_clearance(target: BlockRecord, pool: List[BlockRecord]) -> float:
            dists = [
                math.hypot(target.x_mm - other.x_mm, target.y_mm - other.y_mm)
                for other in pool
                if other.id != target.id
            ]
            return min(dists) if dists else 999.0

        ordered: List[BlockRecord] = []
        remaining = list(candidates)
        cx, cy = current_x_mm, current_y_mm

        while remaining:
            pool = list(remaining)

            def pick_key(b: BlockRecord) -> Tuple[int, int, int, int, int, float, int]:
                obs = obstructing_zone(b)
                is_blocker = 0 if obs is not None else 1
                is_priority = 0 if (priority_block_id is not None and b.id == priority_block_id and not ordered) else 1
                is_deferred = 1 if b.id in deferred else 0
                # If clearing an obstacle, prefer blocks whose nominal color zone is not itself blocked
                dest_blocked = 0
                if obs is not None and b.color in zones:
                    dzx, dzy, _dzz = zones[b.color]
                    if any(
                        other.id != b.id
                        and math.hypot(other.x_mm - dzx, other.y_mm - dzy) <= zone_clear_radius_mm
                        for other in pool
                    ):
                        dest_blocked = 1
                clearance_tier = 0 if min_neighbor_clearance(b, pool) >= 35.0 else 1
                return (is_blocker, is_priority, is_deferred, dest_blocked, clearance_tier, math.hypot(b.x_mm - cx, b.y_mm - cy), b.id)

            remaining = sorted(pool, key=pick_key)
            nxt = remaining.pop(0)
            nxt.status = "QUEUED"
            ordered.append(nxt)
            cx, cy = nxt.x_mm, nxt.y_mm

        return ordered

    def find_zone_obstruction(
        self,
        zone_x_mm: float,
        zone_y_mm: float,
        exclude_block_id: int,
        radius_mm: float = 38.0,
    ) -> Optional[BlockRecord]:
        """Returns the closest unstacked block sitting within `radius_mm` of a laydown zone."""
        blockers = [
            b
            for b in self.blocks
            if b.id != exclude_block_id
            and b.status in ("DETECTED", "QUEUED", "PICKING", "WEIGHED")
            and math.hypot(b.x_mm - zone_x_mm, b.y_mm - zone_y_mm) <= radius_mm
        ]
        if not blockers:
            return None
        return min(
            blockers,
            key=lambda b: math.hypot(b.x_mm - zone_x_mm, b.y_mm - zone_y_mm),
        )

    def find_staging_spot(
        self,
        ref_x_mm: float,
        ref_y_mm: float,
        stack_zones_mm: Dict[str, Tuple[float, float, float]],
        exclude_block_id: Optional[int] = None,
        kinematics: Optional[EyeInHandKinematics] = None,
        travel_x_mm: float = 227.0,
        travel_y_mm: float = 128.0,
        min_safe_y_mm: float = 10.0,
        aluminum_platform_size_mm: float = 76.2,
    ) -> Optional[Tuple[float, float]]:
        """Finds a clear intermediate staging spot reachable by both camera and gripper,
        strictly outside every laydown zone's 38.0 mm obstruction radius so staged blocks
        never re-trigger zone blocking, and accounting for the 76.2 x 76.2 mm raised aluminum platform."""
        zone_obstacles: List[Tuple[float, float]] = [
            (zx, zy) for zx, zy, _zz in stack_zones_mm.values()
        ]
        block_obstacles: List[Tuple[float, float]] = [
            (b.x_mm, b.y_mm)
            for b in self.blocks
            if exclude_block_id is None or b.id != exclude_block_id
        ]

        alum_zone = stack_zones_mm.get("aluminum")
        alum_half = aluminum_platform_size_mm / 2.0

        # Tiers: (cam_y_min, min_zone_clear, min_block_clear, allow_on_raised_aluminum_platform)
        # Every tier keeps staged blocks > 38.0 mm outside all laydown zones.
        tiers = [
            (min_safe_y_mm, 44.0, 48.0, False),
            (min_safe_y_mm, 42.0, 40.0, False),
            (0.0, 42.0, 40.0, False),
            (0.0, 42.0, 32.0, False),
            (0.0, 39.5, 29.5, False),
            (0.0, 39.5, 29.5, True),
        ]

        xs = [round(42.0 + 8.0 * i, 1) for i in range(26)]   # 42 .. 242 mm
        ys = [round(36.0 + 8.0 * j, 1) for j in range(12)]   # 36 .. 124 mm

        for cam_y_min, req_zone_clear, req_block_clear, allow_on_platform in tiers:
            best_score: Optional[float] = None
            best_pt: Optional[Tuple[float, float]] = None

            for px in xs:
                for py in ys:
                    if kinematics is not None:
                        cam_mx, cam_my = kinematics.plate_to_camera_motor(px, py)
                        grip_mx, grip_my = kinematics.plate_to_gripper_motor(px, py)
                        if not (4.0 <= cam_mx <= travel_x_mm - 4.0):
                            continue
                        if not (cam_y_min <= cam_my <= travel_y_mm - 4.0):
                            continue
                        if not (4.0 <= grip_mx <= travel_x_mm - 4.0):
                            continue
                        if not (min_safe_y_mm <= grip_my <= travel_y_mm - 4.0):
                            continue
                    else:
                        if not (42.0 <= px <= 235.0 and max(60.0 if cam_y_min > 0 else 42.0, py) <= 122.0):
                            continue

                    if alum_zone is not None and alum_half > 0.0:
                        ax, ay, _az = alum_zone
                        plat_dx = max(ax - alum_half - px, 0.0, px - (ax + alum_half))
                        plat_dy = max(ay - alum_half - py, 0.0, py - (ay + alum_half))
                        dist_to_plat = math.hypot(plat_dx, plat_dy)
                        inset_in_plat = min(
                            px - (ax - alum_half),
                            (ax + alum_half) - px,
                            py - (ay - alum_half),
                            (ay + alum_half) - py,
                        )
                        if allow_on_platform:
                            # Either safely off the platform or cleanly seated inside it (never straddling the step)
                            if dist_to_plat < 24.0 and inset_in_plat < 15.0:
                                continue
                        else:
                            if dist_to_plat < 24.0:
                                continue

                    min_zone = (
                        min(math.hypot(px - ox, py - oy) for ox, oy in zone_obstacles)
                        if zone_obstacles
                        else 999.0
                    )
                    if min_zone < req_zone_clear:
                        continue

                    min_blk = (
                        min(math.hypot(px - ox, py - oy) for ox, oy in block_obstacles)
                        if block_obstacles
                        else 999.0
                    )
                    if min_blk < req_block_clear:
                        continue

                    min_obs = min(min_zone, min_blk)
                    dist_ref = math.hypot(px - ref_x_mm, py - ref_y_mm)
                    score = min(min_obs, 70.0) - 0.15 * dist_ref
                    if best_score is None or score > best_score:
                        best_score = score
                        best_pt = (px, py)

            if best_pt is not None:
                return best_pt

        return None

    def max_stack_top_z_up_mm(
        self, stack_zones_mm: Dict[str, Tuple[float, float, float]]
    ) -> float:
        """Returns the highest stacked block top Z (mm above machine base plate)."""
        max_top = 0.0
        for zk, count in self.stack_counts.items():
            if count > 0 and zk in stack_zones_mm:
                _zx, _zy, zone_z_up = stack_zones_mm[zk]
                max_top = max(max_top, zone_z_up + count * self.block_height_mm)
        return max_top

    def get_stack_target(
        self,
        color: str,
        mass_class: str,
        stack_zones_mm: Dict[str, Tuple[float, float, float]],
        base_plate_z_down_mm: float,
        commit: bool = True,
    ) -> Tuple[float, float, float, str]:
        """
        Phase 6 Sort & Stack:
        Routes aluminum cubes (~21.6 g) to the aluminum pedestal and non-aluminum cubes
        by color (red, green, blue) to their respective laydown zones.
        Zone Z coordinates include floor offsets from machine base plate (base_plate_z_down_mm = 76.71 mm);
        Z_down = base_plate_z_down_mm - zone_z_up_mm, reduced by 20 mm per stacked cube.
        """
        if mass_class == "aluminum" and "aluminum" in stack_zones_mm:
            zone_key = "aluminum"
        elif color in stack_zones_mm:
            zone_key = color
        else:
            zone_key = next(iter(stack_zones_mm))

        drop_x, drop_y, drop_z_up = stack_zones_mm[zone_key]
        count = self.stack_counts.get(zone_key, 0)
        drop_z_down = max(0.0, (base_plate_z_down_mm - drop_z_up) - count * self.block_height_mm)
        if commit:
            self.stack_counts[zone_key] = count + 1
        return drop_x, drop_y, drop_z_down, zone_key

    def commit_stack_placement(self, zone_key: str) -> None:
        self.stack_counts[zone_key] = self.stack_counts.get(zone_key, 0) + 1
