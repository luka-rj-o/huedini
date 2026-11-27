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
    z_mm: Optional[float] = None

    def to_dict(self) -> dict:
        data = {
            "id": self.id,
            "color": self.color,
            "x_mm": round(self.x_mm, 2),
            "y_mm": round(self.y_mm, 2),
            "angle_deg": round(self.angle_deg, 1),
            "mass_g": round(self.mass_g, 2),
            "mass_class": self.mass_class,
            "status": self.status,
        }
        if self.z_mm is not None:
            data["z_mm"] = round(self.z_mm, 2)
        return data


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
        cam_tilt_x_mm_per_mm_z: float = 0.0,
        cam_tilt_y_mm_per_mm_z: float = 0.0,
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
        self.cam_tilt_x_mm_per_mm_z = cam_tilt_x_mm_per_mm_z
        self.cam_tilt_y_mm_per_mm_z = cam_tilt_y_mm_per_mm_z
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
    def _fit_similarity_2d(
        src_pts: np.ndarray,
        dst_pts: np.ndarray,
        min_scale: Optional[float] = None,
        max_scale: Optional[float] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
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
        scale = math.hypot(a, b)
        if scale > 1e-6 and (min_scale is not None or max_scale is not None):
            clamped = scale
            if min_scale is not None:
                clamped = max(min_scale, clamped)
            if max_scale is not None:
                clamped = min(max_scale, clamped)
            factor = clamped / scale
            a *= factor
            b *= factor
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
        if not (0.992 <= sx <= 1.008 and 0.992 <= sy <= 1.008 and abs(sx - sy) <= 0.008):
            return False
        # Verify axes remain nearly orthogonal (within +/- 2 deg of 90 deg)
        dot = float(A[0, 0] * A[0, 1] + A[1, 0] * A[1, 1])
        cos_ortho = abs(dot) / max(1e-6, sx * sy)
        return cos_ortho <= math.sin(math.radians(2.0))

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

    def _tilt_dz_closer_mm(
        self,
        z_current_mm: float,
        z_scan_mm: float = 0.0,
        target_height_above_surface_mm: float = 0.0,
    ) -> float:
        """
        Distance (mm) the camera optical axis is closer to the observed plane than during
        the Z_scan=0 ArUco fiducial calibration (includes floor elevation when elevated).
        """
        dz_elev = self.course_elevation_mm if target_height_above_surface_mm > 0.0 else 0.0
        return (z_current_mm - z_scan_mm) + target_height_above_surface_mm + dz_elev

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
        has_step_skew = False
        if step_observation is not None:
            u1, v1, mx1, my1, u2, v2, mx2, my2 = step_observation
            if math.hypot(mx2 - mx1, my2 - my1) >= 5.0 and math.hypot(u2 - u1, v2 - v1) >= 5.0:
                du_p = -(u2 - u1) * self.kx_mm_per_px
                dv_p = (v2 - v1) * self.ky_mm_per_px
                motor_ang = math.degrees(math.atan2(my2 - my1, mx2 - mx1))
                sensor_ang = math.degrees(math.atan2(-dv_p, -du_p))
                cam_in_motor_deg = self._wrap_deg(motor_ang - sensor_ang)
                has_step_skew = True

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
                    A, t = self._fit_similarity_2d(C_arr, P_arr, min_scale=0.995, max_scale=1.005)
                    mode = f"{len(ordered_ids)}_corner_similarity"
            else:
                A, t = self._fit_similarity_2d(C_arr, P_arr, min_scale=0.995, max_scale=1.005)
                mode = "2_corner_similarity"

            # P = A @ C + t maps Motor -> Plate, so plate rotation wrt motor is -atan2(A[1,0], A[0,0])
            plate_yaw_deg = -math.degrees(math.atan2(float(A[1, 0]), float(A[0, 0])))
            if has_step_skew:
                break
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
                A, t = self._fit_similarity_2d(C_arr, P_arr, min_scale=0.995, max_scale=1.005)
                mode = f"{len(ordered_ids)}_corner_similarity"
        else:
            A, t = self._fit_similarity_2d(C_arr, P_arr, min_scale=0.995, max_scale=1.005)

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

        final_plate_yaw_deg = -math.degrees(math.atan2(float(A[1, 0]), float(A[0, 0])))
        self.cam_origin_x_mm = round(float(t[0]), 2)
        self.cam_origin_y_mm = round(float(t[1]), 2)
        self.cam_skew_deg = round(
            float(self._wrap_deg(cam_in_motor_deg - final_plate_yaw_deg) if has_step_skew else cam_in_plate_deg),
            2,
        )
        self.cam_motor_skew_deg = round(float(cam_in_motor_deg), 2)
        self.plate_yaw_deg = round(final_plate_yaw_deg, 3)
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
        dz_closer = self._tilt_dz_closer_mm(
            z_current_mm, z_scan_mm, target_height_above_surface_mm
        )
        tilt_dx_mm = self.cam_tilt_x_mm_per_mm_z * dz_closer
        tilt_dy_mm = self.cam_tilt_y_mm_per_mm_z * dz_closer

        if self._plate_A is not None and self._plate_t is not None:
            dx_m, dy_m = self._rotate_2d(u_prime, v_prime, self.cam_motor_skew_deg)
            dx_m -= tilt_dx_mm
            dy_m -= tilt_dy_mm
            cx_m = motor_x_mm + dx_m
            cy_m = motor_y_mm + dy_m
            plate_x = float(self._plate_A[0, 0]) * cx_m + float(self._plate_A[0, 1]) * cy_m + float(self._plate_t[0])
            plate_y = float(self._plate_A[1, 0]) * cx_m + float(self._plate_A[1, 1]) * cy_m + float(self._plate_t[1])
            return plate_x, plate_y

        dx_mm, dy_mm = self._rotate_2d(u_prime, v_prime, self.cam_skew_deg)
        dx_mm -= tilt_dx_mm
        dy_mm -= tilt_dy_mm
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
        dz_closer = self._tilt_dz_closer_mm(
            z_current_mm, z_scan_mm, target_height_above_surface_mm
        )
        tilt_dx_mm = self.cam_tilt_x_mm_per_mm_z * dz_closer
        tilt_dy_mm = self.cam_tilt_y_mm_per_mm_z * dz_closer

        if self._plate_A_inv is not None and self._plate_t_inv is not None:
            cx_m = float(self._plate_A_inv[0, 0]) * plate_x_mm + float(self._plate_A_inv[0, 1]) * plate_y_mm + float(self._plate_t_inv[0])
            cy_m = float(self._plate_A_inv[1, 0]) * plate_x_mm + float(self._plate_A_inv[1, 1]) * plate_y_mm + float(self._plate_t_inv[1])
            dx_m = cx_m - motor_x_mm + tilt_dx_mm
            dy_m = cy_m - motor_y_mm + tilt_dy_mm
            u_prime, v_prime = self._rotate_2d(dx_m, dy_m, -self.cam_motor_skew_deg)
        else:
            cam_x = self.cam_origin_x_mm + motor_x_mm
            cam_y = self.cam_origin_y_mm + motor_y_mm
            dx_mm = plate_x_mm - cam_x + tilt_dx_mm
            dy_mm = plate_y_mm - cam_y + tilt_dy_mm
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
        eff_yaw = self.plate_yaw_deg if abs(self.plate_yaw_deg) >= 0.5 else 0.0
        angle = round((plate_angle_deg + eff_yaw) % 90.0, 1)
        if angle >= 89.0:
            angle = 0.0
        return angle

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

    def gripper_motor_to_plate(self, motor_x_mm: float, motor_y_mm: float) -> Tuple[float, float]:
        """Inverse of plate_to_gripper_motor: returns current EE plate (X, Y) from motor (X, Y)."""
        cam_mx = float(motor_x_mm) + self.offset_x_mm
        cam_my = float(motor_y_mm) + self.offset_y_mm
        if self._plate_A is not None and self._plate_t is not None:
            px = (
                float(self._plate_A[0, 0]) * cam_mx
                + float(self._plate_A[0, 1]) * cam_my
                + float(self._plate_t[0])
            )
            py = (
                float(self._plate_A[1, 0]) * cam_mx
                + float(self._plate_A[1, 1]) * cam_my
                + float(self._plate_t[1])
            )
            return px, py
        return (
            self.cam_origin_x_mm + cam_mx,
            self.cam_origin_y_mm + cam_my,
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
        Converts Phase 6 target stack plate coordinates to gripper motor coordinates.
        Uses exact calibrated plate coordinates so every layer of a stack aligns on the
        same motor (X, Y) target without perspective-parallax drift.
        """
        return self.plate_to_gripper_motor(drop_plate_x_mm, drop_plate_y_mm)


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

    # Exact Onshape V3 Gantry moving-stage CAD clearance parameters (mm)
    _CAD_PRESS_OVERTRAVEL_MM: float = 3.5
    _CAD_CLEARANCE_MARGIN_MM: float = 5.0

    @staticmethod
    def _surface_z_up_at(
        x_mm: float,
        y_mm: float,
        stack_zones_mm: Optional[Dict[str, Tuple[float, float, float]]],
        course_elevation_mm: float = 6.35,
        aluminum_platform_size_mm: float = 76.2,
    ) -> float:
        """Returns resting surface Z_up (15.9 on raised aluminum pedestal, else 6.35)."""
        if stack_zones_mm and "aluminum" in stack_zones_mm and aluminum_platform_size_mm > 0.0:
            ax, ay, az = stack_zones_mm["aluminum"]
            half = aluminum_platform_size_mm / 2.0
            if abs(x_mm - ax) <= half and abs(y_mm - ay) <= half:
                return az
        return course_elevation_mm

    @staticmethod
    def _min_cad_z0_over_zone(
        ee_x_mm: float,
        ee_y_mm: float,
        zone_x_mm: float,
        zone_y_mm: float,
    ) -> Optional[float]:
        """
        Returns the lowest CAD stage Z_up (at motor Z_down=0) overlapping the 20x20 mm
        footprint around a zone stack (zone_x +/- 10, zone_y +/- 10), or None if disjoint.
        Measured from v3_gantry_compiled.bin (cad_motor_z_mm = 4.9).
        """
        bx0, bx1 = zone_x_mm - 10.0, zone_x_mm + 10.0
        by0, by1 = zone_y_mm - 10.0, zone_y_mm + 10.0
        z0: Optional[float] = None

        def _upd(val: float) -> None:
            nonlocal z0
            z0 = val if z0 is None or val < z0 else z0

        # 1. zStage main Y-boom & linear rail over playable plate (plateY in [10.0, 146.0])
        if min(ee_x_mm - 38.0, bx1) > max(ee_x_mm - 62.0, bx0) and min(146.0, by1) > max(10.0, by0):
            _upd(118.9)
        # 2. zStage front boom end-cap (plateY in [146.0, 171.5])
        if min(ee_x_mm - 41.0, bx1) > max(ee_x_mm - 64.0, bx0) and min(171.5, by1) > max(146.0, by0):
            _upd(114.9)
        # 3. yStage boom clamp & low carriage plate (dY in [-18.0, +22.0])
        if min(ee_x_mm - 15.0, bx1) > max(ee_x_mm - 57.0, bx0) and min(ee_y_mm + 22.0, by1) > max(ee_y_mm - 18.0, by0):
            _upd(111.4)
        # 4. yStage upper camera arm & guide (dY in [+22.0, +68.0])
        if min(ee_x_mm + 13.0, bx1) > max(ee_x_mm - 24.0, bx0) and min(ee_y_mm + 68.0, by1) > max(ee_y_mm + 22.0, by0):
            _upd(125.8)
        # 5. eeStage radial body guard (prevents picking blocks tightly adjacent to existing stack)
        if math.hypot(ee_x_mm - zone_x_mm, ee_y_mm - zone_y_mm) < 33.0:
            _upd(92.7)

        return z0

    def max_safe_stack_count_for_pose(
        self,
        ee_x_mm: float,
        ee_y_mm: float,
        z_down_mm: float,
        zone_x_mm: float,
        zone_y_mm: float,
        zone_z_up_mm: float,
        clearance_margin_mm: float = _CAD_CLEARANCE_MARGIN_MM,
    ) -> int:
        """Maximum cube count allowed on `zone` without colliding with an action at (ee_x, ee_y, z_down)."""
        z0 = self._min_cad_z0_over_zone(ee_x_mm, ee_y_mm, zone_x_mm, zone_y_mm)
        if z0 is None:
            return 99
        avail_mm = z0 - (z_down_mm + self._CAD_PRESS_OVERTRAVEL_MM) - clearance_margin_mm - zone_z_up_mm
        return max(0, int(math.floor(avail_mm / self.block_height_mm)))

    def is_pose_boom_safe(
        self,
        ee_x_mm: float,
        ee_y_mm: float,
        z_down_mm: float,
        stack_zones_mm: Dict[str, Tuple[float, float, float]],
        stack_counts: Optional[Dict[str, int]] = None,
        ignore_zone: Optional[str] = None,
        clearance_margin_mm: float = _CAD_CLEARANCE_MARGIN_MM,
    ) -> bool:
        """Checks whether an EE action at (ee_x, ee_y, z_down) clears all existing zone stacks."""
        counts = self.stack_counts if stack_counts is None else stack_counts
        for zk, cnt in counts.items():
            if cnt <= 0 or zk == ignore_zone or zk not in stack_zones_mm:
                continue
            zx, zy, zz = stack_zones_mm[zk]
            max_cnt = self.max_safe_stack_count_for_pose(
                ee_x_mm,
                ee_y_mm,
                z_down_mm,
                zx,
                zy,
                zz,
                clearance_margin_mm=clearance_margin_mm,
            )
            if cnt > max_cnt:
                return False
        return True

    @staticmethod
    def _predict_transit_z_down(
        counts: Tuple[int, ...],
        z_coords: List[Tuple[float, float, float]],
        block_height_mm: float,
        base_plate_z_down_mm: float,
        z_hover_mm: float,
    ) -> float:
        max_top = 0.0
        for zi, c in enumerate(counts):
            if c > 0:
                top = z_coords[zi][2] + c * block_height_mm
                if top > max_top:
                    max_top = top
        if max_top <= 0.0:
            return z_hover_mm
        return max(2.0, min(z_hover_mm, base_plate_z_down_mm - (max_top + 8.0)))

    def compute_pick_sequence(
        self,
        current_x_mm: float,
        current_y_mm: float,
        stack_zones_mm: Optional[Dict[str, Tuple[float, float, float]]] = None,
        zone_clear_radius_mm: float = 26.0,
        priority_block_id: Optional[int] = None,
        deferred_block_ids: Optional[Set[int]] = None,
        boom_x_min_offset_mm: float = -64.0,
        boom_x_max_offset_mm: float = -33.0,
        base_plate_z_down_mm: float = 76.71,
        course_elevation_mm: float = 6.35,
        aluminum_platform_size_mm: float = 76.2,
        z_hover_mm: float = 25.0,
        clearance_margin_mm: float = _CAD_CLEARANCE_MARGIN_MM,
        beam_width: int = 24,
    ) -> List[BlockRecord]:
        """
        Precomputes a collision-free, minimum-cost 3D (XY + Z) pick-and-stack sequence
        using a lightweight 2.5D CAD twin beam search:
        1) Evaluates exact Onshape moving-stage envelopes (zStage Y-boom/rail, yStage
           clamp/arm/camera, eeStage radial guard) on every pick AND every drop layer.
        2) Minimizes total tour cost (XY travel + weighted Z stroke) starting from the
           gantry's current pose and spanning outward smoothly across all colors,
           pulling zone blockers cleanly forward when they unlock destinations.
        3) Prunes dead-end stack states where raising a left-side stack would block a
           remaining pick or drop layer, falling back to minimum-staging transitions
           only when no zero-staging sequence exists.
        """
        del boom_x_min_offset_mm, boom_x_max_offset_mm  # Superseded by exact CAD envelope model
        candidates = [b for b in self.blocks if b.status in ("DETECTED", "QUEUED")]
        if not candidates:
            return []

        zones = stack_zones_mm or {}
        deferred = deferred_block_ids or set()
        if not zones:
            remaining = list(candidates)
            ordered: List[BlockRecord] = []
            cx, cy = current_x_mm, current_y_mm
            while remaining:
                remaining.sort(
                    key=lambda b: (
                        0 if (priority_block_id is not None and b.id == priority_block_id and not ordered) else 1,
                        1 if b.id in deferred else 0,
                        math.hypot(b.x_mm - cx, b.y_mm - cy),
                        b.id,
                    )
                )
                nxt = remaining.pop(0)
                nxt.status = "QUEUED"
                ordered.append(nxt)
                cx, cy = nxt.x_mm, nxt.y_mm
            return ordered

        n = len(candidates)
        zone_keys = tuple(zones.keys())
        nz = len(zone_keys)
        zk_idx = {k: i for i, k in enumerate(zone_keys)}
        z_coords = [zones[k] for k in zone_keys]
        init_counts = tuple(self.stack_counts.get(k, 0) for k in zone_keys)
        max_layers = [init_counts[zi] + n for zi in range(nz)]

        default_non_rgb = "aluminum" if "aluminum" in zk_idx else zone_keys[0]
        b_xy = [(b.x_mm, b.y_mm) for b in candidates]
        b_dest = [zk_idx[b.color if b.color in zk_idx else default_non_rgb] for b in candidates]
        b_pz = [
            max(
                0.0,
                base_plate_z_down_mm
                - self._surface_z_up_at(
                    x,
                    y,
                    zones,
                    course_elevation_mm=course_elevation_mm,
                    aluminum_platform_size_mm=aluminum_platform_size_mm,
                ),
            )
            for x, y in b_xy
        ]
        b_drop_dist = [
            math.hypot(z_coords[b_dest[i]][0] - b_xy[i][0], z_coords[b_dest[i]][1] - b_xy[i][1])
            for i in range(n)
        ]
        b_laydown_dist = [
            min(math.hypot(x - zx, y - zy) for zx, zy, _zz in z_coords)
            for x, y in b_xy
        ]

        # Obstruction bitmasks & close-neighbor bitmasks
        obs_mask = [0] * nz
        is_any_blocker = [False] * n
        for i, (x, y) in enumerate(b_xy):
            for zi, (zx, zy, _zz) in enumerate(z_coords):
                if math.hypot(x - zx, y - zy) <= zone_clear_radius_mm:
                    obs_mask[zi] |= 1 << i
                    is_any_blocker[i] = True

        close_nb = [0] * n
        for i in range(n):
            for j in range(i + 1, n):
                if math.hypot(b_xy[i][0] - b_xy[j][0], b_xy[i][1] - b_xy[j][1]) < 34.0:
                    close_nb[i] |= 1 << j
                    close_nb[j] |= 1 << i

        # Precompute integer CAD safety tables
        pick_max = [
            tuple(
                self.max_safe_stack_count_for_pose(
                    b_xy[i][0],
                    b_xy[i][1],
                    b_pz[i],
                    z_coords[zi][0],
                    z_coords[zi][1],
                    z_coords[zi][2],
                    clearance_margin_mm=clearance_margin_mm,
                )
                for zi in range(nz)
            )
            for i in range(n)
        ]
        drop_z_table = [
            [
                max(0.0, (base_plate_z_down_mm - z_coords[zi][2]) - layer * self.block_height_mm)
                for layer in range(max_layers[zi] + 1)
            ]
            for zi in range(nz)
        ]
        drop_max = [
            [
                tuple(
                    99
                    if ozi == zi
                    else self.max_safe_stack_count_for_pose(
                        z_coords[zi][0],
                        z_coords[zi][1],
                        drop_z_table[zi][layer],
                        z_coords[ozi][0],
                        z_coords[ozi][1],
                        z_coords[ozi][2],
                        clearance_margin_mm=clearance_margin_mm,
                    )
                    for ozi in range(nz)
                )
                for layer in range(max_layers[zi] + 1)
            ]
            for zi in range(nz)
        ]

        is_boom_shadowed = [any(pick_max[i][zi] < 3 for zi in range(nz)) for i in range(n)]

        full_mask = (1 << n) - 1

        # Hard step-0 enforcement for priority_block_id / deferred_block_ids:
        # 1) Never pick a deferred (just-staged) block on Step 0 when non-deferred safe candidates exist.
        # 2) Trace transitive zone blockers from priority_block_id so if the priority blocker's own
        #    destination is blocked by an unblocked block, Step 0 clears the unblocked blocker first
        #    instead of forcing a chain-staging move.
        safe_step0 = [
            i for i in range(n) if all(init_counts[zi] <= pick_max[i][zi] for zi in range(nz))
        ]
        non_def_safe = [i for i in safe_step0 if candidates[i].id not in deferred]
        step0_pool = non_def_safe if non_def_safe else safe_step0

        def _is_unblocked_at_step0(idx: int) -> bool:
            di0 = b_dest[idx]
            if obs_mask[di0] & (full_mask ^ (1 << idx)):
                return False
            dm0 = drop_max[di0][init_counts[di0]]
            return all(init_counts[zi] <= dm0[zi] for zi in range(nz))

        step0_allowed: Optional[Set[int]] = None
        priority_bonus_ids: Set[int] = set()
        if priority_block_id is not None:
            id_to_idx = {candidates[i].id: i for i in range(n)}
            if priority_block_id in id_to_idx:
                prio_chain: Set[int] = set()
                frontier = [id_to_idx[priority_block_id]]
                while frontier:
                    cur_i = frontier.pop()
                    if cur_i in prio_chain:
                        continue
                    prio_chain.add(cur_i)
                    mask = obs_mask[b_dest[cur_i]] & (full_mask ^ (1 << cur_i))
                    for j in range(n):
                        if (mask & (1 << j)) and j not in prio_chain:
                            frontier.append(j)
                unblocked_chain = [
                    i for i in step0_pool if i in prio_chain and _is_unblocked_at_step0(i)
                ]
                if unblocked_chain:
                    step0_allowed = set(unblocked_chain)
                    priority_bonus_ids = {candidates[i].id for i in unblocked_chain}
        if step0_allowed is None and non_def_safe and deferred:
            step0_allowed = set(non_def_safe)

        # State tuple: (cost, last_x, last_y, counts_tuple, rem_mask, path_tuple)
        beam = [(0.0, float(current_x_mm), float(current_y_mm), init_counts, full_mask, ())]

        for step in range(n):
            next_states: Dict[Tuple[int, int, Tuple[int, ...]], Tuple[float, float, float, Tuple[int, ...], int, Tuple[int, ...]]] = {}
            outward_mult = 0.06 * (n - step)
            for cost, cx, cy, counts, rem_mask, path in beam:
                tz = self._predict_transit_z_down(
                    counts, z_coords, self.block_height_mm, base_plate_z_down_mm, z_hover_mm
                )
                for i in range(n):
                    if step == 0 and step0_allowed is not None and i not in step0_allowed:
                        continue
                    bit = 1 << i
                    if not (rem_mask & bit):
                        continue

                    pick_safe = True
                    for zi in range(nz):
                        if counts[zi] > pick_max[i][zi]:
                            pick_safe = False
                            break
                    if not pick_safe:
                        continue

                    di = b_dest[i]
                    other_rem = rem_mask ^ bit
                    dest_blocked = bool(obs_mask[di] & other_rem)

                    can_stack_now = not dest_blocked
                    layer = counts[di]
                    if can_stack_now:
                        dm = drop_max[di][layer]
                        for zi in range(nz):
                            if counts[zi] > dm[zi]:
                                can_stack_now = False
                                break
                    if can_stack_now:
                        nc = list(counts)
                        new_c = layer + 1
                        nc[di] = new_c
                        candidate_counts = tuple(nc)
                        needed_zones = 0
                        for j in range(n):
                            if other_rem & (1 << j):
                                if new_c > pick_max[j][di]:
                                    can_stack_now = False
                                    break
                                needed_zones |= 1 << b_dest[j]
                        if can_stack_now:
                            for zi in range(nz):
                                if (needed_zones & (1 << zi)) and zi != di:
                                    if new_c > drop_max[zi][candidate_counts[zi]][di]:
                                        can_stack_now = False
                                        break
                    else:
                        candidate_counts = counts

                    if can_stack_now:
                        next_counts = candidate_counts
                        drop_z = drop_z_table[di][layer]
                        end_x, end_y = z_coords[di][0], z_coords[di][1]
                        step_pen = 0.0
                        dedup_zi = di
                    else:
                        # Staging fallback: useful stage (blocker or boom-shadowed) vs futile no-op stage
                        next_counts = counts
                        drop_z = b_pz[i]
                        end_x, end_y = b_xy[i]
                        step_pen = 5000.0 if (is_any_blocker[i] or is_boom_shadowed[i]) else 50000.0
                        dedup_zi = -1

                    next_tz = self._predict_transit_z_down(
                        next_counts, z_coords, self.block_height_mm, base_plate_z_down_mm, z_hover_mm
                    )
                    d_xy = math.hypot(b_xy[i][0] - cx, b_xy[i][1] - cy) + math.hypot(
                        end_x - b_xy[i][0], end_y - b_xy[i][1]
                    )
                    d_z = 2.0 * abs(b_pz[i] - tz) + abs(drop_z - tz) + abs(drop_z - next_tz)
                    nb_count = bin(close_nb[i] & other_rem).count("1")
                    nb_pen = 22.0 * nb_count
                    blk_bonus = -18.0 if is_any_blocker[i] else 0.0
                    out_pen = outward_mult * b_laydown_dist[i]
                    prio_bonus = (
                        -3000.0
                        if (step == 0 and candidates[i].id in priority_bonus_ids)
                        else 0.0
                    )
                    def_pen = 800.0 / (step + 1) if candidates[i].id in deferred else 0.0

                    new_cost = (
                        cost
                        + d_xy
                        + 1.8 * d_z
                        + step_pen
                        + nb_pen
                        + blk_bonus
                        + out_pen
                        + prio_bonus
                        + def_pen
                    )
                    key = (other_rem, dedup_zi, next_counts)
                    prev = next_states.get(key)
                    if prev is None or new_cost < prev[0]:
                        next_states[key] = (
                            new_cost,
                            end_x,
                            end_y,
                            next_counts,
                            other_rem,
                            path + (i,),
                        )

            if not next_states:
                break
            beam = sorted(next_states.values(), key=lambda s: s[0])[:beam_width]

        chosen_indices: List[int] = []
        if beam:
            best = min(beam, key=lambda s: s[0] + math.hypot(51.7 - s[1], 36.4 - s[2]))
            chosen_indices = list(best[5])

        ordered: List[BlockRecord] = []
        used = set(chosen_indices)
        for idx in chosen_indices:
            blk = candidates[idx]
            blk.status = "QUEUED"
            ordered.append(blk)

        # Defensive tail-fill if any caller invokes from an impossible pre-stacked state
        if len(ordered) < n:
            tail = [candidates[i] for i in range(n) if i not in used]
            cx = ordered[-1].x_mm if ordered else current_x_mm
            cy = ordered[-1].y_mm if ordered else current_y_mm
            while tail:
                tail.sort(
                    key=lambda b: (
                        0 if (priority_block_id is not None and b.id == priority_block_id and not ordered) else 1,
                        1 if b.id in deferred else 0,
                        math.hypot(b.x_mm - cx, b.y_mm - cy),
                        b.id,
                    )
                )
                nxt = tail.pop(0)
                nxt.status = "QUEUED"
                ordered.append(nxt)
                cx, cy = nxt.x_mm, nxt.y_mm

        return ordered

    def would_stack_create_boom_hazard(
        self,
        zone_key: str,
        stack_zones_mm: Dict[str, Tuple[float, float, float]],
        exclude_block_id: Optional[int] = None,
        boom_x_min_offset_mm: float = -64.0,
        boom_x_max_offset_mm: float = -33.0,
        base_plate_z_down_mm: float = 76.71,
        course_elevation_mm: float = 6.35,
        aluminum_platform_size_mm: float = 76.2,
        clearance_margin_mm: float = _CAD_CLEARANCE_MARGIN_MM,
    ) -> Optional[BlockRecord]:
        """
        Defense-in-depth CAD boom check: if committing onto `zone_key` would cause any
        remaining unstacked block's pick OR its next nominal stack-layer drop to collide
        with the gantry's moving CAD structures (zStage boom/rail, yStage clamp/arm/cam,
        eeStage body), returns the highest-priority conflicting unstacked block, else None.
        """
        del boom_x_min_offset_mm, boom_x_max_offset_mm
        if zone_key not in stack_zones_mm:
            return None

        hypothetical_counts = dict(self.stack_counts)
        hypothetical_counts[zone_key] = hypothetical_counts.get(zone_key, 0) + 1

        sorted_zones = sorted(stack_zones_mm.items(), key=lambda kv: -kv[1][0])
        zone_rank: Dict[str, int] = {zk: idx for idx, (zk, _coords) in enumerate(sorted_zones)}
        default_non_rgb = "aluminum" if "aluminum" in stack_zones_mm else sorted_zones[0][0]

        hazards: List[Tuple[int, int, int, BlockRecord]] = []
        for other in self.blocks:
            if other.status not in ("DETECTED", "QUEUED") or other.id == exclude_block_id:
                continue

            pick_z = max(
                0.0,
                base_plate_z_down_mm
                - self._surface_z_up_at(
                    other.x_mm,
                    other.y_mm,
                    stack_zones_mm,
                    course_elevation_mm=course_elevation_mm,
                    aluminum_platform_size_mm=aluminum_platform_size_mm,
                ),
            )
            pick_bad = not self.is_pose_boom_safe(
                other.x_mm,
                other.y_mm,
                pick_z,
                stack_zones_mm,
                stack_counts=hypothetical_counts,
                clearance_margin_mm=clearance_margin_mm,
            )

            dz = other.color if other.color in stack_zones_mm else default_non_rgb
            nxt_layer = hypothetical_counts.get(dz, 0)
            dzx, dzy, dzz = stack_zones_mm[dz]
            nxt_drop_z = max(0.0, (base_plate_z_down_mm - dzz) - nxt_layer * self.block_height_mm)
            drop_bad = (dz != zone_key) and not self.is_pose_boom_safe(
                dzx,
                dzy,
                nxt_drop_z,
                stack_zones_mm,
                stack_counts=hypothetical_counts,
                ignore_zone=dz,
                clearance_margin_mm=clearance_margin_mm,
            )

            if pick_bad or drop_bad:
                hazards.append((0 if pick_bad else 1, zone_rank.get(dz, 0), other.id, other))

        if not hazards:
            return None
        return min(hazards, key=lambda item: (item[0], item[1], item[2]))[3]

    def find_zone_obstruction(
        self,
        zone_x_mm: float,
        zone_y_mm: float,
        exclude_block_id: int,
        radius_mm: float = 26.0,
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
                    surface_z = self._surface_z_up_at(
                        px,
                        py,
                        stack_zones_mm,
                        course_elevation_mm=6.35,
                        aluminum_platform_size_mm=aluminum_platform_size_mm,
                    )
                    pick_z_down = max(0.0, 76.71 - surface_z)
                    min_safe = min(
                        (
                            self.max_safe_stack_count_for_pose(
                                px, py, pick_z_down, zx, zy, zz
                            )
                            for zx, zy, zz in stack_zones_mm.values()
                        ),
                        default=99,
                    )
                    shadow_pen = 0.0
                    if min_safe < 2:
                        shadow_pen = 45.0
                    elif min_safe < 3:
                        shadow_pen = 20.0
                    score = min(min_obs, 70.0) - 0.15 * dist_ref - shadow_pen
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
