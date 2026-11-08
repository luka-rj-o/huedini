"""Eye-in-Hand coordinate transformations, block deduplication, and path planning."""

from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Tuple


@dataclass
class BlockRecord:
    """Cataloged 20x20 mm block in the virtual environment model."""
    id: int
    color: str
    x_mm: float
    y_mm: float
    angle_deg: float
    mass_g: float = 0.0
    mass_class: str = "unknown"  # aluminum, high_infill, med_infill, low_infill, reject
    status: str = "DETECTED"     # DETECTED, QUEUED, PICKING, WEIGHED, STACKED, REJECTED
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
      - Motor travel coordinates (motor_x in [0..236], motor_y in [0..128], motor_z_down in [0..70.36])
      - Fiducial Plate coordinates (plate_x in [0..284], plate_y in [0..167.4], plate_z above bed)
      - Camera pixel coordinates (u_px, v_px)
    """

    def __init__(
        self,
        frame_width: int = 640,
        frame_height: int = 480,
        kx_mm_per_px: float = 0.25,
        ky_mm_per_px: float = 0.25,
        offset_x_mm: float = 0.2,
        offset_y_mm: float = -54.1,
        cam_homed_z_above_plate_mm: float = 135.4,
        cam_origin_x_mm: float = 27.8,
        cam_origin_y_mm: float = 60.1,
        corner_fiducials: Optional[Dict[int, Tuple[float, float]]] = None,
    ):
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.cx_px = frame_width / 2.0
        self.cy_px = frame_height / 2.0
        self.kx_mm_per_px = kx_mm_per_px
        self.ky_mm_per_px = ky_mm_per_px
        self.offset_x_mm = offset_x_mm
        self.offset_y_mm = offset_y_mm
        self.cam_homed_z_above_plate_mm = cam_homed_z_above_plate_mm
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

    def _height_ratio(self, z_down_mm: float, z_scan_down_mm: float = 0.0) -> float:
        """
        Since Z=0 is at the TOP when homed (camera 135.4 mm above plate) and +Z moves DOWNWARD,
        camera height above plate = (135.4 - z_down_mm).
        """
        cam_h_now = max(25.0, self.cam_homed_z_above_plate_mm - z_down_mm)
        cam_h_scan = max(25.0, self.cam_homed_z_above_plate_mm - z_scan_down_mm)
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
        du = abs(u2_px - u1_px)
        dv = abs(v2_px - v1_px)
        if du < 5.0 and dv < 5.0:
            raise ValueError(
                f"Fiducial pixel shift too small for calibration (du={du:.1f}, dv={dv:.1f})."
            )
        if du >= 5.0 and abs(delta_x_mm) > 0.0:
            self.kx_mm_per_px = abs(delta_x_mm) / du
        if dv >= 5.0 and abs(delta_y_mm) > 0.0:
            self.ky_mm_per_px = abs(delta_y_mm) / dv
        elif du >= 5.0 and abs(delta_x_mm) > 0.0:
            self.ky_mm_per_px = self.kx_mm_per_px
        self.calibrated = True
        return self.kx_mm_per_px, self.ky_mm_per_px

    def calibrate_origin(
        self,
        marker_id: int,
        u_px: float,
        v_px: float,
        motor_x_mm: float,
        motor_y_mm: float,
    ) -> Tuple[float, float]:
        """
        Calibrates the true camera plate origin using the detected fiducial marker.
        Marker position in Plate Coordinates = (fx, fy).
        Camera optical center in Plate Coordinates = (X_cam_origin + motor_x, Y_cam_origin + motor_y).
        Marker relative to camera:
          dx = -(u_px - cx_px) * kx
          dy = -(v_px - cy_px) * ky
        Therefore:
          fx = X_cam_origin + motor_x + dx  =>  X_cam_origin = fx - (motor_x + dx)
          fy = Y_cam_origin + motor_y + dy  =>  Y_cam_origin = fy - (motor_y + dy)
        """
        if marker_id not in self.corner_fiducials:
            raise ValueError(f"Unknown ArUco marker ID {marker_id} for origin calibration.")

        fx, fy = self.corner_fiducials[marker_id]
        dx_mm = -(u_px - self.cx_px) * self.kx_mm_per_px
        dy_mm = -(v_px - self.cy_px) * self.ky_mm_per_px

        self.cam_origin_x_mm = round(fx - (motor_x_mm + dx_mm), 2)
        self.cam_origin_y_mm = round(fy - (motor_y_mm + dy_mm), 2)
        self.origin_calibrated = True
        return self.cam_origin_x_mm, self.cam_origin_y_mm

    def pixel_to_world_mm(
        self,
        u_px: float,
        v_px: float,
        motor_x_mm: float,
        motor_y_mm: float,
        z_current_mm: float = 0.0,
        z_scan_mm: float = 0.0,
    ) -> Tuple[float, float]:
        """
        Converts pixel coordinates (u, v) captured at motor position (motor_x, motor_y)
        into true Plate Coordinates (mm wrt fiducial plate origin 0,0).
        """
        height_ratio = self._height_ratio(z_current_mm, z_scan_mm)
        kx = self.kx_mm_per_px * height_ratio
        ky = self.ky_mm_per_px * height_ratio

        # Inverted image X axis: u > cx is to the physical -X direction
        dx_mm = -(u_px - self.cx_px) * kx
        dy_mm = -(v_px - self.cy_px) * ky

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
    ) -> Tuple[float, float]:
        """
        Projects Plate Coordinates (mm) back to pixel coordinates (u, v)
        for a given motor carriage position.
        """
        height_ratio = self._height_ratio(z_current_mm, z_scan_mm)
        kx = self.kx_mm_per_px * height_ratio
        ky = self.ky_mm_per_px * height_ratio

        cam_x = self.cam_origin_x_mm + motor_x_mm
        cam_y = self.cam_origin_y_mm + motor_y_mm

        dx_mm = plate_x_mm - cam_x
        dy_mm = plate_y_mm - cam_y

        u_px = self.cx_px - dx_mm / kx
        v_px = self.cy_px - dy_mm / ky
        return u_px, v_px

    def plate_to_camera_motor(self, plate_x_mm: float, plate_y_mm: float) -> Tuple[float, float]:
        """Calculates motor coordinates to position camera optical center directly over a plate location."""
        return (
            plate_x_mm - self.cam_origin_x_mm,
            plate_y_mm - self.cam_origin_y_mm,
        )

    def plate_to_gripper_motor(self, plate_x_mm: float, plate_y_mm: float) -> Tuple[float, float]:
        """
        Calculates motor coordinates to position gripper jaws directly over a plate location.
        Since gripper is offset from camera by (offset_x_mm, offset_y_mm),
        motor = plate - cam_origin - offset.
        """
        return (
            plate_x_mm - self.cam_origin_x_mm - self.offset_x_mm,
            plate_y_mm - self.cam_origin_y_mm - self.offset_y_mm,
        )

    def gripper_target_for_block(self, block_plate_x: float, block_plate_y: float) -> Tuple[float, float]:
        """Convenience alias for plate_to_gripper_motor."""
        return self.plate_to_gripper_motor(block_plate_x, block_plate_y)


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
            if b.status in ("STACKED", "REJECTED"):
                continue
            dist = math.hypot(b.x_mm - x_mm, b.y_mm - y_mm)
            if dist <= self.dedup_radius_mm and b.color == color:
                n = b.confidence_views
                b.x_mm = (b.x_mm * n + x_mm) / (n + 1)
                b.y_mm = (b.y_mm * n + y_mm) / (n + 1)
                # Angle wrap-around handling for 180-degree periodic orientation
                angle_diff = (angle_deg - b.angle_deg + 90.0) % 180.0 - 90.0
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

    def compute_pick_sequence(self, current_x_mm: float, current_y_mm: float) -> List[BlockRecord]:
        candidates = [b for b in self.blocks if b.status in ("DETECTED", "QUEUED")]
        if not candidates:
            return []

        color_priority = {"red": 0, "green": 1, "blue": 2, "yellow": 3}

        def min_neighbor_clearance(target: BlockRecord) -> float:
            dists = [
                math.hypot(target.x_mm - other.x_mm, target.y_mm - other.y_mm)
                for other in candidates
                if other.id != target.id
            ]
            return min(dists) if dists else 999.0

        ordered: List[BlockRecord] = []
        remaining = list(candidates)
        cx, cy = current_x_mm, current_y_mm

        while remaining:
            remaining.sort(
                key=lambda b: (
                    0 if min_neighbor_clearance(b) >= 40.0 else 1,
                    color_priority.get(b.color, 99),
                    math.hypot(b.x_mm - cx, b.y_mm - cy),
                )
            )
            nxt = remaining.pop(0)
            nxt.status = "QUEUED"
            ordered.append(nxt)
            cx, cy = nxt.x_mm, nxt.y_mm

        return ordered

    def get_stack_target(
        self,
        color: str,
        mass_class: str,
        stack_zones_mm: Dict[str, Tuple[float, float]],
        base_pick_z_down_mm: float,
    ) -> Tuple[float, float, float, bool]:
        """
        Phase 6 Sort & Stack:
        Note that Z=0 is at the TOP when homed and +Z moves DOWNWARD (76.71 mm to floor).
        Each stacked 20 mm block reduces the required downward Z travel by 20 mm.
        """
        is_rejected = mass_class == "reject"
        zone_key = "reject" if is_rejected else (mass_class if mass_class in stack_zones_mm else color)
        if zone_key not in stack_zones_mm:
            zone_key = "reject"

        drop_x, drop_y = stack_zones_mm[zone_key]
        count = self.stack_counts.get(zone_key, 0)
        drop_z_down = max(0.0, base_pick_z_down_mm - count * self.block_height_mm)
        self.stack_counts[zone_key] = count + 1
        return drop_x, drop_y, drop_z_down, is_rejected
