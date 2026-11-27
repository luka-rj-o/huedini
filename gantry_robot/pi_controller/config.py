"""Configuration constants for the HUE-DINI 3-Axis Cartesian Gantry Robot."""

from dataclasses import dataclass, field
from typing import Dict, List, Tuple


@dataclass
class HSVRange:
    """HSV color threshold definition (OpenCV H: 0-179, S: 0-255, V: 0-255)."""
    name: str
    lower1: Tuple[int, int, int]
    upper1: Tuple[int, int, int]
    lower2: Tuple[int, int, int] | None = None
    upper2: Tuple[int, int, int] | None = None
    display_hex: str = "#ffffff"


@dataclass
class MassClassSpec:
    """Mass classification bin for the 20x20 mm blocks."""
    name: str
    nominal_g: float
    min_g: float
    max_g: float


@dataclass
class GantryConfig:
    """Master configuration for kinematics, CAD datum offsets, camera, serial, and course geometry."""

    # --- Serial Communication (Pi Zero USB-A -> ESP32 USB-C) ---
    serial_port: str = "/dev/ttyUSB0"
    serial_baud: int = 115200
    serial_timeout_s: float = 35.0

    # --- Stepper Calibration ---
    # Lead screw pitch vs. lead:
    # Most standard T8 lead screws (T8x8, 4 starts) advance 8.0 mm per rev:
    # X uses 8x microstepping (1600 microsteps/rev -> 200.0 steps/mm on 8 mm lead screw) for quiet motion.
    # Z uses 2x microstepping (400 microsteps/rev -> 50.0 steps/mm on 8 mm lead screw).
    # Y axis: 20-tooth GT2 belt pulley (40.0 mm/rev), 200 steps/rev, 8x microstepping = 40.0 steps/mm.
    lead_screw_pitch_mm: float = 2.0
    lead_screw_lead_mm: float = 8.0
    y_belt_pulley_mm_rev: float = 40.0
    microsteps: int = 2
    steps_per_mm_x: float = 200.0
    steps_per_mm_y: float = 40.0
    steps_per_mm_z: float = 50.0

    # --- Physical Motor Travel Strokes from Homed (0, 0, 0) (mm) ---
    # Z=0 is at the TOP when homed; positive motor Z_down moves DOWNWARD up to 70.36 mm.
    # Full mechanical strokes: (227.0, 128.0, 76.7) mm.
    # Note: Course is elevated 1/4" (6.35 mm) above the machine base plate.
    # Bottom descent is preserved at 70.36 mm (76.71 - 6.35 = 70.36 mm).
    course_elevation_mm: float = 6.35
    travel_x_mm: float = 227.0
    travel_y_mm: float = 128.0
    travel_z_max_mm: float = 76.7
    travel_z_bottom_mm: float = 70.36

    # --- CAD Homed Datum & Component Positions in Course/Plate Coordinates (0, 0, 0) ---
    # 1. Tower datum origin wrt course origin: 28.5 mm right, 43.5 mm behind, 127.5 mm above
    homed_datum_plate_x_mm: float = 28.5
    homed_datum_plate_y_mm: float = -43.5
    homed_datum_plate_z_mm: float = 127.5

    # 2. Homed End-Effector top wrt course origin: 31.7 mm right, +6.4 mm fore, 97.6 mm above
    #    (Relative to tower datum: +3.2 mm X, +49.9 mm Y, -29.9 mm Z)
    ee_from_datum_x_mm: float = 3.2
    ee_from_datum_y_mm: float = 49.9
    ee_from_datum_z_mm: float = -29.9
    # -> In plate coordinates at motor (0,0,0): EE = (31.7, +6.4, 97.6) mm

    # 3. Homed Camera optical center: calibrated V3 Gantry nominal origin (33.2, 48.0, 135.4) mm
    #    (Relative to tower datum: +4.7 mm X, +91.5 mm Y, +7.9 mm Z)
    cam_from_datum_x_mm: float = 4.7
    cam_from_datum_y_mm: float = 91.5
    cam_from_datum_z_mm: float = 7.9
    nominal_cam_origin_x_mm: float = 33.2
    nominal_cam_origin_y_mm: float = 48.0
    cam_homed_z_above_plate_mm: float = 135.4

    # --- Course & Outer Fiducial Border Dimensions (mm) ---
    workspace_x_mm: float = 284.0
    workspace_y_mm: float = 167.4
    workspace_z_mm: float = 140.0

    # Inner playable course (254.0 x 152.4 mm) shares back edge Y = 0.0 with the fiducial plate
    course_length_x_mm: float = 254.0
    course_width_y_mm: float = 152.4
    course_origin_x_mm: float = 15.0  # (284.0 - 254.0) / 2 -> X in [15.0 .. 269.0]
    course_origin_y_mm: float = 0.0   # Coincident with fiducial back edge Y = 0.0 -> Y in [0.0 .. 152.4]

    # --- Motor Z Travel Coordinates (mm down from Z=0 top home) ---
    # With 1/4" (6.35 mm) floor elevation, pick descent is 70.36 mm down (76.71 - 6.35)
    z_scan_mm: float = 0.0         # Scan at top home height (Z_down = 0.0 mm -> Cam Z = 135.4 mm above plate)
    z_hover_mm: float = 25.0       # Safe travel hover height (Z_down = 25.0 mm)
    z_pick_mm: float = 70.36       # Full descent to pick block on elevated course floor (76.71 - 6.35 mm)
    # Motor Z_down corresponding to Z=0.0 machine base plate (course floor 6.35 mm -> 70.36 mm down)
    base_plate_z_down_mm: float = 76.71

    # --- Block Geometry & 4 Mass Classes (20 x 20 mm cubes) ---
    block_size_mm: float = 20.0
    block_height_mm: float = 20.0

    mass_classes: List[MassClassSpec] = field(
        default_factory=lambda: [
            MassClassSpec("low_infill", nominal_g=2.46, min_g=0.0, max_g=4.75),
            MassClassSpec("med_infill", nominal_g=7.05, min_g=4.75, max_g=8.40),
            MassClassSpec("high_infill", nominal_g=9.75, min_g=8.40, max_g=17.50),
            MassClassSpec("aluminum", nominal_g=21.6, min_g=17.50, max_g=60.0),
        ]
    )

    # --- Motion Feedrates (mm/s), Surface Press, Scan Settling, and EE Rotation Clearance ---
    feedrate_travel_mm_s: float = 100.0  # Firmware caps X at 60.0 mm/s, Y at 100.0 mm/s
    feedrate_z_mm_s: float = 35.0
    feedrate_fine_mm_s: float = 40.0
    feedrate_press_mm_s: float = 8.0     # Controlled downward press (<0.8 mm per HX711 sample)
    feedrate_pick_guard_mm_s: float = 26.0  # Continuous guarded PICK descent speed (no mid-air stop)
    feedrate_drop_guard_mm_s: float = 28.0  # Continuous guarded DROP descent speed (no mid-air stop)
    press_approach_offset_mm: float = 0.5  # Fast Z descend stops slightly above surface before PRESS
    pick_guard_offset_mm: float = 80.0     # Start HX711 force-guarded PRESS directly from current Z hover
    drop_guard_offset_mm: float = 80.0     # Start HX711 force-guarded PRESS directly from pre-drop Z hover
    fine_align_max_radius_mm: float = 18.0  # Max association radius for Phase 5A fine alignment (blocks hijacking)
    pick_press_max_mm: float = 2.5       # Max downward press past nominal pick surface
    drop_press_max_mm: float = 2.5       # Max downward press past nominal drop surface (firmly pins cube on release)
    press_force_g: float = 85.0          # Upward load-cell delta target during open-jaw PICK press
    pick_guard_force_g: float = 95.0     # Early-collision threshold during PICK guarded approach
    drop_guard_force_g: float = 95.0     # Early-collision threshold (above held mass) during DROP guarded approach
    drop_press_force_g: float = 120.0    # Upward load-cell delta target beyond cube weight before releasing on DROP
    early_force_z_margin_mm: float = 4.5  # Force trigger above (pick_z_down - margin) flags PICK misalignment
    drop_early_force_z_margin_mm: float = 9.0  # Real obstacles are ~20 mm tall; within 9.0 mm is normal drop seating
    max_pick_misalign_retries: int = 3   # Consecutive early force-spike retries on PICK (3 retries = 4 total attempts)
    max_drop_collision_retries: int = 3  # Consecutive early force-spike retries on DROP (3 retries = 4 total attempts)
    scan_settle_s: float = 0.45
    tare_settle_s: float = 0.25          # Mechanical + servo settling before HX711 TARE
    weigh_settle_s: float = 0.25         # Post-lift cantilever settling before HX711 WEIGH
    weigh_servo_angle_deg: float = 0.0   # Keep wrist clear of camera FOV and matched across TARE and WEIGH
    servo_safe_y_mm: float = 10.0
    servo_zero_offset_deg: float = 6.0   # Mechanical neutral trim calibrated in UI (matches servo_trim.json)

    # --- Xbox Series X|S Manual Teleoperation Parameters ---
    teleop_hz: float = 20.0
    teleop_max_vx_mm_s: float = 60.0
    teleop_max_vy_mm_s: float = 100.0
    teleop_max_vz_up_mm_s: float = 35.0
    teleop_max_vz_down_mm_s: float = 22.0
    teleop_max_va_deg_s: float = 92.0
    teleop_precision_scale: float = 0.25
    teleop_deadzone: float = 0.12
    teleop_curve_exp: float = 2.2
    teleop_z_guard_enabled: bool = True
    teleop_z_guard_force_g: float = 150.0
    teleop_telemetry_mode: str = "suspended"  # "suspended" | "low_rate"
    teleop_low_rate_hz: float = 5.0

    # --- Eye-in-Hand Camera & Offset Parameters ---
    camera_index: int = 0
    frame_width: int = 640
    frame_height: int = 480

    default_kx_mm_per_px: float = 0.25
    default_ky_mm_per_px: float = 0.25
    default_cam_skew_deg: float = 0.0

    # Camera is offset from gripper in CAD (calibrated for camera Z-tilt + laydown symmetry):
    # Measured Z-tilt (+0.0207 mm/mmZ X, +0.0387 mm/mmZ Y) + Z=25 staged-block repicks (-1.815 X, -0.770 Y),
    # empirical -2.0 mm X laydown trim (-4.38 -> -2.38 X), +1.0 mm +X trim (-2.38 -> -3.38 X),
    # plus final +1.0 mm +X EE trim (-3.38 -> -4.38 X).
    camera_to_gripper_offset_x_mm: float = -4.38
    camera_to_gripper_offset_y_mm: float = -44.36
    cam_tilt_x_mm_per_mm_z: float = 0.0207
    cam_tilt_y_mm_per_mm_z: float = 0.0387

    # --- Calibration Fiducials (15 x 15 mm squares at the 4 corners of the 284 x 167.4 mm outer border) ---
    aruco_marker_id: int = 1
    fiducial_size_mm: float = 15.0
    # Calibration Fiducial search point in motor travel coordinates (Marker #1 at (7.5, 159.9))
    fiducial_base_x_mm: float = 0.0
    fiducial_base_y_mm: float = 115.0
    corner_fiducials_mm: List[Tuple[float, float]] = field(
        default_factory=lambda: [
            (7.5, 7.5),       # Bottom-Left corner (center of 15x15 mm marker)
            (7.5, 159.9),     # Top-Left corner (Marker ID #1 in physical layout)
            (276.5, 159.9),   # Top-Right corner (Marker ID #2 in physical layout)
            (276.5, 7.5),     # Bottom-Right corner
        ]
    )
    corner_fiducials_by_id: Dict[int, Tuple[float, float]] = field(
        default_factory=lambda: {
            0: (7.5, 7.5),       # Marker ID 0: Bottom-Left
            1: (7.5, 159.9),     # Marker ID 1: Top-Left
            2: (276.5, 159.9),   # Marker ID 2: Top-Right
            3: (276.5, 7.5),     # Marker ID 3: Bottom-Right
        }
    )
    # Ordered 4-corner calibration waypoints (motor X, Y mm) keeping each 15x15 mm marker cleanly in FOV
    calibration_corner_waypoints_mm: Dict[int, Tuple[float, float]] = field(
        default_factory=lambda: {
            1: (0.0, 115.0),     # Top-Left (Marker #1)
            2: (215.0, 115.0),   # Top-Right (Marker #2)
            3: (215.0, 0.0),     # Bottom-Right (Marker #3)
            0: (0.0, 0.0),       # Bottom-Left (Marker #0)
        }
    )
    calibration_step_x_mm: float = 20.0
    calibration_step_y_mm: float = 0.0

    # --- Global Scan Grid Coordinates (in motor travel coordinates within 227 x 128 mm envelope) ---
    # Column-serpentine 6-point (3x2) grid reaches Y=0.0 cleanly for rear-row coverage and finishes
    # at (196.0, 65.0) over the laydown zones so shortest-path planning spans outward without deadhead.
    scan_grid_points_mm: List[Tuple[float, float]] = field(
        default_factory=lambda: [
            (20.0, 0.0),
            (20.0, 65.0),
            (108.0, 65.0),
            (108.0, 0.0),
            (196.0, 0.0),
            (196.0, 65.0),
        ]
    )

    # --- Sorting / Laydown Zones (Plate X, Plate Y, Z_up wrt course origin incl. floor offsets) ---
    stack_zones_mm: Dict[str, Tuple[float, float, float]] = field(
        default_factory=lambda: {
            "red": (250.2, 115.0, 6.35),
            "green": (210.2, 115.0, 6.35),
            "blue": (170.2, 115.0, 6.35),
            "aluminum": (224.6, 44.5, 18.5),
        }
    )
    # Aluminum pedestal is a 76.2 x 76.2 mm (3" x 3") raised square centered at stack_zones_mm["aluminum"][:2]
    aluminum_platform_size_mm: float = 76.2

    # --- Block Contour Filtering (pixels^2 at z_scan_mm and hover z_hover_mm) ---
    min_contour_area_px: float = 2200.0
    max_contour_area_px: float = 35000.0
    deduplication_radius_mm: float = 18.0

    # --- Classical Computer Vision HSV Color Ranges ---
    hsv_ranges: Dict[str, HSVRange] = field(
        default_factory=lambda: {
            "red": HSVRange(
                name="red",
                lower1=(0, 110, 80),
                upper1=(10, 255, 255),
                lower2=(168, 110, 80),
                upper2=(179, 255, 255),
                display_hex="#ef4444",
            ),
            "green": HSVRange(
                name="green",
                lower1=(38, 90, 70),
                upper1=(85, 255, 255),
                display_hex="#22c55e",
            ),
            "blue": HSVRange(
                name="blue",
                lower1=(98, 100, 70),
                upper1=(132, 255, 255),
                display_hex="#3b82f6",
            ),
            "yellow": HSVRange(
                name="yellow",
                lower1=(20, 110, 90),
                upper1=(34, 255, 255),
                display_hex="#eab308",
            ),
        }
    )

    @property
    def ee_homed_plate_xyz(self) -> Tuple[float, float, float]:
        """End-effector top position in plate coordinates when homed at motor (0, 0, 0)."""
        return (
            round(self.homed_datum_plate_x_mm + self.ee_from_datum_x_mm, 2),
            round(self.homed_datum_plate_y_mm + self.ee_from_datum_y_mm, 2),
            round(self.homed_datum_plate_z_mm + self.ee_from_datum_z_mm, 2),
        )

    @property
    def cam_homed_plate_xyz(self) -> Tuple[float, float, float]:
        """Camera position in plate coordinates when homed at motor (0, 0, 0)."""
        return (
            round(self.homed_datum_plate_x_mm + self.cam_from_datum_x_mm, 2),
            round(self.homed_datum_plate_y_mm + self.cam_from_datum_y_mm, 2),
            round(self.homed_datum_plate_z_mm + self.cam_from_datum_z_mm, 2),
        )

    def classify_mass(self, mass_g: float) -> str:
        """Classifies a block into one of the 4 valid mass classes (no reject bin)."""
        if mass_g <= self.mass_classes[0].max_g:
            return self.mass_classes[0].name
        for mc in self.mass_classes[1:-1]:
            if mass_g <= mc.max_g:
                return mc.name
        return self.mass_classes[-1].name

    def distance_to_aluminum_platform(self, plate_x_mm: float, plate_y_mm: float) -> float:
        """Euclidean distance (mm) from plate (x, y) to the 76.2 x 76.2 mm aluminum platform (0 if inside)."""
        alum = self.stack_zones_mm.get("aluminum")
        if alum is None or self.aluminum_platform_size_mm <= 0.0:
            return 999.0
        ax, ay, _az = alum
        half = self.aluminum_platform_size_mm / 2.0
        dx = max(ax - half - plate_x_mm, 0.0, plate_x_mm - (ax + half))
        dy = max(ay - half - plate_y_mm, 0.0, plate_y_mm - (ay + half))
        return (dx * dx + dy * dy) ** 0.5

    def surface_z_up_mm_at(self, plate_x_mm: float, plate_y_mm: float) -> float:
        """Returns local surface Z-up (mm wrt origin) at plate (x, y): 18.5 on the aluminum platform, else 6.35."""
        alum = self.stack_zones_mm.get("aluminum")
        if alum is not None and self.distance_to_aluminum_platform(plate_x_mm, plate_y_mm) <= 1e-6:
            return alum[2]
        return self.course_elevation_mm

    def surface_pick_z_down_mm(self, plate_x_mm: float, plate_y_mm: float) -> float:
        """Returns motor Z_down (mm) to grasp/place a single cube resting at plate (x, y)."""
        return round(
            max(0.0, self.base_plate_z_down_mm - self.surface_z_up_mm_at(plate_x_mm, plate_y_mm)),
            2,
        )

    def block_top_height_mm_at(self, plate_x_mm: float, plate_y_mm: float) -> float:
        """Returns block top-face height above nominal course surface for perspective scaling."""
        surface_offset = max(
            0.0, self.surface_z_up_mm_at(plate_x_mm, plate_y_mm) - self.course_elevation_mm
        )
        return self.block_height_mm + surface_offset
