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
    # 400 microsteps / 8.0 mm = 50.0 steps/mm.
    # If 2-start (T8x4, 4.0 mm lead): 100.0 steps/mm.
    # If 1-start (T8x2, 2.0 mm lead): 200.0 steps/mm.
    # Y axis: 20-tooth GT2 belt pulley (40.0 mm/rev), 200 steps/rev, 2x microstepping = 10.0 steps/mm.
    lead_screw_pitch_mm: float = 2.0
    lead_screw_lead_mm: float = 8.0
    y_belt_pulley_mm_rev: float = 40.0
    microsteps: int = 2
    steps_per_mm_x: float = 50.0
    steps_per_mm_y: float = 10.0
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

    # 2. Homed End-Effector top wrt course origin: 32.5 mm right, 6.0 mm behind, 97.6 mm above
    #    (Relative to tower datum: +4.0 mm X, +37.5 mm Y, -29.9 mm Z)
    ee_from_datum_x_mm: float = 4.0
    ee_from_datum_y_mm: float = 37.5
    ee_from_datum_z_mm: float = -29.9
    # -> In plate coordinates at motor (0,0,0): EE = (32.5, -6.0, 97.6) mm -> X[32.5..259.5], Y[-6.0..122.0]

    # 3. Homed Camera optical center: 54.1 mm ahead (+Y) and 37.8 mm above (+Z) EE origin
    #    -> Cam = (32.5, +48.1, 135.4) mm -> X[32.5..259.5], Y[48.1..176.1]
    #    (Relative to tower datum: +4.0 mm X, +91.6 mm Y, +7.9 mm Z)
    cam_from_datum_x_mm: float = 4.0
    cam_from_datum_y_mm: float = 91.6
    cam_from_datum_z_mm: float = 7.9
    nominal_cam_origin_x_mm: float = 32.5
    nominal_cam_origin_y_mm: float = 48.1
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

    # --- Drop-Off Coordinate wrt Fiducial Plate Origin (X=224.5 right, Y=44.5 fore, Z=12.6 up) ---
    dropoff_plate_x_mm: float = 224.5
    dropoff_plate_y_mm: float = 44.5
    dropoff_plate_z_up_mm: float = 12.6
    # Motor Z_down at drop-off surface = 70.36 - 12.6 = 57.76 mm down from top home
    z_dropoff_base_down_mm: float = 57.76

    # --- Block Geometry & 4 Mass Classes (20 x 20 mm cubes) ---
    block_size_mm: float = 20.0
    block_height_mm: float = 20.0

    mass_classes: List[MassClassSpec] = field(
        default_factory=lambda: [
            MassClassSpec("low_infill", nominal_g=5.0, min_g=2.0, max_g=7.5),
            MassClassSpec("med_infill", nominal_g=10.0, min_g=7.5, max_g=12.5),
            MassClassSpec("high_infill", nominal_g=15.0, min_g=12.5, max_g=18.3),
            MassClassSpec("aluminum", nominal_g=21.6, min_g=18.3, max_g=35.0),
        ]
    )

    # --- Motion Feedrates (mm/s) ---
    feedrate_travel_mm_s: float = 15.0
    feedrate_z_mm_s: float = 10.0
    feedrate_fine_mm_s: float = 8.0

    # --- Eye-in-Hand Camera & Offset Parameters ---
    camera_index: int = 0
    frame_width: int = 640
    frame_height: int = 480

    default_kx_mm_per_px: float = 0.25
    default_ky_mm_per_px: float = 0.25
    default_cam_skew_deg: float = 0.0

    # Camera is offset from gripper in CAD:
    # Camera homed = (32.5, +48.1, 135.4), EE homed = (32.5, -6.0, 97.6)
    # Camera is 54.1 mm ahead (+Y) and 37.8 mm above (+Z) EE.
    # Therefore Gripper relative to Camera is (0.0 mm X, -54.1 mm Y).
    camera_to_gripper_offset_x_mm: float = 0.0
    camera_to_gripper_offset_y_mm: float = -54.1

    # --- Calibration Fiducials (15 x 15 mm squares at the 4 corners of the 284 x 167.4 mm outer border) ---
    aruco_marker_id: int = 1
    fiducial_size_mm: float = 15.0
    # Calibration Fiducial search point in motor travel coordinates (at X=0.0, Y=100.0, Marker #1 is centered)
    fiducial_base_x_mm: float = 0.0
    fiducial_base_y_mm: float = 100.0
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
            1: (7.5, 159.9),     # Marker ID 1: Top-Left (detected at motor X=0, Y=100)
            2: (276.5, 159.9),   # Marker ID 2: Top-Right (detected at motor X=220, Y=100)
            3: (276.5, 7.5),     # Marker ID 3: Bottom-Right
        }
    )
    calibration_step_x_mm: float = 15.0
    calibration_step_y_mm: float = 0.0

    # --- Global Scan Grid Coordinates (in motor travel coordinates within 227 x 128 mm envelope) ---
    # 6-point (3x2) overlapping grid shifted further back in Y (0.0 and 58.0 mm) because camera is +54.1 mm ahead of EE:
    # With Cam Y origin ~48.1..61.3 mm and 20 mm cube top-face FOV (~135 x 101 mm at Z=0):
    # Y=0 covers back edge (Cam Y ~48..61 mm -> Y[10..99] mm), Y=58 covers front edge (Cam Y ~106..119 mm -> Y[68..157] mm).
    scan_grid_points_mm: List[Tuple[float, float]] = field(
        default_factory=lambda: [
            (20.0, 58.0),
            (108.0, 58.0),
            (196.0, 58.0),
            (196.0, 0.0),
            (108.0, 0.0),
            (20.0, 0.0),
        ]
    )

    # --- Sorting / Stacking Drop-Off Zone (X=224.5 right, Y=44.5 fore, Z=12.6 mm up) ---
    stack_zones_mm: Dict[str, Tuple[float, float]] = field(
        default_factory=lambda: {
            "aluminum": (224.5, 44.5),
            "high_infill": (224.5, 44.5),
            "med_infill": (224.5, 44.5),
            "low_infill": (224.5, 44.5),
            "white": (224.5, 44.5),
            "red": (224.5, 44.5),
            "green": (224.5, 44.5),
            "blue": (224.5, 44.5),
            "yellow": (224.5, 44.5),
            "reject": (55.0, 44.5),
        }
    )

    # --- Mass Verification Limits (grams) ---
    min_valid_mass_g: float = 2.0
    max_valid_mass_g: float = 35.0

    # --- Block Contour Filtering (pixels^2 at z_scan_mm and hover z_hover_mm) ---
    min_contour_area_px: float = 2200.0
    max_contour_area_px: float = 35000.0
    deduplication_radius_mm: float = 18.0

    # --- Classical Computer Vision HSV Color Ranges ---
    hsv_ranges: Dict[str, HSVRange] = field(
        default_factory=lambda: {
            "white": HSVRange(
                name="white",
                lower1=(0, 0, 175),
                upper1=(179, 50, 255),
                display_hex="#f8fafc",
            ),
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
        """Classifies a block into one of the 4 mass classes (or 'reject')."""
        for mc in self.mass_classes:
            if mc.min_g <= mass_g <= mc.max_g:
                return mc.name
        return "reject"
