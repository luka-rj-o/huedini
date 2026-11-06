"""Deterministic Serial Master bridge between Raspberry Pi Zero W and ESP32."""

from dataclasses import dataclass
import logging
import math
import random
import threading
import time
from typing import Callable, Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None  # Handled gracefully when running in --mock-hardware mode

from .config import GantryConfig

logger = logging.getLogger("gantry.serial")


@dataclass
class GantryPose:
    """Current physical or simulated gantry coordinates."""
    x_mm: float = 0.0
    y_mm: float = 0.0
    z_mm: float = 0.0
    angle_deg: float = 0.0
    grip_closed: bool = True  # Normally-Closed (NC) solenoid default

    def to_dict(self) -> dict:
        return {
            "x_mm": round(self.x_mm, 2),
            "y_mm": round(self.y_mm, 2),
            "z_mm": round(self.z_mm, 2),
            "angle_deg": round(self.angle_deg, 1),
            "grip_closed": self.grip_closed,
        }


class ESP32SerialBridge:
    """
    Line-terminated ASCII protocol master over USB Serial (/dev/ttyUSB0 or /dev/ttyACM0).
    Waits for deterministic 'OK' completion replies before returning so that vision
    frames are only ever captured when the gantry is completely stationary.
    """

    def __init__(
        self,
        config: GantryConfig,
        mock_hardware: bool = False,
        pose_update_callback: Optional[Callable[[GantryPose], None]] = None,
        event_callback: Optional[Callable[[str], None]] = None,
    ):
        self.config = config
        self.mock_hardware = mock_hardware
        self.pose = GantryPose(z_mm=config.z_scan_mm)
        self.pose_update_callback = pose_update_callback
        self.event_callback = event_callback
        self._ser: Optional["serial.Serial"] = None
        self._lock = threading.Lock()
        self._abort_flag = False
        self._simulated_held_mass_g: Optional[float] = None

    def connect(self) -> None:
        if self.mock_hardware:
            logger.info("ESP32SerialBridge initialized in --mock-hardware mode.")
            return

        if serial is None:
            raise RuntimeError("pyserial is not installed. Run: pip install pyserial")

        port = self._auto_detect_port(self.config.serial_port)
        logger.info("Connecting to ESP32 on %s @ %d baud...", port, self.config.serial_baud)
        self._ser = serial.Serial(
            port=port,
            baudrate=self.config.serial_baud,
            timeout=self.config.serial_timeout_s,
        )
        # Ensure ESP32 boots into normal SPI Flash run mode (not download boot):
        self._ser.dtr = False
        self._ser.rts = True
        time.sleep(0.15)
        self._ser.rts = False
        time.sleep(1.0)
        self._ser.reset_input_buffer()

        # Configure steps/mm and enable TMC2209 drivers
        self.send_command(
            f"STEPS X{self.config.steps_per_mm_x:.2f} "
            f"Y{self.config.steps_per_mm_y:.2f} "
            f"Z{self.config.steps_per_mm_z:.2f}"
        )
        self.send_command("SETUP")

    @staticmethod
    def _auto_detect_port(preferred: str) -> str:
        if serial is None:
            return preferred
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if preferred in ports:
            return preferred
        for candidate in ("/dev/ttyUSB0", "/dev/ttyACM0", "COM3", "COM4", "COM5", "COM7"):
            if candidate in ports:
                return candidate
        return ports[0] if ports else preferred

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()

    def trigger_emergency_stop(self) -> None:
        self._abort_flag = True
        if not self.mock_hardware and self._ser and self._ser.is_open:
            try:
                self._ser.write(b"ESTOP\n")
                self._ser.flush()
            except Exception as exc:
                logger.error("Failed to send ESTOP: %s", exc)

    def clear_emergency_stop(self) -> None:
        self._abort_flag = False

    def send_command(self, cmd: str) -> list[str]:
        """
        Sends a single ASCII command terminated with '\\n' and blocks until completion:
          - Automated protocol commands wait for 'OK' or 'ERR:<reason>'.
          - Interactive manual commands (e.g. 'home s1 -', 's1 hs20', 's1 500') wait for
            their corresponding completion marker or parameter response without timing out.
        Returns all informational lines received from the ESP32.
        """
        cmd_clean = cmd.strip()
        if self._abort_flag and cmd_clean.upper() != "ESTOP":
            raise RuntimeError("ERR:EMERGENCY_STOP_ACTIVE")

        if self.mock_hardware:
            return self._simulate_command(cmd_clean)

        with self._lock:
            if not self._ser or not self._ser.is_open:
                raise RuntimeError("Serial port is not open.")

            logger.debug("TX -> ESP32: %s", cmd_clean)
            self._ser.write((cmd_clean + "\n").encode("ascii"))
            self._ser.flush()

            lines: list[str] = []
            deadline = time.monotonic() + self.config.serial_timeout_s

            cmd_lower = cmd_clean.lower()
            is_motion_cmd = cmd_lower.startswith("home s") or (
                len(cmd_lower) > 3
                and cmd_lower[0] == "s"
                and cmd_lower[1] in "123"
                and cmd_lower[2] == " "
                and (cmd_lower[3].isdigit() or cmd_lower[3] in "+-")
            )
            is_manual_setting = (
                cmd_lower.startswith(("s1 ", "s2 ", "s3 ", "servo ", "relay ", "l", "help"))
                and not is_motion_cmd
                and not cmd_clean.upper().startswith(("SETUP", "STATUS", "STEPS"))
            )

            while time.monotonic() < deadline:
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                logger.debug("RX <- ESP32: %s", line)

                if line == "EVENT:START_BTN":
                    if self.event_callback:
                        self.event_callback("START_SCAN")
                    continue

                lines.append(line)

                if line == "OK":
                    return lines

                if line.startswith("ERR:") or line.startswith("[ERROR]"):
                    raise RuntimeError(line)

                if is_motion_cmd:
                    if (
                        "Position set to 0." in line
                        or "move complete." in line
                        or "stall detected" in line
                        or "timed out" in line
                        or "homing cancelled" in line
                    ):
                        return lines

                elif is_manual_setting:
                    if line.startswith("-> ") or line.startswith("---") or line.startswith("["):
                        time.sleep(0.06)
                        if self._ser.in_waiting == 0:
                            return lines

            raise TimeoutError(f"ESP32 timed out waiting for completion of command: {cmd_clean}")

    # ------------------------------------------------------------------
    # High-level deterministic operations
    # ------------------------------------------------------------------
    def home_all(self) -> None:
        """Commands ESP32 to home Z, X, and Y axes using TMC2209 StallGuard4."""
        self.send_command("HOME")
        self.pose.x_mm = 0.0
        self.pose.y_mm = 0.0
        self.pose.z_mm = 0.0
        self._notify_pose()

    def move_to(
        self,
        x_mm: Optional[float] = None,
        y_mm: Optional[float] = None,
        z_mm: Optional[float] = None,
        angle_deg: Optional[float] = None,
        feedrate_mm_s: Optional[float] = None,
    ) -> None:
        """Coordinated linear move to target coordinates (mm/deg) at feedrate F."""
        tx = self.pose.x_mm if x_mm is None else float(x_mm)
        ty = self.pose.y_mm if y_mm is None else float(y_mm)
        tz = self.pose.z_mm if z_mm is None else float(z_mm)
        ta = self.pose.angle_deg if angle_deg is None else float(angle_deg)
        tf = self.config.feedrate_travel_mm_s if feedrate_mm_s is None else float(feedrate_mm_s)

        # Enforce physical mechanical limit clamping to prevent collisions:
        tx = max(0.0, min(self.config.travel_x_mm, tx))
        ty = max(0.0, min(self.config.travel_y_mm, ty))
        tz = max(0.0, min(self.config.travel_z_bottom_mm, tz))

        cmd = f"MOVE X{tx:.2f} Y{ty:.2f} Z{tz:.2f} A{ta:.1f} F{tf:.1f}"
        self.send_command(cmd)

        self.pose.x_mm = tx
        self.pose.y_mm = ty
        self.pose.z_mm = tz
        self.pose.angle_deg = ta
        self._notify_pose()

    def set_gripper(self, closed: bool) -> None:
        """
        Controls the Normally-Closed solenoid gripper via PIN_RELAY.
        closed=True  -> GRIP 1 (grip block)
        closed=False -> GRIP 0 (open/release block)
        """
        val = 1 if closed else 0
        self.send_command(f"GRIP {val}")
        self.pose.grip_closed = closed
        self._notify_pose()

    def tare_scale(self) -> None:
        """Zeroes the HX711 load cell offset before picking a block."""
        self.send_command("TARE")

    def weigh_block(self) -> float:
        """Samples the HX711 load cell and parses WEIGHT:<float> in grams."""
        lines = self.send_command("WEIGH")
        for line in reversed(lines):
            if line.startswith("WEIGHT:"):
                return float(line.split(":", 1)[1].strip())
        raise RuntimeError(f"Expected WEIGHT:<float> from ESP32, got: {lines}")

    def set_simulated_held_mass(self, mass_g: Optional[float]) -> None:
        """Used in --mock-hardware mode so WEIGH returns the picked block's mass."""
        self._simulated_held_mass_g = mass_g

    def _notify_pose(self) -> None:
        if self.pose_update_callback:
            self.pose_update_callback(self.pose)

    # ------------------------------------------------------------------
    # Mock Hardware Simulation (--mock-hardware)
    # ------------------------------------------------------------------
    def _simulate_command(self, cmd: str) -> list[str]:
        upper = cmd.upper()
        if upper == "HOME":
            self._animate_mock_move(0.0, 0.0, 0.0, self.pose.angle_deg, 60.0)
            return ["-> S3 homed", "-> S1 homed", "-> S2 homed"]

        if upper.startswith("MOVE"):
            tokens = upper.split()[1:]
            tx, ty, tz, ta, tf = (
                self.pose.x_mm,
                self.pose.y_mm,
                self.pose.z_mm,
                self.pose.angle_deg,
                self.config.feedrate_travel_mm_s,
            )
            for tok in tokens:
                code, val = tok[0], float(tok[1:])
                if code == "X":
                    tx = val
                elif code == "Y":
                    ty = val
                elif code == "Z":
                    tz = val
                elif code == "A":
                    ta = val
                elif code == "F":
                    tf = max(5.0, val)
            self._animate_mock_move(tx, ty, tz, ta, tf)
            return []

        if upper.startswith("GRIP"):
            time.sleep(0.12)
            parts = upper.split()
            if len(parts) > 1 and parts[1] in ("0", "1"):
                self.pose.grip_closed = parts[1] == "1"
            return []

        if upper == "TARE":
            time.sleep(0.1)
            return []

        if upper == "WEIGH":
            time.sleep(0.18)
            if self._simulated_held_mass_g is not None:
                mass = self._simulated_held_mass_g + random.uniform(-0.3, 0.3)
            else:
                mass = round(random.uniform(48.0, 56.0), 2)
            return [f"WEIGHT:{mass:.2f}"]

        time.sleep(0.05)
        return [f"MOCK_ACK:{cmd}"]

    def _animate_mock_move(
        self, tx: float, ty: float, tz: float, ta: float, feedrate: float
    ) -> None:
        dist = math.sqrt(
            (tx - self.pose.x_mm) ** 2
            + (ty - self.pose.y_mm) ** 2
            + (tz - self.pose.z_mm) ** 2
        )
        # Speed up simulation 3x while still giving smooth telemetry interpolation
        duration = min(1.2, max(0.08, (dist / max(10.0, feedrate)) * 0.35))
        steps = max(2, int(duration / 0.04))
        sx, sy, sz, sa = (
            self.pose.x_mm,
            self.pose.y_mm,
            self.pose.z_mm,
            self.pose.angle_deg,
        )
        for i in range(1, steps + 1):
            if self._abort_flag:
                raise RuntimeError("ERR:EMERGENCY_STOP_ACTIVE")
            alpha = i / steps
            self.pose.x_mm = sx + (tx - sx) * alpha
            self.pose.y_mm = sy + (ty - sy) * alpha
            self.pose.z_mm = sz + (tz - sz) * alpha
            self.pose.angle_deg = sa + (ta - sa) * alpha
            self._notify_pose()
            time.sleep(duration / steps)
