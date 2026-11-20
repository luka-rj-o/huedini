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

SERIAL_SLICE_TIMEOUT_S = 0.2
IDLE_KEEPALIVE_INTERVAL_S = 25.0
SAFE_WHEN_ESTOPPED = ("ESTOP", "SLEEP", "SETUP", "STATUS", "TARE", "WEIGH", "PING")
IDEMPOTENT_RETRY_COMMANDS = ("TARE", "WEIGH", "SETUP", "STATUS", "SLEEP")
IDEMPOTENT_RETRY_PREFIXES = ("GRIP ", "STEPS ")


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
        command_log_callback: Optional[Callable[[str, list[str], float], None]] = None,
    ):
        self.config = config
        self.mock_hardware = mock_hardware
        self.pose = GantryPose(z_mm=config.z_scan_mm)
        self.pose_update_callback = pose_update_callback
        self.event_callback = event_callback
        self.command_log_callback = command_log_callback
        self._ser: Optional["serial.Serial"] = None
        self._lock = threading.Lock()
        self._rx_buffer = bytearray()
        self._running = False
        self._monitor_thread: Optional[threading.Thread] = None
        self._abort_flag = False
        self._abort_generation = 0
        self._last_activity_mono = time.monotonic()
        self._simulated_held_mass_g: Optional[float] = None

    def _start_event_monitor(self) -> None:
        if self._monitor_thread is not None and self._monitor_thread.is_alive():
            return
        self._running = True
        self._monitor_thread = threading.Thread(
            target=self._event_monitor_loop,
            name="esp32-event-monitor",
            daemon=True,
        )
        self._monitor_thread.start()

    def _handle_unsolicited_line(self, line: str) -> bool:
        if line == "EVENT:START_BTN":
            if self.event_callback:
                self.event_callback("START_SCAN")
            return True
        if line == "EVENT:ESTOP":
            self._abort_flag = True
            if self.event_callback:
                self.event_callback("EMERGENCY_STOP")
            return True
        if line == "EVENT:CHIME_BTN":
            if self.event_callback:
                self.event_callback("CHIME_BTN")
            return True
        return False

    def _drain_unsolicited_events(self) -> None:
        """Non-blockingly drains available serial bytes and dispatches complete EVENT:* lines."""
        if not self._ser or not self._ser.is_open:
            return
        waiting = self._ser.in_waiting
        if waiting <= 0:
            return
        chunk = self._ser.read(waiting)
        if not chunk:
            return
        self._last_activity_mono = time.monotonic()
        self._rx_buffer.extend(chunk)
        while b"\n" in self._rx_buffer:
            raw_line, _, rest = self._rx_buffer.partition(b"\n")
            self._rx_buffer = bytearray(rest)
            line = raw_line.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            logger.debug("RX (idle) <- ESP32: %s", line)
            self._handle_unsolicited_line(line)

    def _read_serial_line(self) -> bytes:
        """Reads a complete newline-terminated line using short-slice serial reads."""
        if b"\n" not in self._rx_buffer:
            chunk = self._ser.readline()
            if chunk:
                self._last_activity_mono = time.monotonic()
                self._rx_buffer.extend(chunk)
        if b"\n" in self._rx_buffer:
            raw_line, _, rest = self._rx_buffer.partition(b"\n")
            self._rx_buffer = bytearray(rest)
            return raw_line + b"\n"
        return b""

    def _idle_keepalive_check(self) -> None:
        """Sends a silent PING while holding self._lock to catch USB URB / ESP32 stalls early."""
        self._last_activity_mono = time.monotonic()
        try:
            self._send_command_locked("PING", timeout_s=2.0)
        except Exception as exc:
            logger.warning("Idle keepalive PING failed (%s); reconnecting serial bridge...", exc)
            self._reconnect()

    def _event_monitor_loop(self) -> None:
        while self._running:
            if self._lock.acquire(blocking=False):
                try:
                    self._drain_unsolicited_events()
                    if (
                        not self.mock_hardware
                        and self._ser is not None
                        and self._ser.is_open
                        and (time.monotonic() - self._last_activity_mono) >= IDLE_KEEPALIVE_INTERVAL_S
                    ):
                        self._idle_keepalive_check()
                except Exception as exc:
                    logger.warning("Idle serial monitor error (%s); attempting reconnect...", exc)
                    try:
                        self._reconnect()
                    except Exception as rec_exc:
                        logger.debug("Idle serial reconnect failed: %s", rec_exc)
                    self._last_activity_mono = time.monotonic()
                finally:
                    self._lock.release()
            time.sleep(0.025)

    def _open_serial_no_reset(self, port: str) -> None:
        """Opens the serial port without toggling DTR/RTS so the ESP32 keeps running without rebooting."""
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = self.config.serial_baud
        ser.timeout = SERIAL_SLICE_TIMEOUT_S
        ser.write_timeout = 2.0
        ser.dtr = False
        ser.rts = False
        ser.open()
        self._ser = ser
        time.sleep(0.05)
        self._ser.reset_input_buffer()
        self._rx_buffer.clear()
        self._last_activity_mono = time.monotonic()

    def _open_and_reset_serial(self, port: str) -> None:
        """Opens the serial port with short slice read timeout and resets the ESP32 into run mode."""
        self._ser = serial.Serial(
            port=port,
            baudrate=self.config.serial_baud,
            timeout=SERIAL_SLICE_TIMEOUT_S,
            write_timeout=2.0,
        )
        # Ensure ESP32 boots into normal SPI Flash run mode (not download boot)
        self._ser.dtr = False
        self._ser.rts = True
        time.sleep(0.15)
        self._ser.rts = False
        time.sleep(1.0)
        self._ser.reset_input_buffer()
        self._rx_buffer.clear()
        self._last_activity_mono = time.monotonic()

    def connect(self) -> None:
        if self.mock_hardware:
            logger.info("ESP32SerialBridge initialized in --mock-hardware mode.")
            return

        if serial is None:
            raise RuntimeError("pyserial is not installed. Run: pip install pyserial")

        port = self._auto_detect_port(self.config.serial_port)
        if not port:
            logger.warning(
                "No ESP32 USB serial device found (/dev/ttyUSB* or /dev/ttyACM*). "
                "Starting in offline mode (Web GUI and camera remain online)."
            )
            self._ser = None
            return

        logger.info("Connecting to ESP32 on %s @ %d baud...", port, self.config.serial_baud)
        try:
            self._open_and_reset_serial(port)

            # Configure steps/mm and enable TMC2209 drivers
            self.send_command(
                f"STEPS X{self.config.steps_per_mm_x:.2f} "
                f"Y{self.config.steps_per_mm_y:.2f} "
                f"Z{self.config.steps_per_mm_z:.2f}"
            )
            self.send_command("SETUP")
            self._start_event_monitor()
        except Exception as exc:
            msg = str(exc)
            if "VM" in msg or "UART" in msg or "driver not responding" in msg:
                logger.warning(
                    "Motor supply VM is unpowered (%s). Continuing in idle/manual-test mode.",
                    msg,
                )
                self._start_event_monitor()
            else:
                logger.warning(
                    "ESP32 initial connection failed on %s (%s). Starting in offline mode so GUI remains active.",
                    port,
                    msg,
                )
                if self._ser:
                    try:
                        self._ser.close()
                    except Exception:
                        pass
                    self._ser = None

    def _close_ser_quietly(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def _reconnect(self) -> bool:
        """
        Recovers a broken or dropped serial connection (e.g. after cp210x urb stopped -32,
        POSIX EIO, or ESP32 stall) using a two-stage strategy:
          1. Soft reopen without toggling DTR/RTS + fast PING (preserves ESP32 state & homed position).
          2. Hard RTS reset + STEPS/SETUP fallback if the ESP32 itself is unresponsive.
        """
        if self.mock_hardware:
            return True

        if serial is None:
            return False

        logger.warning("Attempting automatic serial bridge reconnection...")
        self._close_ser_quietly()
        time.sleep(0.25)

        port = self._auto_detect_port(self.config.serial_port)
        if not port:
            return False

        # Stage 1: Soft reopen (clears Linux cp210x -EPIPE / stopped URB without resetting ESP32)
        try:
            logger.info("Soft-reopening %s @ %d baud (no ESP32 reset)...", port, self.config.serial_baud)
            self._open_serial_no_reset(port)
            self._send_command_locked("PING", timeout_s=1.0)
            self._start_event_monitor()
            logger.info("ESP32SerialBridge soft-reconnected on %s (stepper position preserved).", port)
            return True
        except Exception as soft_exc:
            logger.warning("Soft reopen failed on %s (%s); falling back to ESP32 RTS reset...", port, soft_exc)
            self._close_ser_quietly()
            time.sleep(0.25)

        # Stage 2: Hard RTS reset fallback
        try:
            logger.info("Hard-reconnecting to ESP32 on %s @ %d baud (RTS reset)...", port, self.config.serial_baud)
            self._open_and_reset_serial(port)
            steps_cmd = (
                f"STEPS X{self.config.steps_per_mm_x:.2f} "
                f"Y{self.config.steps_per_mm_y:.2f} "
                f"Z{self.config.steps_per_mm_z:.2f}"
            )
            self._send_command_locked(steps_cmd, timeout_s=3.0)
            try:
                self._send_command_locked("SETUP", timeout_s=3.0)
            except Exception as setup_exc:
                msg = str(setup_exc)
                if "VM" in msg or "UART" in msg or "driver not responding" in msg:
                    logger.warning("Motor supply VM is unpowered (%s). Continuing in idle/manual-test mode.", msg)
                else:
                    raise
            self._start_event_monitor()
            logger.info("ESP32SerialBridge successfully hard-reconnected and initialized on %s.", port)
            return True
        except Exception as exc:
            logger.error("ESP32SerialBridge reconnection failed: %s", exc)
            self._close_ser_quietly()
            return False

    @staticmethod
    def _auto_detect_port(preferred: str) -> Optional[str]:
        if serial is None:
            return preferred
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if preferred in ports:
            return preferred
        for candidate in ("/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyACM0", "/dev/ttyACM1", "COM3", "COM4", "COM5", "COM7"):
            if candidate in ports:
                return candidate
        # Never fall back to internal hardware UART ports like /dev/ttyS0 or /dev/ttyAMA0
        for p in ports:
            if not p.startswith(("/dev/ttyS", "/dev/ttyAMA")):
                return p
        return None

    @property
    def is_connected(self) -> bool:
        return self.mock_hardware or (self._ser is not None and self._ser.is_open)

    def close(self) -> None:
        self._running = False
        if self._monitor_thread is not None and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=0.5)
        if self._ser and self._ser.is_open:
            self._ser.close()

    def trigger_emergency_stop(self) -> None:
        self._abort_flag = True
        self._abort_generation += 1
        if not self.mock_hardware and self._ser and self._ser.is_open:
            try:
                self._ser.write(b"ESTOP\n")
                self._ser.flush()
            except Exception as exc:
                logger.error("Failed to send ESTOP: %s", exc)

    def clear_emergency_stop(self) -> None:
        self._abort_flag = False

    def send_command(self, cmd: str, timeout_s: Optional[float] = None) -> list[str]:
        """
        Sends a single ASCII command terminated with '\\n' and blocks until completion:
          - Automated protocol commands wait for 'OK' or 'ERR:<reason>'.
          - Interactive manual commands wait for 'OK', parameter responses, or motion completion.
        Flushes stale unread bytes before transmission so previous timeouts cannot desync replies.
        Automatically recovers from USB serial URB stalls (-EPIPE) on TimeoutError and retries
        idempotent non-motion commands once after reconnecting.
        Raises RuntimeError on explicit errors, E-STOP, or detected axis crashes.
        """
        cmd_clean = cmd.strip()
        upper = cmd_clean.upper()
        if self._abort_flag and upper not in SAFE_WHEN_ESTOPPED:
            raise RuntimeError("ERR:EMERGENCY_STOP_ACTIVE")

        t_start = time.monotonic()

        def _log_and_return(results: list[str]) -> list[str]:
            dur_ms = (time.monotonic() - t_start) * 1000.0
            if self.command_log_callback:
                try:
                    self.command_log_callback(cmd_clean, list(results), dur_ms)
                except Exception:
                    pass
            return results

        if self.mock_hardware:
            return _log_and_return(self._simulate_command(cmd_clean))

        with self._lock:
            try:
                return _log_and_return(self._send_command_locked(cmd_clean, timeout_s))
            except TimeoutError as exc:
                logger.warning(
                    "Command '%s' timed out (%s); reconnecting serial bridge to recover from USB/ESP32 stall...",
                    cmd_clean,
                    exc,
                )
                reconnected = self._reconnect()
                is_idempotent = (
                    upper in IDEMPOTENT_RETRY_COMMANDS
                    or upper.startswith(IDEMPOTENT_RETRY_PREFIXES)
                )
                if reconnected and is_idempotent:
                    logger.info("Retrying idempotent command '%s' after automatic reconnect...", cmd_clean)
                    return _log_and_return(self._send_command_locked(cmd_clean, timeout_s))
                raise

    def _send_command_locked(self, cmd_clean: str, timeout_s: Optional[float] = None) -> list[str]:
        """Executes a single serial command while self._lock is held."""
        upper = cmd_clean.upper()
        start_gen = self._abort_generation

        if not self._ser or not self._ser.is_open:
            if not self._reconnect():
                if upper == "STATUS":
                    return ["STATUS: ESP32 OFFLINE", "OK"]
                logger.warning("Command '%s' dropped: ESP32 is offline.", cmd_clean)
                return ["ERR: ESP32 OFFLINE"]

        # Drain any pending hardware switch events before transmitting; keep partial lines intact
        try:
            self._drain_unsolicited_events()
            if self._rx_buffer and not self._rx_buffer.startswith(b"EVENT:"):
                logger.debug("Flushed %d stale serial bytes before TX", len(self._rx_buffer))
                self._rx_buffer.clear()
        except Exception:
            pass

        io_exceptions = (serial.SerialException, OSError) if serial is not None else (OSError,)
        logger.debug("TX -> ESP32: %s", cmd_clean)
        try:
            self._ser.write((cmd_clean + "\n").encode("ascii"))
            self._ser.flush()
            self._last_activity_mono = time.monotonic()
        except io_exceptions as io_err:
            logger.warning("Serial write failed (%s); attempting auto-reconnect...", io_err)
            if self._reconnect():
                logger.info("Retrying TX -> ESP32: %s after successful reconnect", cmd_clean)
                self._ser.write((cmd_clean + "\n").encode("ascii"))
                self._ser.flush()
                self._last_activity_mono = time.monotonic()
            else:
                raise RuntimeError(f"Serial write failed: {io_err} (reconnection failed)") from io_err

        lines: list[str] = []
        effective_timeout = float(timeout_s) if timeout_s is not None else self.config.serial_timeout_s
        deadline = time.monotonic() + effective_timeout

        cmd_lower = cmd_clean.lower()
        is_motion_cmd = (
            cmd_lower.startswith("home s")
            or cmd_lower.startswith(("s1 home", "s2 home", "s3 home"))
            or (
                len(cmd_lower) > 3
                and cmd_lower[0] == "s"
                and cmd_lower[1] in "123"
                and cmd_lower[2] == " "
                and (cmd_lower[3].isdigit() or cmd_lower[3] in "+-")
            )
        )
        is_manual_setting = (
            cmd_lower.startswith(("s1 ", "s2 ", "s3 ", "servo ", "relay ", "l", "help"))
            and not is_motion_cmd
            and not upper.startswith(("SETUP", "STATUS", "STEPS"))
        )

        while time.monotonic() < deadline:
            if self._abort_generation != start_gen or (self._abort_flag and upper not in SAFE_WHEN_ESTOPPED):
                raise RuntimeError("ERR:EMERGENCY_STOP_ACTIVE")

            try:
                raw = self._read_serial_line()
            except io_exceptions as io_err:
                logger.warning("Serial read failed (%s); attempting auto-reconnect...", io_err)
                if self._reconnect():
                    raise RuntimeError(
                        f"Serial connection was reset during command '{cmd_clean}'; "
                        f"bridge automatically reconnected and restored driver state. Please retry."
                    ) from io_err
                raise RuntimeError(f"Serial read failed: {io_err} (reconnection failed)") from io_err

            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            logger.debug("RX <- ESP32: %s", line)

            if self._handle_unsolicited_line(line):
                continue

            lines.append(line)

            # Explicit crash detection from firmware [ALERT] or ERR:STALL
            if "stall detected" in line.lower() or line.startswith("ERR:STALL") or line.startswith("[ALERT]"):
                raise RuntimeError(f"CRASH_DETECTED: {line}")

            if line == "OK":
                self._last_activity_mono = time.monotonic()
                return lines

            if line.startswith("ERR:") or line.startswith("[ERROR]"):
                if "use hv<speed> or hs<sensitivity>" in line.lower():
                    logger.warning("Ignoring non-fatal firmware warning: %s", line)
                    continue
                raise RuntimeError(line)

            if is_motion_cmd:
                if (
                    "Position set to 0." in line
                    or "move complete." in line
                    or "timed out" in line
                    or "homing cancelled" in line
                ):
                    time.sleep(0.04)
                    while self._ser.in_waiting > 0:
                        extra = self._read_serial_line().decode("utf-8", errors="replace").strip()
                        if extra:
                            lines.append(extra)
                            if extra == "OK":
                                return lines
                    return lines

            elif is_manual_setting:
                if line.startswith("-> ") or line.startswith("---") or line.startswith("["):
                    time.sleep(0.06)
                    if self._ser.in_waiting == 0:
                        return lines

        raise TimeoutError(f"ESP32 timed out ({effective_timeout:.1f}s) waiting for completion of command: {cmd_clean}")

    # ------------------------------------------------------------------
    # High-level deterministic operations
    # ------------------------------------------------------------------
    def ensure_awake(self) -> None:
        """Ensures steppers are enabled and drivers initialized (sends SETUP if needed)."""
        try:
            self.send_command("SETUP", timeout_s=4.0)
        except Exception as exc:
            logger.warning("ensure_awake failed: %s", exc)

    def home_all(self, timeout_s: float = 120.0) -> None:
        """Commands ESP32 to home Z, X, and Y axes using TMC2209 StallGuard4 with safe timeout."""
        self.ensure_awake()
        self.send_command("HOME", timeout_s=timeout_s)
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
        timeout_s: Optional[float] = None,
    ) -> None:
        """Coordinated linear move to target coordinates (mm/deg) at feedrate F with dynamic timeout."""
        tx = self.pose.x_mm if x_mm is None else float(x_mm)
        ty = self.pose.y_mm if y_mm is None else float(y_mm)
        tz = self.pose.z_mm if z_mm is None else float(z_mm)
        ta = self.pose.angle_deg if angle_deg is None else float(angle_deg)
        tf = self.config.feedrate_travel_mm_s if feedrate_mm_s is None else float(feedrate_mm_s)

        # Enforce physical mechanical limit clamping to prevent collisions:
        tx = max(0.0, min(self.config.travel_x_mm, tx))
        ty = max(0.0, min(self.config.travel_y_mm, ty))
        tz = max(0.0, min(self.config.travel_z_bottom_mm, tz))

        # EE rotation clearance: never rotate servo A when Y < servo_safe_y_mm (10 mm),
        # because swinging the end-effector near Y=0 hits the rear end stop.
        safe_y = self.config.servo_safe_y_mm
        if abs(ta - self.pose.angle_deg) > 0.1 and ty < safe_y:
            if self.pose.y_mm < safe_y:
                self.move_to(
                    y_mm=safe_y,
                    angle_deg=self.pose.angle_deg,
                    feedrate_mm_s=tf,
                    timeout_s=timeout_s,
                )
            self.move_to(
                y_mm=max(self.pose.y_mm, safe_y),
                angle_deg=ta,
                feedrate_mm_s=tf,
                timeout_s=timeout_s,
            )

        # Dynamically scale timeout based on travel distance and feedrate
        dist = max(abs(tx - self.pose.x_mm), abs(ty - self.pose.y_mm), abs(tz - self.pose.z_mm))
        calc_timeout = max(self.config.serial_timeout_s, (dist / max(1.0, tf)) * 2.5 + 10.0)
        eff_timeout = timeout_s if timeout_s is not None else calc_timeout

        cmd = f"MOVE X{tx:.2f} Y{ty:.2f} Z{tz:.2f} A{ta:.1f} F{tf:.1f}"
        self.send_command(cmd, timeout_s=eff_timeout)

        self.pose.x_mm = tx
        self.pose.y_mm = ty
        self.pose.z_mm = tz
        self.pose.angle_deg = ta
        self._notify_pose()

    def press_into_surface(
        self,
        nominal_z_mm: float,
        max_press_mm: float,
        force_g: float,
        feedrate_mm_s: Optional[float] = None,
    ) -> float:
        """
        Gently presses the cube/end-effector downward into the floor or stack using
        HX711 compressive force feedback, strictly bounded by travel_z_max_mm.
        """
        max_z = max(
            self.pose.z_mm,
            min(self.config.travel_z_max_mm, float(nominal_z_mm) + max(0.0, float(max_press_mm))),
        )
        press_f = max(1.0, float(self.config.feedrate_press_mm_s if feedrate_mm_s is None else feedrate_mm_s))
        req_force_g = max(0.0, float(force_g))
        dist = max(0.0, max_z - self.pose.z_mm)
        timeout = max(self.config.serial_timeout_s, (dist / press_f) * 2.5 + 5.0)

        lines = self.send_command(
            f"PRESS Z{max_z:.2f} D{req_force_g:.1f} F{press_f:.1f}",
            timeout_s=timeout,
        )
        actual_z = max_z
        for line in reversed(lines):
            if line.startswith("PRESSED "):
                for tok in line.split():
                    if tok.startswith("Z:"):
                        try:
                            actual_z = float(tok.split(":", 1)[1])
                        except ValueError:
                            pass
                break
        self.pose.z_mm = max(0.0, min(self.config.travel_z_max_mm, actual_z))
        self._notify_pose()
        return self.pose.z_mm

    def set_gripper(self, closed: bool) -> None:
        """
        Controls the Normally-Closed solenoid gripper via PIN_RELAY.
        closed=True  -> GRIP 1 (grip block)
        closed=False -> GRIP 0 (open/release block)
        """
        val = 1 if closed else 0
        self.send_command(f"GRIP {val}", timeout_s=3.0)
        self.pose.grip_closed = closed
        self._notify_pose()

    def tare_scale(self) -> None:
        """Zeroes the HX711 load cell offset before picking a block."""
        self.send_command("TARE", timeout_s=4.0)

    def weigh_block(self) -> float:
        """Samples the HX711 load cell and parses WEIGHT:<float> in grams."""
        lines = self.send_command("WEIGH", timeout_s=4.0)
        for line in reversed(lines):
            if line.startswith("WEIGHT:"):
                return float(line.split(":", 1)[1].strip())
        raise RuntimeError(f"Expected WEIGHT:<float> from ESP32, got: {lines}")

    def play_chime(self, tune: str = "TADA") -> None:
        """Plays a celebratory stepper victory chime on the X/Z axes without losing position."""
        self.send_command(f"CHIME {tune}".strip(), timeout_s=10.0)

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

        if upper.startswith("PRESS"):
            tokens = upper.split()[1:]
            tz, tf = self.pose.z_mm, self.config.feedrate_press_mm_s
            for tok in tokens:
                if tok.startswith("Z"):
                    tz = float(tok[1:])
                elif tok.startswith("F"):
                    tf = max(1.0, float(tok[1:]))
            sim_z = min(tz, self.pose.z_mm + min(1.5, max(0.0, tz - self.pose.z_mm)))
            self._animate_mock_move(self.pose.x_mm, self.pose.y_mm, sim_z, self.pose.angle_deg, tf)
            return [f"PRESSED Z:{sim_z:.2f} DF:-36.00"]

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

        if upper == "CHIME" or upper.startswith("CHIME "):
            tune = upper.split(maxsplit=1)[1] if " " in upper else "TADA"
            time.sleep(1.5 if tune in ("WIN95", "95") else 0.75)
            return [f"CHIME:{tune}"]

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
