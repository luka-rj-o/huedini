"""
Core 6-Phase Operational State Machine for the HUE-DINI Gantry Robot:
  Phase 1: IDLE         — Wait for START_SCAN trigger from PC GUI or ESP32 button
  Phase 2: CALIBRATING  — StallGuard4 homing + Eye-in-Hand ArUco Auto-Scale routine
  Phase 3: SCANNING     — Multi-point stop-and-capture global course scan
  Phase 4: PLANNING     — Deduplicated block catalog & clearance-based pick order
  Phase 5: PICKING & WEIGHING — Fine visual alignment, angle match, NC solenoid grip, HX711 weigh
  Phase 6: STACKING     — Sort by color & mass into pedestal/laydown zones and stack
"""

from collections import deque
import json
import logging
import math
from pathlib import Path
import queue
import threading
import time
from typing import Deque, Optional

from .config import GantryConfig
from .gamepad import (
    DEFAULT_GAMEPAD_MAPPING,
    GamepadSnapshot,
    TeleopInputMapper,
    XboxBluetoothManager,
    XboxGamepadReader,
    load_gamepad_mapping,
    normalize_gamepad_mapping,
    save_gamepad_mapping,
)
from .kinematics import BlockRecord, EyeInHandKinematics, WorldCatalogAndPlanner
from .report_logger import PickActionRecord, RunReportRecorder
from .serial_bridge import ESP32SerialBridge, GantryPose
from .telemetry_server import TelemetryServer
from .vision import VisionPipeline

logger = logging.getLogger("gantry.fsm")


class _ManualModeInterrupt(Exception):
    """Internal control-flow signal to cleanly hand off an active auto cycle to MANUAL mode."""


class GantryStateMachine:
    """Orchestrates vision, kinematics, serial motion, and telemetry."""

    def __init__(
        self,
        config: GantryConfig,
        mock_hardware: bool = False,
        ws_host: str = "0.0.0.0",
        ws_port: int = 8765,
    ):
        self.config = config
        self.mock_hardware = mock_hardware
        self.state = "IDLE"
        self._logs: Deque[str] = deque(maxlen=60)
        self.reports = RunReportRecorder()

        self.kinematics = EyeInHandKinematics(
            frame_width=config.frame_width,
            frame_height=config.frame_height,
            kx_mm_per_px=config.default_kx_mm_per_px,
            ky_mm_per_px=config.default_ky_mm_per_px,
            cam_skew_deg=config.default_cam_skew_deg,
            offset_x_mm=config.camera_to_gripper_offset_x_mm,
            offset_y_mm=config.camera_to_gripper_offset_y_mm,
            cam_homed_z_above_plate_mm=config.cam_homed_z_above_plate_mm,
            course_elevation_mm=config.course_elevation_mm,
            cam_origin_x_mm=config.nominal_cam_origin_x_mm,
            cam_origin_y_mm=config.nominal_cam_origin_y_mm,
            corner_fiducials=config.corner_fiducials_by_id,
            cam_tilt_x_mm_per_mm_z=config.cam_tilt_x_mm_per_mm_z,
            cam_tilt_y_mm_per_mm_z=config.cam_tilt_y_mm_per_mm_z,
        )
        self.planner = WorldCatalogAndPlanner(
            dedup_radius_mm=config.deduplication_radius_mm,
            block_height_mm=config.block_height_mm,
        )
        self.vision = VisionPipeline(
            config=config,
            kinematics=self.kinematics,
            mock_hardware=mock_hardware,
        )
        self._load_servo_trim()
        self.serial = ESP32SerialBridge(
            config=config,
            mock_hardware=mock_hardware,
            pose_update_callback=self._on_pose_updated,
            event_callback=self._on_hardware_event,
            command_log_callback=self.reports.record_motor_command,
        )
        self.telemetry = TelemetryServer(
            host=ws_host,
            port=ws_port,
            command_callback=self._on_user_command,
            report_recorder=self.reports,
        )

        self._start_requested = False
        self._pause_requested = False
        self._estop_active = False
        self._running = True
        self._live_stream_enabled = True
        self._live_cv_enabled = False  # Eco stream mode by default in IDLE to save ~70% CPU on Pi Zero
        self._current_mass_g: Optional[float] = None
        self._load_cell_error: Optional[str] = None
        self._load_cell_seq: int = 0
        self._held_block_id: Optional[int] = None
        self._cmd_queue: queue.Queue = queue.Queue()
        self._cmd_thread: Optional[threading.Thread] = None
        self._latest_live_fiducials = []
        self._latest_live_blocks = []
        self._worker_thread: Optional[threading.Thread] = None
        self._live_vision_thread: Optional[threading.Thread] = None

        # Xbox Manual Mode & Teleop State
        self._homed_known = False
        self._manual_interrupt_requested = False
        self._manual_lock = threading.Lock()
        self._teleop_cycle_lock = threading.Lock()
        self._gamepad_mapping = load_gamepad_mapping(self._gamepad_mapping_path())
        self._gamepad_reader = XboxGamepadReader(
            on_connect=self._on_gamepad_connected,
            on_disconnect=self._on_gamepad_disconnected,
        )
        self._bt_manager = XboxBluetoothManager(on_status_change=self._on_bt_status_change)
        self._teleop_mapper = TeleopInputMapper()
        self._browser_gp_snapshot: Optional[GamepadSnapshot] = None
        self._browser_gp_prev_buttons: dict[str, bool] = {}
        self._browser_gp_device_name: str = ""
        self._browser_last_button: str = ""
        self._browser_last_button_seq: int = 0
        self._rumble_seq: int = 0
        self._rumble_payload: Optional[dict] = None
        self._last_auto_cycle_s: Optional[float] = None
        self._last_auto_blocks: int = 0
        self._manual_timer = {
            "running": False,
            "elapsed_s": 0.0,
            "started_mono": None,
            "blocks_placed": 0,
            "best_run_s": None,
            "runs": [],
        }
        self._teleop_thread: Optional[threading.Thread] = None

    def _gamepad_mapping_path(self) -> Path:
        return Path(__file__).resolve().parent / "gamepad_mapping.json"

    def _servo_trim_path(self) -> Path:
        return Path(__file__).resolve().parent / "servo_trim.json"

    def _load_servo_trim(self) -> None:
        path = self._servo_trim_path()
        try:
            if path.is_file():
                data = json.loads(path.read_text(encoding="utf-8"))
                val = float(data.get("servo_zero_offset_deg", self.config.servo_zero_offset_deg))
                self.config.servo_zero_offset_deg = round(max(-45.0, min(45.0, val)), 1)
        except Exception as exc:
            logger.warning("Failed to load servo trim from %s: %s", path, exc)

    def _save_servo_trim(self, offset_deg: float) -> None:
        path = self._servo_trim_path()
        try:
            path.write_text(
                json.dumps({"servo_zero_offset_deg": round(float(offset_deg), 1)}, indent=2) + "\n",
                encoding="utf-8",
            )
        except Exception as exc:
            logger.warning("Failed to save servo trim to %s: %s", path, exc)

    def log_event(self, message: str) -> None:
        timestamped = f"[{time.strftime('%H:%M:%S')}] {message}"
        logger.info(message)
        self._logs.append(timestamped)
        self.reports.record_event_log(timestamped)
        self.publish_telemetry()

    def start(self, auto_start_scan: bool = False) -> None:
        self.telemetry.start()
        self.vision.open_camera()
        self.serial.connect()

        if self.serial.is_connected:
            try:
                self.serial.play_chime("WIN95")
                self.log_event("Played boot chime (Windows 95 startup theme).")
            except Exception as exc:
                logger.debug("Boot startup chime skipped: %s", exc)

        # Render initial stationary preview frame in IDLE
        frame = self.vision.capture_stationary_frame(
            self.serial.pose.x_mm, self.serial.pose.y_mm, self.serial.pose.z_mm
        )
        self.vision.detect_blocks_hsv(frame)
        self.log_event(
            f"System initialized in {'MOCK' if self.mock_hardware else 'HARDWARE'} mode. State: IDLE."
        )

        if auto_start_scan:
            self._start_requested = True

        self._worker_thread = threading.Thread(target=self._MainLoop, daemon=True)
        self._worker_thread.start()

        self._cmd_thread = threading.Thread(target=self._command_worker_loop, daemon=True)
        self._cmd_thread.start()

        self._live_vision_thread = threading.Thread(target=self._live_vision_loop, daemon=True)
        self._live_vision_thread.start()

        self._gamepad_reader.start()
        self._teleop_thread = threading.Thread(target=self._teleop_loop, name="teleop-loop", daemon=True)
        self._teleop_thread.start()

    def stop(self) -> None:
        self._running = False
        self._gamepad_reader.stop()
        if self.state == "MANUAL":
            try:
                self.serial.stop_velocity()
                self.serial.set_gripper(closed=True, nowait=True)
            except Exception:
                pass
        self.vision.close_camera()
        self.serial.close()

    def publish_telemetry(self) -> None:
        mass_val = self._current_mass_g
        mass_cls = self.config.classify_mass(mass_val) if mass_val is not None else None
        payload = {
            "timestamp": round(time.time(), 2),
            "state": self.state,
            "gantry_pose": self.serial.pose.to_dict(),
            "current_mass_g": mass_val,
            "mass_class": mass_cls,
            "load_cell_error": self._load_cell_error,
            "load_cell_seq": self._load_cell_seq,
            "held_block_id": self._held_block_id,
            "steps_per_mm": {
                "x": round(self.config.steps_per_mm_x, 2),
                "y": round(self.config.steps_per_mm_y, 2),
                "z": round(self.config.steps_per_mm_z, 2),
            },
            "calibration": {
                "calibrated": self.kinematics.calibrated,
                "origin_calibrated": self.kinematics.origin_calibrated,
                "calibration_mode": self.kinematics.calibration_mode,
                "markers_detected": list(self.kinematics.markers_detected),
                "cam_origin_x_mm": round(self.kinematics.cam_origin_x_mm, 2),
                "cam_origin_y_mm": round(self.kinematics.cam_origin_y_mm, 2),
                "offset_x_mm": round(self.kinematics.offset_x_mm, 2),
                "offset_y_mm": round(self.kinematics.offset_y_mm, 2),
                "cam_skew_deg": round(self.kinematics.cam_skew_deg, 2),
                "cam_motor_skew_deg": round(self.kinematics.cam_motor_skew_deg, 2),
                "plate_yaw_deg": round(self.kinematics.plate_yaw_deg, 3),
                "plate_scale_x": round(self.kinematics.plate_scale_x, 4),
                "plate_scale_y": round(self.kinematics.plate_scale_y, 4),
                "residual_rms_mm": self.kinematics.residual_rms_mm,
                "residual_max_mm": self.kinematics.residual_max_mm,
                "corner_residuals_mm": {str(k): v for k, v in self.kinematics.corner_residuals_mm.items()},
                "plate_transform": self.kinematics.plate_transform_dict(),
                "kx_mm_per_px": round(self.kinematics.kx_mm_per_px, 4),
                "ky_mm_per_px": round(self.kinematics.ky_mm_per_px, 4),
                "servo_zero_offset_deg": round(self.config.servo_zero_offset_deg, 1),
            },
            "blocks": [b.to_dict() for b in self.planner.blocks],
            "live_fiducials": list(self._latest_live_fiducials),
            "live_blocks": list(self._latest_live_blocks),
            "live_cv_enabled": self._live_cv_enabled,
            "logs": list(self._logs),
            "manual_mode": self._manual_mode_telemetry(),
            "vision_frame_b64": self.vision.last_annotated_jpeg_b64,
            "vision_frame_seq": getattr(self.vision, "frame_seq", 0),
        }
        self.telemetry.broadcast_state(payload)

    def _on_pose_updated(self, pose: GantryPose) -> None:
        self.publish_telemetry()

    def _set_mass_reading(self, mass_g: Optional[float], error: Optional[str] = None) -> None:
        self._current_mass_g = mass_g
        self._load_cell_error = error
        self._load_cell_seq += 1

    def _drain_cmd_queue(self) -> None:
        while not self._cmd_queue.empty():
            try:
                self._cmd_queue.get_nowait()
                self._cmd_queue.task_done()
            except queue.Empty:
                break

    def _activate_emergency_stop(self, reason: str, notify_esp32: bool) -> None:
        self._estop_active = True
        self._start_requested = False
        self._manual_interrupt_requested = False
        self._homed_known = False
        self._held_block_id = None
        self._stop_manual_timer()
        self.state = "ESTOPPED"
        if notify_esp32:
            self.serial.trigger_emergency_stop()
        self.log_event(reason)
        self._drain_cmd_queue()
        self.publish_telemetry()

    def _on_hardware_event(self, event_name: str) -> None:
        if event_name == "START_SCAN":
            self._on_user_command("START_SCAN", {"source": "hardware"})
        elif event_name == "EMERGENCY_STOP":
            self._activate_emergency_stop(
                "Hardware E-STOP switch triggered on ESP32! All steppers disabled.",
                notify_esp32=False,
            )
        elif event_name == "CHIME_BTN":
            self.log_event("Hardware CHIME switch pressed on ESP32.")
        elif event_name == "TOGGLE_MANUAL_MODE":
            self._on_user_command("TOGGLE_MANUAL_MODE", {"source": "hardware"})

    def _on_user_command(self, cmd: str, data: dict) -> None:
        """
        Fast non-blocking command receiver called by the WebSocket server thread.
        ESTOP and RESET unblock any waiting commands and drain stale queues immediately;
        all other motion and configuration commands are queued sequentially.
        """
        if cmd == "TELEOP_BROWSER_INPUT":
            raw_btns = data.get("buttons") or {}
            btns = {str(k).upper(): bool(v) for k, v in raw_btns.items()}
            snap = GamepadSnapshot(
                lx=max(-1.0, min(1.0, float(data.get("lx", 0.0)))),
                ly=max(-1.0, min(1.0, float(data.get("ly", 0.0)))),
                rx=max(-1.0, min(1.0, float(data.get("rx", 0.0)))),
                ry=max(-1.0, min(1.0, float(data.get("ry", 0.0)))),
                lt=max(0.0, min(1.0, float(data.get("lt", 0.0)))),
                rt=max(0.0, min(1.0, float(data.get("rt", 0.0)))),
                dpad_x=max(-1.0, min(1.0, float(data.get("dpad_x", 0.0)))),
                dpad_y=max(-1.0, min(1.0, float(data.get("dpad_y", 0.0)))),
                buttons=btns,
                timestamp_mono=time.monotonic(),
                source="browser",
            )
            with self._manual_lock:
                self._browser_gp_snapshot = snap
                self._browser_gp_device_name = str(data.get("device_name", "Browser Gamepad"))[:48]
                for b_name, is_down in btns.items():
                    if is_down and not self._browser_gp_prev_buttons.get(b_name, False):
                        self._browser_last_button = b_name
                        self._browser_last_button_seq += 1
                self._browser_gp_prev_buttons = btns
            return

        if cmd == "EMERGENCY_STOP":
            self._activate_emergency_stop(
                "EMERGENCY STOP triggered! All steppers disabled.",
                notify_esp32=True,
            )
            return

        if cmd == "RESET":
            self._pause_requested = False
            self._start_requested = False
            self.serial.trigger_emergency_stop()
            self._drain_cmd_queue()
            self._cmd_queue.put((cmd, data))
            return

        if cmd in ("WEIGH_BLOCK", "SAMPLE_MASS", "TARE_SCALE"):
            if self.state in ("CALIBRATING", "SCANNING", "PLANNING", "PICKING", "WEIGHING", "STACKING"):
                self.log_event(f"Ignored manual {cmd} while cycle is active ({self.state}).")
                self._set_mass_reading(self._current_mass_g, f"Busy ({self.state})")
                self.publish_telemetry()
                return

        if cmd in ("WEIGH_BLOCK", "SAMPLE_MASS", "TARE_SCALE", "HOME_ALL", "START_SCAN", "TOGGLE_MANUAL_MODE", "BT_RECONNECT", "BT_PAIR_XBOX"):
            with self._cmd_queue.mutex:
                if any(queued_cmd == cmd for queued_cmd, _ in self._cmd_queue.queue):
                    return

        self._cmd_queue.put((cmd, data))

    def _command_worker_loop(self) -> None:
        """Dedicated background thread processing user commands sequentially without blocking networking."""
        while self._running:
            try:
                cmd, data = self._cmd_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                self._dispatch_command(cmd, data)
            except Exception as exc:
                logger.error("Error executing queued command %s: %s", cmd, exc)
            finally:
                self._cmd_queue.task_done()

    def _dispatch_command(self, cmd: str, data: dict) -> None:
        if cmd == "START_SCAN":
            if self.state in ("IDLE", "COMPLETE", "ERROR", "ESTOPPED"):
                self._estop_active = False
                self._pause_requested = False
                self.serial.clear_emergency_stop()
                if self.mock_hardware:
                    self.vision.reset_simulated_course()
                self.planner.clear()
                self._start_requested = True
                source = "hardware START switch (ESP32)" if data.get("source") == "hardware" else "Tier 1 Dashboard"
                self.log_event(f"Received START_SCAN trigger from {source}.")
        elif cmd == "PAUSE":
            self._pause_requested = True
            self.log_event("System PAUSED by user.")
            self.publish_telemetry()
        elif cmd == "RESUME":
            self._pause_requested = False
            self.log_event("System RESUMED by user.")
            self.publish_telemetry()
        elif cmd == "RESET":
            self._estop_active = False
            self._pause_requested = False
            self._start_requested = False
            self.serial.clear_emergency_stop()
            self.serial.ensure_awake(force=True)
            if self.mock_hardware:
                self.vision.reset_simulated_course()
            self.planner.clear()
            self._held_block_id = None
            self._set_mass_reading(None, None)
            self.state = "IDLE"
            self.log_event("System reset to IDLE and motors re-energized.")
        elif cmd == "HOME_ALL":
            if self.state in ("IDLE", "COMPLETE", "ERROR", "ESTOPPED"):
                self._estop_active = False
                self._held_block_id = None
                self.serial.clear_emergency_stop()
                self.state = "CALIBRATING"
                self.publish_telemetry()
                self.log_event("Supervisor manual HOME_ALL triggered for Z, X, Y...")
                try:
                    self.serial.ensure_awake()
                    self.serial.home_all()
                    # Step off mechanical Z=0/Y=0 stops before auto-taring, then park cleanly at (0, 0, 0)
                    self.serial.move_to(
                        y_mm=self.config.servo_safe_y_mm,
                        z_mm=10.0,
                        angle_deg=0.0,
                        feedrate_mm_s=self.config.feedrate_z_mm_s,
                    )
                    self._auto_tare_scale("HOME_ALL")
                    self.serial.move_to(z_mm=0.0, feedrate_mm_s=self.config.feedrate_z_mm_s)
                    self.serial.move_to(y_mm=0.0, feedrate_mm_s=self.config.feedrate_travel_mm_s)
                    self._homed_known = True
                    self.state = "IDLE"
                    self.log_event("Gantry successfully homed to (0.0, 0.0, 0.0). Motors ready in IDLE.")
                except Exception as exc:
                    self.state = "ERROR"
                    self.log_event(f"Manual homing failed: {exc}")
                self.publish_telemetry()
            else:
                self.log_event(f"Cannot home while state is {self.state}. Pause or Stop first.")
        elif cmd == "JOG_AXIS":
            if self.state in ("IDLE", "PAUSED", "COMPLETE", "ERROR"):
                axis = str(data.get("axis", "")).upper()
                dist_mm = float(data.get("dist_mm", 0.0))
                with self._cmd_queue.mutex:
                    while self._cmd_queue.queue:
                        nxt_cmd, nxt_data = self._cmd_queue.queue[0]
                        if nxt_cmd != "JOG_AXIS" or str(nxt_data.get("axis", "")).upper() != axis:
                            break
                        self._cmd_queue.queue.popleft()
                        self._cmd_queue.unfinished_tasks = max(
                            0, self._cmd_queue.unfinished_tasks - 1
                        )
                        dist_mm += float(nxt_data.get("dist_mm", 0.0))
                if axis in ("X", "Y", "Z") and dist_mm != 0.0:
                    try:
                        self.serial.ensure_awake()
                        cur_x = self.serial.pose.x_mm
                        cur_y = self.serial.pose.y_mm
                        cur_z = self.serial.pose.z_mm
                        if axis == "X":
                            target_x = max(0.0, min(self.config.travel_x_mm, cur_x + dist_mm))
                            self.serial.move_to(x_mm=target_x)
                        elif axis == "Y":
                            target_y = max(0.0, min(self.config.travel_y_mm, cur_y + dist_mm))
                            self.serial.move_to(y_mm=target_y)
                        elif axis == "Z":
                            target_z = max(0.0, min(self.config.travel_z_bottom_mm, cur_z + dist_mm))
                            self.serial.move_to(z_mm=target_z)
                        self.log_event(f"Manual jog {axis} {dist_mm:+.1f} mm -> ({self.serial.pose.x_mm:.1f}, {self.serial.pose.y_mm:.1f}, {self.serial.pose.z_mm:.1f})")
                        self.publish_telemetry()
                    except Exception as exc:
                        self.log_event(f"Manual jog {axis} failed: {exc}")
        elif cmd == "SET_STEPS_CALIBRATION":
            steps_x = float(data.get("steps_per_mm_x", self.config.steps_per_mm_x))
            steps_y = float(data.get("steps_per_mm_y", self.config.steps_per_mm_y))
            steps_z = float(data.get("steps_per_mm_z", self.config.steps_per_mm_z))
            self.config.steps_per_mm_x = steps_x
            self.config.steps_per_mm_y = steps_y
            self.config.steps_per_mm_z = steps_z
            try:
                self.serial.ensure_awake()
                self.serial.send_command(f"STEPS X{steps_x:.2f} Y{steps_y:.2f} Z{steps_z:.2f}")
                self.log_event(f"Calibration updated: X={steps_x:.1f}, Y={steps_y:.1f}, Z={steps_z:.1f} steps/mm")
            except Exception as exc:
                self.log_event(f"Update steps/mm failed: {exc}")
            self.publish_telemetry()
        elif cmd == "SET_SERVO_OFFSET":
            if self.state in ("IDLE", "PAUSED", "COMPLETE", "ERROR"):
                try:
                    if data.get("delta_deg") is not None:
                        target = self.config.servo_zero_offset_deg + float(data["delta_deg"])
                    elif data.get("offset_deg") is not None:
                        target = float(data["offset_deg"])
                    else:
                        target = self.config.servo_zero_offset_deg
                    target = round(max(-45.0, min(45.0, target)), 1)
                    applied = self.serial.set_servo_zero_offset(target)
                    self._save_servo_trim(applied)
                    self.log_event(f"Servo neutral trim set to {applied:+.1f}° (saved).")
                except Exception as exc:
                    self.log_event(f"Update servo trim failed: {exc}")
                self.publish_telemetry()
            else:
                self.log_event(f"Cannot adjust servo trim while state is {self.state}.")
        elif cmd in ("FREE_MOTORS", "SLEEP_MOTORS"):
            try:
                if self.state == "MANUAL":
                    self._exit_manual_mode("sleep")
                self._estop_active = False
                self._homed_known = False
                self.serial.clear_emergency_stop()
                self.serial.send_command("SLEEP")
                if self.state == "ESTOPPED":
                    self.state = "IDLE"
                self.log_event("Motors unpowered (FREE). Gantry can be moved freely by hand.")
            except Exception as exc:
                self.log_event(f"Free motors failed: {exc}")
            self.publish_telemetry()
        elif cmd in ("WAKE_MOTORS", "SETUP_MOTORS"):
            try:
                self._estop_active = False
                self.serial.clear_emergency_stop()
                self.serial.send_command("SETUP")
                self.serial._drivers_awake = True
                if self.state == "ESTOPPED":
                    self.state = "IDLE"
                self.log_event("Motors re-energized (SETUP). Drivers ready.")
            except Exception as exc:
                self.log_event(f"Wake motors failed: {exc}")
            self.publish_telemetry()
        elif cmd == "TOGGLE_LIVE_STREAM":
            self._live_stream_enabled = not self._live_stream_enabled
            status_str = "ACTIVE" if self._live_stream_enabled else "PAUSED"
            self.log_event(f"Live camera stream {status_str}.")
            self.publish_telemetry()
        elif cmd == "TOGGLE_LIVE_CV":
            self._live_cv_enabled = not self._live_cv_enabled
            status_str = "FULL (ArUco + HSV Detection)" if self._live_cv_enabled else "ECO (Fast Stream)"
            self.log_event(f"Live CV processing mode set to: {status_str}.")
            self.publish_telemetry()
        elif cmd == "TARE_SCALE":
            try:
                self.serial.tare_scale()
                self._set_mass_reading(0.00, None)
                self.log_event("HX711 scale tared (zeroed) successfully.")
            except Exception as exc:
                self._set_mass_reading(None, str(exc))
                self.log_event(f"Tare scale failed: {exc}")
            self.publish_telemetry()
        elif cmd in ("WEIGH_BLOCK", "SAMPLE_MASS"):
            try:
                measured = self.serial.weigh_block()
                self._set_mass_reading(round(measured, 2), None)
                cls = self.config.classify_mass(measured)
                self.log_event(f"HX711 sample: {measured:.2f} g -> Class [{cls.upper()}].")
            except Exception as exc:
                self._set_mass_reading(None, str(exc))
                self.log_event(f"Weigh sample failed: {exc}")
            self.publish_telemetry()
        elif cmd == "RAW_SERIAL":
            raw_line = str(data.get("line", "")).strip()
            # Allow safe control/query commands even in ESTOPPED
            upper_line = raw_line.upper()
            allowed = self.state in ("IDLE", "COMPLETE", "ERROR") or upper_line in ("SETUP", "STATUS", "SLEEP", "ESTOP", "PING")
            if raw_line and allowed:
                try:
                    replies = self.serial.send_command(raw_line)
                    self.log_event(f"Manual TX '{raw_line}' -> OK ({', '.join(replies) if replies else 'done'})")
                except Exception as exc:
                    self.log_event(f"Manual TX '{raw_line}' failed: {exc}")
        elif cmd == "PLAY_CHIME":
            if self.state in ("IDLE", "COMPLETE", "ERROR"):
                tune = str(data.get("tune", "TADA")).strip() or "TADA"
                try:
                    self.serial.ensure_awake()
                    self.serial.play_chime(tune)
                    self.log_event(f"Played victory chime ({tune}).")
                except Exception as exc:
                    self.log_event(f"Victory chime failed: {exc}")
            else:
                self.log_event(f"Cannot play chime while state is {self.state}.")
        elif cmd == "TOGGLE_MANUAL_MODE":
            source = str(data.get("source", "dashboard"))
            if self.state == "MANUAL":
                self._exit_manual_mode(source)
            elif self.state in ("CALIBRATING", "SCANNING", "PLANNING", "PICKING", "WEIGHING", "STACKING", "PAUSED"):
                self._manual_interrupt_requested = True
                self._pause_requested = False
                self.log_event(f"Mode toggle ({source}) requested — handing off active cycle to MANUAL mode...")
            else:
                self._enter_manual_mode(source)
        elif cmd == "SET_MODE":
            target = str(data.get("mode", "")).upper().strip()
            source = str(data.get("source", "dashboard"))
            if target == "MANUAL" and self.state != "MANUAL":
                if self.state in ("CALIBRATING", "SCANNING", "PLANNING", "PICKING", "WEIGHING", "STACKING", "PAUSED"):
                    self._manual_interrupt_requested = True
                    self._pause_requested = False
                    self.log_event(f"Mode switch to MANUAL ({source}) requested — handing off active cycle...")
                else:
                    self._enter_manual_mode(source)
            elif target in ("AUTO", "IDLE") and self.state == "MANUAL":
                self._exit_manual_mode(source)
        elif cmd == "SET_GAMEPAD_MAPPING":
            raw_map = data.get("mapping")
            if isinstance(raw_map, dict):
                with self._manual_lock:
                    merged = dict(self._gamepad_mapping)
                    for k, v in raw_map.items():
                        if k in ("axes", "buttons") and isinstance(v, dict) and isinstance(merged.get(k), dict):
                            sub = dict(merged[k])  # type: ignore[arg-type]
                            sub.update(v)
                            merged[k] = sub
                        else:
                            merged[k] = v
                    self._gamepad_mapping = save_gamepad_mapping(self._gamepad_mapping_path(), merged)
                spd_pct = int(round(float(self._gamepad_mapping.get("speed_scale", 1.0)) * 100))
                self.log_event(
                    f"Saved Xbox mapping (preset={self._gamepad_mapping['axis_preset']}, "
                    f"speed={spd_pct}%, "
                    f"telemetry={self._gamepad_mapping['telemetry_mode']}, "
                    f"z_guard={'ON' if self._gamepad_mapping['z_guard_enabled'] else 'OFF'})."
                )
                self.publish_telemetry()
        elif cmd == "RESET_GAMEPAD_MAPPING":
            with self._manual_lock:
                self._gamepad_mapping = save_gamepad_mapping(
                    self._gamepad_mapping_path(), dict(DEFAULT_GAMEPAD_MAPPING)
                )
            self.log_event("Reset Xbox controller mapping to defaults.")
            self.publish_telemetry()
        elif cmd == "MANUAL_TIMER_ACTION":
            self._handle_manual_timer_action(str(data.get("action", "")).lower().strip())
            self.publish_telemetry()
        elif cmd == "BT_RECONNECT":
            self._bt_manager.reconnect_saved()
            self.publish_telemetry()
        elif cmd == "BT_PAIR_XBOX":
            self._bt_manager.pair_new_xbox()
            self.publish_telemetry()

    def _check_pause_or_abort(self) -> None:
        if self._manual_interrupt_requested:
            raise _ManualModeInterrupt()
        if self._pause_requested and not self._estop_active and self._running:
            saved_state = self.state
            self.state = "PAUSED"
            self.publish_telemetry()
            while self._pause_requested and not self._estop_active and self._running and not self._manual_interrupt_requested:
                time.sleep(0.15)
            if self._manual_interrupt_requested:
                raise _ManualModeInterrupt()
            if not self._estop_active and self._running:
                self.state = saved_state
                self.publish_telemetry()

        if self._estop_active or not self._running:
            raise RuntimeError("ERR:EMERGENCY_STOP_ACTIVE")

    def _live_vision_loop(self) -> None:
        """
        Background loop providing semi-live (~2.2 FPS) video feed to the Tier 1 Dashboard
        whenever a client is connected and the robot is idle/manual/paused/errored.
        Pauses during automated cycles, pending user commands, or when no dashboard client is viewing.
        """
        last_frame_time = 0.0
        target_period_s = 0.45  # ~2.2 FPS - leaves CPU headroom for responsive jog/UI updates
        while self._running:
            can_stream = (
                self._live_stream_enabled
                and not self._start_requested
                and self.telemetry.client_count > 0
                and self._cmd_queue.empty()
                and self.state in ("IDLE", "PAUSED", "COMPLETE", "ERROR", "ESTOPPED")
            )
            if not can_stream:
                time.sleep(0.12)
                continue

            now = time.time()
            elapsed = now - last_frame_time
            if elapsed < target_period_s:
                time.sleep(max(0.02, target_period_s - elapsed))

            last_frame_time = time.time()
            try:
                frame = self.vision.capture_live_frame(
                    self.serial.pose.x_mm,
                    self.serial.pose.y_mm,
                    self.serial.pose.z_mm,
                )
                if frame is not None:
                    if self._live_cv_enabled:
                        fids, blocks = self.vision.process_live_frame(frame)
                        self._latest_live_fiducials = fids
                        self._latest_live_blocks = [b.to_dict() for b in blocks]
                    else:
                        # Eco Preview: fast crosshair + JPEG, zero heavy HSV morphology or ArUco loops
                        self.vision.encode_fast_preview(frame)
                        self._latest_live_fiducials = []
                        self._latest_live_blocks = []
                    self.publish_telemetry()
            except Exception as e:
                logger.debug("Live vision capture loop: %s", e)
                time.sleep(0.4)

    # ------------------------------------------------------------------
    # Main 6-Phase State Machine Loop
    # ------------------------------------------------------------------
    def _MainLoop(self) -> None:
        while self._running:
            if not self._start_requested or self._estop_active:
                time.sleep(0.1)
                continue

            self._start_requested = False
            self._manual_interrupt_requested = False
            try:
                self.reports.start_new_run(mock_hardware=self.mock_hardware)
                cycle_t0 = time.monotonic()
                self._run_full_cycle()
                self._last_auto_cycle_s = round(time.monotonic() - cycle_t0, 2)
                self._last_auto_blocks = sum(1 for b in self.planner.blocks if b.status == "STACKED")
                self.reports.finish_run(status="COMPLETED")
                self.publish_telemetry()
            except _ManualModeInterrupt:
                self._manual_interrupt_requested = False
                self.reports.finish_run(status="SWITCHED_TO_MANUAL")
                self._enter_manual_mode(source="cycle_interrupt")
            except Exception as exc:
                try:
                    if not self.serial.pose.grip_closed:
                        self.serial.set_gripper(closed=True)
                except Exception:
                    pass
                exc_str = str(exc)
                if not self._estop_active:
                    self.state = "ERROR"
                    self.reports.finish_run(status="ERROR", error=exc_str)
                    if "CRASH_DETECTED" in exc_str or "STALL" in exc_str:
                        self.kinematics.calibrated = False
                        self.log_event(f"CRASH / STALL DETECTED on end-stop! {exc_str}. Motors powered down safely. Re-home before continuing.")
                    else:
                        self.log_event(f"Workflow aborted: {exc}")
                else:
                    self.reports.finish_run(status="ESTOPPED", error="EMERGENCY_STOP_TRIGGERED")

    def _auto_tare_scale(
        self,
        context_label: str,
        settle_s: Optional[float] = None,
        retries: int = 2,
    ) -> bool:
        """Waits for structural settling and automatically zeroes the HX711 scale with retries."""
        eff_settle = self.config.tare_settle_s if settle_s is None else float(settle_s)
        if not self.mock_hardware and eff_settle > 0.0:
            time.sleep(eff_settle)

        attempts = max(1, int(retries) + 1)
        last_exc: Optional[Exception] = None
        for attempt in range(1, attempts + 1):
            try:
                self.serial.tare_scale()
                self._set_mass_reading(0.00, None)
                chk = self.serial.last_tare_check_g
                chk_str = f", check={chk:+.2f} g" if chk is not None else ""
                self.log_event(f"{context_label}: HX711 scale auto-tared (0.00 g{chk_str}).")
                return True
            except Exception as exc:
                last_exc = exc
                logger.warning("%s auto-tare attempt %d/%d failed: %s", context_label, attempt, attempts, exc)
                if attempt < attempts and not self.mock_hardware:
                    time.sleep(0.30)

        self.log_event(f"Notice: {context_label} auto-tare failed after {attempts} attempts ({last_exc}).")
        return False

    def _run_full_cycle(self) -> None:
        # ==============================================================
        # PHASE 2: Homing & 4-ArUco Ground-Truth Calibration (CALIBRATING)
        # ==============================================================
        self.state = "CALIBRATING"
        self._held_block_id = None
        self.log_event("Phase 2 [CALIBRATING]: Homing Z, X, Y axes via TMC2209 StallGuard4...")
        self._check_pause_or_abort()
        self.serial.home_all()
        self._homed_known = True
        # Move clear of Z=0 and Y=0 mechanical stops so bumper preload never corrupts initial auto-tare
        self.serial.move_to(
            y_mm=self.config.servo_safe_y_mm,
            z_mm=10.0,
            angle_deg=0.0,
            feedrate_mm_s=self.config.feedrate_z_mm_s,
        )
        self._auto_tare_scale("Phase 2 [CALIBRATING]")

        # Reset kinematic plate transform prior to fresh calibration
        self.kinematics.cam_origin_x_mm = self.config.nominal_cam_origin_x_mm
        self.kinematics.cam_origin_y_mm = self.config.nominal_cam_origin_y_mm
        self.kinematics.cam_skew_deg = self.config.default_cam_skew_deg
        self.kinematics.cam_motor_skew_deg = self.config.default_cam_skew_deg
        self.kinematics.plate_yaw_deg = 0.0
        self.kinematics.drift_x_mm = 0.0
        self.kinematics.drift_y_mm = 0.0
        self.kinematics._plate_A = None
        self.kinematics._plate_t = None
        self.kinematics._plate_A_inv = None
        self.kinematics._plate_t_inv = None
        self.kinematics.origin_calibrated = False
        self.kinematics.calibrated = False

        z_scan = self.config.z_scan_mm
        corner_wps = dict(getattr(self.config, "calibration_corner_waypoints_mm", {}))
        if not corner_wps:
            corner_wps = {1: (self.config.fiducial_base_x_mm, self.config.fiducial_base_y_mm)}

        wp_items = list(corner_wps.items())
        first_id, (fx, fy) = wp_items[0]

        # 2A. Visit primary corner fiducial & take 2-frame step for pixel scale (Kx, Ky)
        self.log_event(
            f"Visiting Corner ArUco #{first_id} at motor ({fx:.1f}, {fy:.1f}, {z_scan:.1f}) mm..."
        )
        self.serial.move_to(x_mm=fx, y_mm=fy, z_mm=z_scan, angle_deg=0.0)
        self._check_pause_or_abort()

        observations = []  # (marker_id, u_px, v_px, motor_x, motor_y, angle_deg)
        frame_a = self.vision.capture_stationary_frame(
            self.serial.pose.x_mm, self.serial.pose.y_mm, self.serial.pose.z_mm
        )
        fids_a = self.vision.detect_all_aruco_fiducials(frame_a)
        for m_id, u_px, v_px, ang in fids_a:
            if m_id in self.kinematics.corner_fiducials:
                observations.append((m_id, u_px, v_px, fx, fy, ang))

        step_x = self.config.calibration_step_x_mm
        step_y = self.config.calibration_step_y_mm
        self.serial.move_to(x_mm=fx + step_x, y_mm=fy + step_y, z_mm=z_scan)
        self._check_pause_or_abort()

        frame_b = self.vision.capture_stationary_frame(
            self.serial.pose.x_mm, self.serial.pose.y_mm, self.serial.pose.z_mm
        )
        fids_b = self.vision.detect_all_aruco_fiducials(frame_b)
        for m_id, u_px, v_px, ang in fids_b:
            if m_id in self.kinematics.corner_fiducials:
                observations.append((m_id, u_px, v_px, fx + step_x, fy + step_y, ang))

        # Match a marker seen in both Frame A and Frame B for local pixel scale + camera-to-motor twist
        step_obs = None
        fids_a_map = {m_id: (u_px, v_px, ang) for m_id, u_px, v_px, ang in fids_a}
        fids_b_map = {m_id: (u_px, v_px, ang) for m_id, u_px, v_px, ang in fids_b}
        common_ids = [mid for mid in fids_a_map if mid in fids_b_map and mid in self.kinematics.corner_fiducials]
        primary_fid = None
        if common_ids:
            scale_id = first_id if first_id in common_ids else common_ids[0]
            u1, v1, _a1 = fids_a_map[scale_id]
            u2, v2, a2 = fids_b_map[scale_id]
            primary_fid = (scale_id, u2, v2, a2)
            try:
                kx, ky = self.kinematics.compute_auto_scale(u1, v1, u2, v2, step_x, step_y)
                step_obs = (u1, v1, fx, fy, u2, v2, fx + step_x, fy + step_y)
                self.log_event(
                    f"Auto-Scale calibrated on Marker #{scale_id}: Kx={kx:.4f} mm/px, Ky={ky:.4f} mm/px at Z={z_scan:.0f} mm."
                )
            except Exception as exc:
                self.log_event(f"Auto-Scale step failed ({exc}); keeping Kx={self.kinematics.kx_mm_per_px:.4f} mm/px.")

        # 2B. Visit the remaining 3 corner ArUco markers to cross-reference full course geometry
        for corner_id, (cx_m, cy_m) in wp_items[1:]:
            self._check_pause_or_abort()
            self.log_event(
                f"Scanning Corner ArUco #{corner_id} at motor ({cx_m:.1f}, {cy_m:.1f}, {z_scan:.1f}) mm..."
            )
            self.serial.move_to(x_mm=cx_m, y_mm=cy_m, z_mm=z_scan, angle_deg=0.0)
            self._check_pause_or_abort()
            c_frame = self.vision.capture_stationary_frame(
                self.serial.pose.x_mm, self.serial.pose.y_mm, self.serial.pose.z_mm
            )
            c_fids = self.vision.detect_all_aruco_fiducials(c_frame)
            if not any(m_id == corner_id for m_id, *_ in c_fids):
                c_frame = self.vision.capture_stationary_frame(
                    self.serial.pose.x_mm, self.serial.pose.y_mm, self.serial.pose.z_mm
                )
                c_fids = self.vision.detect_all_aruco_fiducials(c_frame)
            for m_id, u_px, v_px, ang in c_fids:
                if m_id in self.kinematics.corner_fiducials:
                    observations.append((m_id, u_px, v_px, cx_m, cy_m, ang))
                    if primary_fid is None:
                        primary_fid = (m_id, u_px, v_px, ang)

        # 2C. Solve unified ground-truth Plate <-> Motor transform across all observed markers
        if observations:
            try:
                cal_res = self.kinematics.calibrate_from_fiducials(
                    observations, step_observation=step_obs
                )
                detected_ids = cal_res["markers_detected"]
                self.log_event(
                    f"Ground-Truth Calibration [{cal_res['mode']}] using ArUcos {detected_ids}: "
                    f"Cam origin=({cal_res['cam_origin_x_mm']:.2f}, {cal_res['cam_origin_y_mm']:.2f}) mm, "
                    f"Plate yaw={cal_res['plate_yaw_deg']:+.2f}°, Cam skew={cal_res['cam_skew_deg']:+.2f}°, "
                    f"RMS err={cal_res['residual_rms_mm']:.3f} mm (max {cal_res['residual_max_mm']:.3f} mm)."
                )
                self.reports.record_calibration(
                    calibrated=True,
                    origin_calibrated=True,
                    marker_id=primary_fid[0] if primary_fid else detected_ids[0],
                    u_px=primary_fid[1] if primary_fid else None,
                    v_px=primary_fid[2] if primary_fid else None,
                    marker_angle_deg=primary_fid[3] if primary_fid else None,
                    cam_skew_deg=self.kinematics.cam_skew_deg,
                    kx=self.kinematics.kx_mm_per_px,
                    ky=self.kinematics.ky_mm_per_px,
                    cam_origin_x=self.kinematics.cam_origin_x_mm,
                    cam_origin_y=self.kinematics.cam_origin_y_mm,
                    offset_x=self.kinematics.offset_x_mm,
                    offset_y=self.kinematics.offset_y_mm,
                    calibration_mode=cal_res["mode"],
                    markers_detected=detected_ids,
                    cam_motor_skew_deg=cal_res["cam_motor_skew_deg"],
                    plate_yaw_deg=cal_res["plate_yaw_deg"],
                    plate_scale_x=cal_res["scale_x"],
                    plate_scale_y=cal_res["scale_y"],
                    residual_rms_mm=cal_res["residual_rms_mm"],
                    residual_max_mm=cal_res["residual_max_mm"],
                    corner_residuals_mm=cal_res["corner_residuals_mm"],
                    plate_transform=self.kinematics.plate_transform_dict(),
                )
            except Exception as exc:
                self.kinematics.calibrated = True
                self.log_event(
                    f"Fiducial calibration calculation failed ({exc}); falling back to default scale Kx={self.kinematics.kx_mm_per_px:.4f} mm/px."
                )
                self.reports.record_calibration(
                    calibrated=True,
                    origin_calibrated=False,
                    marker_id=primary_fid[0] if primary_fid else None,
                    u_px=primary_fid[1] if primary_fid else None,
                    v_px=primary_fid[2] if primary_fid else None,
                    marker_angle_deg=primary_fid[3] if primary_fid else None,
                    cam_skew_deg=self.kinematics.cam_skew_deg,
                    kx=self.kinematics.kx_mm_per_px,
                    ky=self.kinematics.ky_mm_per_px,
                    cam_origin_x=self.kinematics.cam_origin_x_mm,
                    cam_origin_y=self.kinematics.cam_origin_y_mm,
                    offset_x=self.kinematics.offset_x_mm,
                    offset_y=self.kinematics.offset_y_mm,
                    calibration_mode="nominal_fallback",
                )
        else:
            self.kinematics.calibrated = True
            self.kinematics.calibration_mode = "nominal_fallback"
            self.kinematics.markers_detected = []
            self.log_event(
                f"No corner ArUco fiducials detected; using default scale Kx={self.kinematics.kx_mm_per_px:.4f} mm/px and nominal CAD origin."
            )
            self.reports.record_calibration(
                calibrated=True,
                origin_calibrated=False,
                marker_id=None,
                u_px=None,
                v_px=None,
                marker_angle_deg=None,
                cam_skew_deg=self.kinematics.cam_skew_deg,
                kx=self.kinematics.kx_mm_per_px,
                ky=self.kinematics.ky_mm_per_px,
                cam_origin_x=self.kinematics.cam_origin_x_mm,
                cam_origin_y=self.kinematics.cam_origin_y_mm,
                offset_x=self.kinematics.offset_x_mm,
                offset_y=self.kinematics.offset_y_mm,
                calibration_mode="nominal_fallback",
            )

        # ==============================================================
        # PHASE 3: Global Course Scan (SCANNING)
        # ==============================================================
        self.state = "SCANNING"
        self.log_event(
            f"Phase 3 [SCANNING]: Stepping through {len(self.config.scan_grid_points_mm)}-point grid..."
        )
        for idx, (gx, gy) in enumerate(self.config.scan_grid_points_mm, start=1):
            self._check_pause_or_abort()
            self.serial.move_to(x_mm=gx, y_mm=gy, z_mm=z_scan)
            # Stop-and-Capture Rule: gantry is completely stationary after move_to returns OK
            frame = self.vision.capture_stationary_frame(gx, gy, z_scan)
            detections = self.vision.detect_blocks_hsv(frame)

            det_records = []
            for det in detections:
                wx, wy = self.kinematics.pixel_to_world_mm(
                    det.u_px,
                    det.v_px,
                    gx,
                    gy,
                    z_scan,
                    z_scan,
                    target_height_above_surface_mm=self.config.block_height_mm,
                )
                # Re-project once if the block rests on the raised 76.2x76.2 mm aluminum platform
                if self.config.surface_z_up_mm_at(wx, wy) > self.config.course_elevation_mm + 1e-3:
                    wx, wy = self.kinematics.pixel_to_world_mm(
                        det.u_px,
                        det.v_px,
                        gx,
                        gy,
                        z_scan,
                        z_scan,
                        target_height_above_surface_mm=self.config.block_top_height_mm_at(wx, wy),
                    )
                comp_angle = self.kinematics.compensate_block_angle(det.angle_deg)
                blk = self.planner.merge_detection(det.color, wx, wy, comp_angle)
                blk.z_mm = self.config.surface_z_up_mm_at(blk.x_mm, blk.y_mm)
                det_records.append({
                    "color": det.color,
                    "u_px": round(det.u_px, 1),
                    "v_px": round(det.v_px, 1),
                    "area_px": round(det.area_px, 1),
                    "raw_angle_deg": round(det.angle_deg, 1),
                    "comp_angle_deg": comp_angle,
                    "world_x_mm": round(wx, 2),
                    "world_y_mm": round(wy, 2),
                })

            self.reports.record_scan_waypoint(
                waypoint_idx=idx,
                gx=gx,
                gy=gy,
                gz=z_scan,
                raw_detections=det_records,
                blocks_count=len(self.planner.blocks),
            )

            self.log_event(
                f"Scan waypoint {idx}/{len(self.config.scan_grid_points_mm)} ({gx:.0f}, {gy:.0f} mm): "
                f"{len(detections)} detections ({len(self.planner.blocks)} unique blocks cataloged)."
            )

        # Save snapshot of cataloged blocks
        self.reports.record_cataloged_blocks([b.to_dict() for b in self.planner.blocks])

        # ==============================================================
        # PHASE 4: Path & Sequence Optimization (PLANNING)
        # ==============================================================
        self.state = "PLANNING"
        self._check_pause_or_abort()
        plan_kwargs = dict(
            stack_zones_mm=self.config.stack_zones_mm,
            base_plate_z_down_mm=self.config.base_plate_z_down_mm,
            course_elevation_mm=self.config.course_elevation_mm,
            aluminum_platform_size_mm=self.config.aluminum_platform_size_mm,
            z_hover_mm=self.config.z_hover_mm,
        )
        cur_ee_x, cur_ee_y = self.kinematics.gripper_motor_to_plate(
            self.serial.pose.x_mm, self.serial.pose.y_mm
        )
        pick_queue = self.planner.compute_pick_sequence(
            cur_ee_x,
            cur_ee_y,
            **plan_kwargs,
        )
        total_blocks = len(pick_queue)
        seq_preview = " -> ".join(f"#{b.id} {b.color.upper()}" for b in pick_queue)
        self.log_event(
            f"Phase 4 [PLANNING]: Precomputed CAD-safe shortest-path order for {total_blocks} blocks: {seq_preview}"
        )

        # ==============================================================
        # PHASE 5 & 6: Pick, Align, Weigh (PICKING / WEIGHING) & Sort/Stack (STACKING)
        # ==============================================================
        pair_stage_counts = {}
        cycles_done = 0
        max_cycles = max(8, total_blocks * 6)

        while pick_queue:
            self._check_pause_or_abort()
            cycles_done += 1
            if cycles_done > max_cycles:
                raise RuntimeError("Exceeded max pick/staging cycles; aborting to prevent infinite loop.")

            block = pick_queue.pop(0)
            staged_blocker_id = self._execute_pick_weigh_and_stack(block)
            if staged_blocker_id is not None:
                pair = (block.id, staged_blocker_id)
                pair_stage_counts[pair] = pair_stage_counts.get(pair, 0) + 1
                if pair_stage_counts[pair] > 3:
                    raise RuntimeError(
                        f"Repeated staging loop between Block #{block.id} and blocker Block #{staged_blocker_id}."
                    )
                pick_queue = self.planner.compute_pick_sequence(
                    block.x_mm,
                    block.y_mm,
                    priority_block_id=staged_blocker_id,
                    deferred_block_ids={block.id},
                    **plan_kwargs,
                )
            else:
                cur_ee_x, cur_ee_y = self.kinematics.gripper_motor_to_plate(
                    self.serial.pose.x_mm, self.serial.pose.y_mm
                )
                pick_queue = self.planner.compute_pick_sequence(
                    cur_ee_x,
                    cur_ee_y,
                    **plan_kwargs,
                )

        # Return to home park position: retract Z first to avoid collisions, park clear of Y blanking stop
        self.serial.move_to(z_mm=self.config.z_scan_mm, feedrate_mm_s=self.config.feedrate_z_mm_s)
        self.serial.move_to(x_mm=20.0, y_mm=30.0, angle_deg=0.0)
        self._auto_tare_scale("Post-run park")
        self._set_mass_reading(0.00, None)
        self.state = "COMPLETE"
        self.log_event(
            f"All {total_blocks} blocks processed and sorted! Returning to IDLE/COMPLETE."
        )
        try:
            self.serial.play_chime("TADA")
            self.log_event("Played completion chime (Windows 3.1 Tada!).")
        except Exception as exc:
            logger.debug("Completion chime skipped: %s", exc)

    def _safe_transit_z_down(self, clearance_margin_mm: float = 8.0) -> float:
        """
        Computes safe transit Z_down clearance above any active laydown stacks.
        Enforces at least `clearance_margin_mm` (default 8.0 mm: +2.0 mm over earlier
        6.0 mm margin) above the tallest stack so 2-high and 3-high stacks clear cleanly.
        """
        max_top_z_up = self.planner.max_stack_top_z_up_mm(self.config.stack_zones_mm)
        if max_top_z_up <= 0.0:
            return self.config.z_hover_mm
        req_z_down = self.config.base_plate_z_down_mm - (max_top_z_up + clearance_margin_mm)
        return round(max(2.0, min(self.config.z_hover_mm, req_z_down)), 2)

    def _press_cube_into_surface(
        self,
        surface_z_mm: float,
        max_press_mm: float,
        extra_unload_g: float = 0.0,
        force_g: Optional[float] = None,
        approach_offset_mm: Optional[float] = None,
        feedrate_press_mm_s: Optional[float] = None,
        guard_force_g: Optional[float] = None,
        guard_limit_z_mm: Optional[float] = None,
    ) -> float:
        """
        Fast-descends (never climbs UP) to the guarded approach height above the nominal surface,
        then presses until HX711 compressive force reaches threshold or max_press_mm is reached.
        Returns the achieved motor Z_down position.
        """
        offset_mm = (
            self.config.press_approach_offset_mm
            if approach_offset_mm is None
            else float(approach_offset_mm)
        )
        desired_approach_z = max(0.0, float(surface_z_mm) - offset_mm)
        approach_z = max(self.serial.pose.z_mm, desired_approach_z)
        base_force_g = self.config.press_force_g if force_g is None else float(force_g)
        if approach_z > self.serial.pose.z_mm + 0.1:
            self.serial.move_to(z_mm=approach_z, feedrate_mm_s=self.config.feedrate_z_mm_s)
        return self.serial.press_into_surface(
            nominal_z_mm=surface_z_mm,
            max_press_mm=max_press_mm,
            force_g=base_force_g + max(0.0, float(extra_unload_g)),
            feedrate_mm_s=(
                self.config.feedrate_press_mm_s
                if feedrate_press_mm_s is None
                else float(feedrate_press_mm_s)
            ),
            guard_force_g=guard_force_g,
            guard_limit_z_mm=guard_limit_z_mm,
        )

    def _capture_and_refine_target_block(
        self,
        block: BlockRecord,
        cam_z_mm: float,
        settle: bool = False,
        max_radius_mm: Optional[float] = None,
    ) -> Optional[tuple]:
        """Moves camera above `block`, captures a stationary frame, and returns best matching detection."""
        cam_mx, cam_my = self.kinematics.plate_to_camera_motor(block.x_mm, block.y_mm)
        cam_mx = max(0.0, min(self.config.travel_x_mm, cam_mx))
        cam_my = max(0.0, min(self.config.travel_y_mm, cam_my))
        if self.serial.pose.z_mm > cam_z_mm + 0.1:
            self.serial.move_to(z_mm=cam_z_mm, feedrate_mm_s=self.config.feedrate_z_mm_s)
        self.serial.move_to(
            x_mm=cam_mx,
            y_mm=cam_my,
            z_mm=cam_z_mm,
            angle_deg=0.0,
        )
        self._check_pause_or_abort()
        if settle and not self.mock_hardware and self.config.scan_settle_s > 0.0:
            time.sleep(self.config.scan_settle_s)

        v_frame = self.vision.capture_stationary_frame(
            self.serial.pose.x_mm, self.serial.pose.y_mm, self.serial.pose.z_mm
        )
        v_dets = [d for d in self.vision.detect_blocks_hsv(v_frame) if d.color == block.color]
        if not v_dets:
            return None

        eff_max_radius = (
            float(self.config.fine_align_max_radius_mm)
            if max_radius_mm is None
            else float(max_radius_mm)
        )
        other_blocks = [b for b in self.planner.blocks if b.id != block.id]
        target_h = self.config.block_top_height_mm_at(block.x_mm, block.y_mm)
        min_fine_area_px = 4500.0 if self.serial.pose.z_mm <= 5.0 else 8500.0
        candidates = []
        for det in v_dets:
            if det.area_px < min_fine_area_px:
                continue
            rx, ry = self.kinematics.pixel_to_world_mm(
                det.u_px,
                det.v_px,
                self.serial.pose.x_mm,
                self.serial.pose.y_mm,
                self.serial.pose.z_mm,
                self.config.z_scan_mm,
                target_height_above_surface_mm=target_h,
            )
            dist = math.hypot(rx - block.x_mm, ry - block.y_mm)
            if dist > eff_max_radius:
                continue

            # Exclude detections that belong to already-stacked blocks or another cataloged block
            belongs_to_other = False
            for other in other_blocks:
                d_other = math.hypot(rx - other.x_mm, ry - other.y_mm)
                if other.status == "STACKED" and d_other <= 22.0:
                    belongs_to_other = True
                    break
                if d_other + 2.0 < dist:
                    belongs_to_other = True
                    break
            if belongs_to_other:
                continue

            candidates.append((dist, det, rx, ry))

        if not candidates:
            return None

        err_mm, best, refined_x, refined_y = min(candidates, key=lambda item: item[0])
        return cam_mx, cam_my, best, refined_x, refined_y, err_mm

    def _fine_align_block_with_camera(
        self,
        block: BlockRecord,
        pick_record: PickActionRecord,
        scan_z_mm: float,
        is_retry: bool = False,
    ) -> bool:
        """
        Positions the eye-in-hand camera above `block` and refines its (X, Y, angle).
        For rear-edge blocks requiring cam_my < 0 or whenever the initial
        hover capture misses the target, captures at wider-FOV z_scan_mm=0.0.
        """
        action_word = "Re-scanning" if is_retry else "Moving camera above"
        raw_cam_mx, raw_cam_my = self.kinematics.plate_to_camera_motor(block.x_mm, block.y_mm)
        eff_scan_z_mm = (
            self.config.z_scan_mm
            if (raw_cam_my < -2.0 and scan_z_mm > self.config.z_scan_mm + 0.5)
            else scan_z_mm
        )
        cam_mx = max(0.0, min(self.config.travel_x_mm, raw_cam_mx))
        cam_my = max(0.0, min(self.config.travel_y_mm, raw_cam_my))
        pick_record.camera_motor_x = round(cam_mx, 2)
        pick_record.camera_motor_y = round(cam_my, 2)
        self.log_event(
            f"Phase 5 [{'RETRY ALIGN' if is_retry else 'PICKING'}]: {action_word} Block #{block.id} "
            f"({block.color.upper()}) at plate ({block.x_mm:.1f}, {block.y_mm:.1f}) mm "
            f"-> motor ({cam_mx:.1f}, {cam_my:.1f}, Z={eff_scan_z_mm:.1f}) mm..."
        )

        eff_radius = (
            self.config.fine_align_max_radius_mm + 4.0
            if is_retry
            else min(self.config.fine_align_max_radius_mm, 8.5)
        )
        match = self._capture_and_refine_target_block(
            block, eff_scan_z_mm, settle=is_retry, max_radius_mm=eff_radius
        )
        if match is None and eff_scan_z_mm > self.config.z_scan_mm + 0.5:
            self.log_event(
                f"{'Retry align' if is_retry else 'Fine align'} for Block #{block.id}: "
                f"elevating to Z_scan={self.config.z_scan_mm:.1f} mm for wider FOV..."
            )
            eff_scan_z_mm = self.config.z_scan_mm
            match = self._capture_and_refine_target_block(
                block, eff_scan_z_mm, settle=True, max_radius_mm=eff_radius
            )

        pick_record.fine_align_checked = True
        if match is None:
            pick_record.fine_align_found = False
            self.log_event(
                f"Notice: Fine alignment did not detect '{block.color}' block. "
                f"Proceeding with ({block.x_mm:.1f}, {block.y_mm:.1f}) mm."
            )
            return False

        cam_mx, cam_my, best, refined_x, refined_y, err_mm = match
        if is_retry and err_mm > 2.5:
            block.x_mm, block.y_mm = refined_x, refined_y
            second = self._capture_and_refine_target_block(
                block, eff_scan_z_mm, settle=True, max_radius_mm=eff_radius
            )
            if second is not None:
                cam_mx, cam_my, best, refined_x, refined_y, second_err = second
                err_mm = max(err_mm, second_err)

        refined_angle = self.kinematics.compensate_block_angle(best.angle_deg)
        # Keep square-block angle within +/-45 deg of prior estimate to avoid 90 deg aspect swaps
        angle_diff = (refined_angle - block.angle_deg + 180.0) % 180.0
        if angle_diff > 90.0:
            angle_diff -= 180.0
        if abs(angle_diff) > 45.0:
            refined_angle = round((refined_angle + (-90.0 if angle_diff > 0 else 90.0)) % 180.0, 1)

        pick_record.camera_motor_x = round(cam_mx, 2)
        pick_record.camera_motor_y = round(cam_my, 2)
        pick_record.fine_align_found = True
        pick_record.fine_u_px = round(best.u_px, 1)
        pick_record.fine_v_px = round(best.v_px, 1)
        pick_record.fine_contour_area_px = round(best.area_px, 1)
        pick_record.fine_residual_err_mm = round(err_mm, 2)
        pick_record.refined_plate_x = round(refined_x, 2)
        pick_record.refined_plate_y = round(refined_y, 2)
        pick_record.refined_angle_deg = refined_angle

        block.x_mm, block.y_mm, block.angle_deg = refined_x, refined_y, refined_angle
        self.log_event(
            f"Fine alignment {'re-centered' if is_retry else 'check'} on Block #{block.id}: "
            f"residual offset {err_mm:.2f} mm (area {best.area_px:.0f} px), angle {block.angle_deg:.1f} deg."
        )
        return True

    def _execute_pick_weigh_and_stack(self, block: BlockRecord, can_stage: bool = True) -> Optional[int]:
        pick_record = PickActionRecord(
            block_id=block.id,
            color=block.color,
            target_plate_x=round(block.x_mm, 2),
            target_plate_y=round(block.y_mm, 2),
            target_angle_deg=round(block.angle_deg, 1),
        )
        picked_sb = None
        staged_blocker_id: Optional[int] = None
        coarse_x, coarse_y = block.x_mm, block.y_mm

        try:
            # --- PHASE 5A: Camera Fine Align + Guarded Pick with Up to 3 Retries ---
            self.state = "PICKING"
            block.status = "PICKING"
            transit_z = self._safe_transit_z_down()
            max_pick_retries = max(0, int(self.config.max_pick_misalign_retries))
            transit_hover_z = round(max(2.0, min(self.config.z_hover_mm, transit_z)), 2)
            weigh_z = round(min(transit_hover_z, 15.0), 2)

            for retry_idx in range(0, max_pick_retries + 1):
                self._fine_align_block_with_camera(
                    block,
                    pick_record,
                    scan_z_mm=transit_z,
                    is_retry=(retry_idx > 0),
                )

                # Translate plate coordinate to gripper center target
                grip_mx, grip_my = self.kinematics.plate_to_gripper_motor(block.x_mm, block.y_mm)
                grip_mx = max(0.0, min(self.config.travel_x_mm, grip_mx))
                grip_my = max(0.0, min(self.config.travel_y_mm, grip_my))
                servo_angle = self.kinematics.servo_angle_for_plate_angle(block.angle_deg)
                pick_record.gripper_motor_x = round(grip_mx, 2)
                pick_record.gripper_motor_y = round(grip_my, 2)
                pick_record.servo_angle_deg = servo_angle

                pick_z_down = self.config.surface_pick_z_down_mm(block.x_mm, block.y_mm)
                if retry_idx == 0 and pick_z_down < self.config.z_pick_mm - 0.1:
                    self.log_event(
                        f"Topology [PICKING]: Block #{block.id} rests on raised aluminum platform -> picking at Z_down={pick_z_down:.2f} mm."
                    )

                dip_z = round(max(weigh_z + 10.0, pick_z_down - 22.0), 2)
                open_z = round(max(weigh_z, pick_z_down - 24.0), 2)
                # 1. Move horizontally at stack-safe transit height with wrist at weigh_servo_angle (0 deg)
                self.serial.move_to(
                    x_mm=grip_mx,
                    y_mm=grip_my,
                    z_mm=transit_hover_z,
                    angle_deg=self.config.weigh_servo_angle_deg,
                    feedrate_mm_s=self.config.feedrate_fine_mm_s,
                )
                # 2. Conditioning dip (jaws CLOSED, solenoid OFF) to 2 mm above cube top then pull UP to
                #    weigh_z (<=15 mm) so TARE matches the exact upward Z backlash/cable loop as WEIGH.
                self.serial.move_to(z_mm=dip_z, feedrate_mm_s=self.config.feedrate_z_mm_s)
                self.serial.move_to(z_mm=weigh_z, feedrate_mm_s=self.config.feedrate_z_mm_s)

                # 3. Settle and auto-tare HX711 at exact (X, Y, weigh_z, weigh_servo_angle) used by WEIGH
                pick_record.tare_success = self._auto_tare_scale(f"Pre-pick Block #{block.id}")

                # 4. Rotate wrist at weigh_z, descend closed to open_z, then energize GRIP 0 only for final 24 mm
                if abs(servo_angle - self.serial.pose.angle_deg) > 0.2:
                    self.serial.move_to(
                        angle_deg=servo_angle,
                        feedrate_mm_s=self.config.feedrate_fine_mm_s,
                    )
                if open_z > self.serial.pose.z_mm + 0.5:
                    self.serial.move_to(z_mm=open_z, feedrate_mm_s=self.config.feedrate_z_mm_s)
                self.serial.set_gripper(closed=False)  # Energize relay to open NC jaws
                early_limit_z = pick_z_down - self.config.early_force_z_margin_mm
                actual_z = self._press_cube_into_surface(
                    surface_z_mm=pick_z_down,
                    max_press_mm=self.config.pick_press_max_mm,
                    approach_offset_mm=self.config.pick_guard_offset_mm,
                    feedrate_press_mm_s=self.config.feedrate_pick_guard_mm_s,
                    guard_force_g=self.config.pick_guard_force_g,
                    guard_limit_z_mm=early_limit_z,
                )

                hit_early = (actual_z < early_limit_z) and (
                    self.serial.last_press_force_hit
                    or self.serial.last_press_guard_hit
                    or abs(self.serial.last_press_delta_g) > 0.5
                )
                if hit_early:
                    spike_g = abs(self.serial.last_press_delta_g)
                    self.log_event(
                        f"Misalignment [PICKING]: Early force spike ({spike_g:.1f} g) on Block #{block.id} "
                        f"at Z_down={actual_z:.2f} mm (expected >= {early_limit_z:.2f} mm) "
                        f"[attempt {retry_idx + 1}/{max_pick_retries + 1}]."
                    )
                    self.serial.move_to(z_mm=transit_z, feedrate_mm_s=self.config.feedrate_z_mm_s)
                    self.serial.set_gripper(closed=True)
                    if retry_idx >= max_pick_retries:
                        pick_record.pick_retries = max_pick_retries
                        raise RuntimeError(
                            f"Pick failed after {max_pick_retries} retries ({max_pick_retries + 1} attempts) "
                            f"on Block #{block.id} ({block.color.upper()}): last contact at Z_down={actual_z:.2f} mm (< {early_limit_z:.2f} mm)."
                        )
                    pick_record.pick_retries = retry_idx + 1
                    continue

                self.serial.set_gripper(closed=True)   # De-energize relay so NC solenoid grips block
                self._held_block_id = block.id
                block.status = "HELD"
                self.publish_telemetry()
                break

            if pick_record.fine_align_found:
                self.kinematics.record_fine_alignment_drift(
                    coarse_x, coarse_y, block.x_mm, block.y_mm
                )

            # Mark closest ground-truth block as picked in synthetic renderer if in mock mode
            if self.mock_hardware:
                unpicked = [sb for sb in self.vision.simulated_blocks if not sb.picked]
                if unpicked:
                    nearest_sb = min(
                        unpicked,
                        key=lambda sb: (sb.x_mm - block.x_mm) ** 2 + (sb.y_mm - block.y_mm) ** 2,
                    )
                    nearest_sb.picked = True
                    picked_sb = nearest_sb
                    self.serial.set_simulated_held_mass(nearest_sb.mass_g)

            # Lift straight UP to weigh_z matching TARE height, direction & wrist angle (0 deg)
            if grip_my < self.config.servo_safe_y_mm:
                self.serial.move_to(z_mm=weigh_z, feedrate_mm_s=self.config.feedrate_z_mm_s)
                if abs(self.serial.pose.angle_deg - self.config.weigh_servo_angle_deg) > 0.2:
                    self.serial.move_to(
                        angle_deg=self.config.weigh_servo_angle_deg,
                        feedrate_mm_s=self.config.feedrate_fine_mm_s,
                    )
            else:
                self.serial.move_to(
                    z_mm=weigh_z,
                    angle_deg=self.config.weigh_servo_angle_deg,
                    feedrate_mm_s=self.config.feedrate_z_mm_s,
                )
            if not self.mock_hardware and self.config.weigh_settle_s > 0.0:
                time.sleep(self.config.weigh_settle_s)

            # --- PHASE 5B: Weigh Block on HX711 Load Cell & Classify Mass ---
            self.state = "WEIGHING"
            self._check_pause_or_abort()
            try:
                measured_mass_g = self.serial.weigh_block()
                self._set_mass_reading(round(measured_mass_g, 2), None)
            except Exception as exc:
                self.log_event(f"Notice: Load cell sample failed ({exc}); assigning default nominal mass 7.05g.")
                measured_mass_g = 7.05
                self._set_mass_reading(round(measured_mass_g, 2), str(exc))

            block.mass_g = measured_mass_g
            block.mass_class = self.config.classify_mass(measured_mass_g)
            block.status = "WEIGHED"
            pick_record.measured_mass_g = round(measured_mass_g, 2)
            pick_record.mass_class = block.mass_class
            self.log_event(
                f"Phase 5 [WEIGHING]: Block #{block.id} ({block.color.upper()}) mass = {measured_mass_g:.2f} g "
                f"-> class [{block.mass_class.upper()}]."
            )

            # --- PHASE 6: Sort & Laydown (Aluminum pedestal vs Red/Green/Blue laydown areas) ---
            self.state = "STACKING"
            self._check_pause_or_abort()
            drop_plate_x, drop_plate_y, drop_z_down, zone_key = self.planner.get_stack_target(
                color=block.color,
                mass_class=block.mass_class,
                stack_zones_mm=self.config.stack_zones_mm,
                base_plate_z_down_mm=self.config.base_plate_z_down_mm,
                commit=False,
            )

            blocker = self.planner.find_zone_obstruction(
                drop_plate_x,
                drop_plate_y,
                exclude_block_id=block.id,
            )

            stage_pt = None
            if blocker is not None:
                if not can_stage:
                    raise RuntimeError(
                        f"Cannot stack Block #{block.id}: destination {zone_key.upper()} blocked by Block #{blocker.id} and staging disallowed."
                    )
                stage_pt = self.planner.find_staging_spot(
                    ref_x_mm=block.x_mm,
                    ref_y_mm=block.y_mm,
                    stack_zones_mm=self.config.stack_zones_mm,
                    exclude_block_id=block.id,
                    kinematics=self.kinematics,
                    travel_x_mm=self.config.travel_x_mm,
                    travel_y_mm=self.config.travel_y_mm,
                    min_safe_y_mm=0.0,
                    aluminum_platform_size_mm=self.config.aluminum_platform_size_mm,
                )
                if stage_pt is None:
                    raise RuntimeError(
                        f"Cannot stack Block #{block.id}: destination {zone_key.upper()} blocked by Block #{blocker.id} and no clear staging spot exists."
                    )

            if blocker is not None and stage_pt is not None:
                staged_blocker_id = blocker.id
                drop_plate_x, drop_plate_y = stage_pt
                drop_z_down = self.config.surface_pick_z_down_mm(drop_plate_x, drop_plate_y)
                zone_label = f"STAGE_TEMP (Blocked by #{blocker.id})"
            else:
                zone_label = f"{zone_key.upper()} ({block.mass_class.upper()})"

            pick_record.drop_plate_x = round(drop_plate_x, 2)
            pick_record.drop_plate_y = round(drop_plate_y, 2)
            pick_record.drop_z_down_mm = round(drop_z_down, 2)
            pick_record.stack_zone = zone_label

            # Convert drop Plate Coordinates to gripper motor coordinates (compensating any observed drift)
            drop_mx, drop_my = self.kinematics.plate_to_drop_gripper_motor(drop_plate_x, drop_plate_y)
            drop_mx = max(0.0, min(self.config.travel_x_mm, drop_mx))
            drop_my = max(0.0, min(self.config.travel_y_mm, drop_my))
            if staged_blocker_id is not None:
                self.log_event(
                    f"Phase 6 [STAGING]: Destination {zone_key.upper()} blocked by Block #{blocker.id} "
                    f"({blocker.color.upper()} at {blocker.x_mm:.1f}, {blocker.y_mm:.1f}); "
                    f"staging Block #{block.id} at intermediate ({drop_plate_x:.1f}, {drop_plate_y:.1f}) mm "
                    f"-> motor ({drop_mx:.1f}, {drop_my:.1f}, Z_down={drop_z_down:.1f}) mm..."
                )
            else:
                self.log_event(
                    f"Phase 6 [STACKING]: Routing Block #{block.id} to {zone_label} "
                    f"at plate ({drop_plate_x:.1f}, {drop_plate_y:.1f}) -> motor ({drop_mx:.1f}, {drop_my:.1f}, Z_down={drop_z_down:.1f}) mm..."
                )

            # Smoothly blend horizontal travel & any pre-drop Z adjust into ONE coordinated MOVE
            pre_drop_z_down = round(min(transit_z, max(2.0, drop_z_down - 12.0)), 2)
            if self.serial.pose.z_mm > transit_z + 0.5:
                self.serial.move_to(z_mm=transit_z, feedrate_mm_s=self.config.feedrate_z_mm_s)
            self.serial.move_to(
                x_mm=drop_mx,
                y_mm=drop_my,
                z_mm=pre_drop_z_down,
                angle_deg=self.kinematics.servo_angle_for_plate_angle(0.0),
                feedrate_mm_s=self.config.feedrate_travel_mm_s,
            )

            # Guarded laydown with up to 3 retries if an unexpected object triggers an early force spike
            max_drop_retries = max(0, int(self.config.max_drop_collision_retries))
            held_mass_g = max(0.0, float(block.mass_g or 0.0))
            drop_early_limit_z = drop_z_down - self.config.drop_early_force_z_margin_mm
            for drop_retry in range(0, max_drop_retries + 1):
                actual_drop_z = self._press_cube_into_surface(
                    surface_z_mm=drop_z_down,
                    max_press_mm=self.config.drop_press_max_mm,
                    extra_unload_g=held_mass_g,
                    force_g=self.config.drop_press_force_g,
                    approach_offset_mm=self.config.drop_guard_offset_mm,
                    feedrate_press_mm_s=self.config.feedrate_drop_guard_mm_s,
                    guard_force_g=self.config.drop_guard_force_g + held_mass_g,
                    guard_limit_z_mm=drop_early_limit_z,
                )
                drop_hit_early = (actual_drop_z < drop_early_limit_z) and (
                    self.serial.last_press_force_hit
                    or self.serial.last_press_guard_hit
                    or abs(self.serial.last_press_delta_g) > 0.5
                )
                if drop_hit_early:
                    spike_g = abs(self.serial.last_press_delta_g)
                    self.log_event(
                        f"Unexpected Object [STACKING]: Early collision force ({spike_g:.1f} g) in {zone_label} "
                        f"for Block #{block.id} at Z_down={actual_drop_z:.2f} mm (expected >= {drop_early_limit_z:.2f} mm) "
                        f"[attempt {drop_retry + 1}/{max_drop_retries + 1}]."
                    )
                    if drop_retry >= max_drop_retries:
                        pick_record.drop_retries = max_drop_retries
                        self.log_event(
                            f"Warning [STACKING]: Max drop retries ({max_drop_retries}) reached in {zone_label} "
                            f"for Block #{block.id} at Z_down={actual_drop_z:.2f} mm (< {drop_early_limit_z:.2f} mm); releasing block to continue."
                        )
                        break
                    # Keep gripper CLOSED while retracting and retrying
                    self.serial.move_to(z_mm=pre_drop_z_down, feedrate_mm_s=self.config.feedrate_z_mm_s)
                    pick_record.drop_retries = drop_retry + 1
                    if not self.mock_hardware:
                        time.sleep(0.35)
                    continue
                break

            self.serial.set_gripper(closed=False)  # Release block only after clean seating
            self._held_block_id = None
            self.serial.set_simulated_held_mass(None)
            if staged_blocker_id is None:
                self.planner.commit_stack_placement(zone_key)
                zone_z_up = self.config.stack_zones_mm[zone_key][2]
                stack_n = self.planner.stack_counts[zone_key]
                block.z_mm = round(zone_z_up + (stack_n - 1) * self.config.block_height_mm, 2)
            else:
                block.z_mm = round(self.config.surface_z_up_mm_at(drop_plate_x, drop_plate_y), 2)

            # Retract straight UP high enough to clear the newly heightened stack BEFORE any X/Y move
            post_drop_z_down = round(min(self._safe_transit_z_down(), max(2.0, drop_z_down - 12.0)), 2)
            self.serial.move_to(z_mm=post_drop_z_down, feedrate_mm_s=self.config.feedrate_z_mm_s)
            self.serial.set_gripper(closed=True)   # Return NC solenoid to de-energized rest state
            self._set_mass_reading(0.00, None)

            block.x_mm = drop_plate_x
            block.y_mm = drop_plate_y
            block.angle_deg = 0.0
            if staged_blocker_id is not None:
                block.status = "QUEUED"
                if self.mock_hardware and picked_sb is not None:
                    picked_sb.x_mm = drop_plate_x
                    picked_sb.y_mm = drop_plate_y
                    picked_sb.angle_deg = 0.0
                    picked_sb.picked = False
                pick_record.status = "STAGED"
            else:
                block.status = "STACKED"
                pick_record.status = "SUCCESS"
            self.publish_telemetry()
        except Exception as exc:
            try:
                if not self.serial.pose.grip_closed:
                    self.serial.set_gripper(closed=True)
            except Exception:
                pass
            pick_record.status = "FAILED"
            pick_record.error = str(exc)
            raise
        finally:
            self.reports.record_pick_action(pick_record)
        self.publish_telemetry()
        return staged_blocker_id

    # ------------------------------------------------------------------
    # Xbox Series X|S Manual Teleoperation Mode
    # ------------------------------------------------------------------
    def _on_gamepad_connected(self, name: str, transport: str) -> None:
        self.log_event(f"Xbox controller connected via {transport.upper()}: {name}")

    def _on_bt_status_change(self, bt_status: dict) -> None:
        msg = bt_status.get("message")
        if msg:
            self.log_event(f"Bluetooth: {msg}")
        else:
            self.publish_telemetry()

    def _on_gamepad_disconnected(self, name: str) -> None:
        if self.state == "MANUAL":
            try:
                with self._teleop_cycle_lock:
                    self.serial.stop_velocity()
                    if not self.serial.pose.grip_closed:
                        self.serial.set_gripper(closed=True, nowait=True)
            except Exception:
                pass
        self.log_event(f"Xbox controller disconnected ({name}). Velocity halted safely.")

    def _trigger_rumble(self, strong: float, weak: float, duration_ms: int) -> None:
        self._gamepad_reader.rumble(strong=strong, weak=weak, duration_ms=duration_ms)
        self._rumble_seq += 1
        self._rumble_payload = {
            "strong": round(float(strong), 2),
            "weak": round(float(weak), 2),
            "duration_ms": int(duration_ms),
        }

    def _start_manual_timer(self) -> None:
        t = self._manual_timer
        if not t["running"]:
            t["started_mono"] = time.monotonic()
            t["running"] = True

    def _stop_manual_timer(self) -> None:
        t = self._manual_timer
        if t["running"] and t["started_mono"] is not None:
            t["elapsed_s"] = round(float(t["elapsed_s"]) + (time.monotonic() - float(t["started_mono"])), 2)
        t["started_mono"] = None
        t["running"] = False

    def _handle_manual_timer_action(self, action: str) -> None:
        t = self._manual_timer
        if action == "start":
            self._start_manual_timer()
        elif action == "stop":
            self._stop_manual_timer()
        elif action == "toggle":
            if t["running"]:
                self._stop_manual_timer()
            else:
                self._start_manual_timer()
        elif action == "reset":
            t["running"] = False
            t["started_mono"] = None
            t["elapsed_s"] = 0.0
            t["blocks_placed"] = 0
        elif action == "add_block":
            t["blocks_placed"] = int(t["blocks_placed"]) + 1
        elif action == "record_run":
            self._stop_manual_timer()
            elapsed = float(t["elapsed_s"])
            if elapsed > 0.1:
                if t["best_run_s"] is None or elapsed < float(t["best_run_s"]):
                    t["best_run_s"] = round(elapsed, 2)
                runs = list(t["runs"])
                runs.append({"elapsed_s": round(elapsed, 2), "blocks": int(t["blocks_placed"])})
                t["runs"] = runs[-8:]

    def _manual_mode_telemetry(self) -> dict:
        ev_status = self._gamepad_reader.status_dict()
        with self._manual_lock:
            mapping_copy = dict(self._gamepad_mapping)
            b_snap = self._browser_gp_snapshot
            b_fresh = b_snap is not None and (time.monotonic() - b_snap.timestamp_mono) < 1.5
            b_name = self._browser_gp_device_name
            b_btn = self._browser_last_button
            b_seq = self._browser_last_button_seq

        if ev_status["connected"]:
            ctrl = {
                "connected": True,
                "device_name": ev_status["device_name"],
                "transport": ev_status["transport"],
                "evdev_supported": ev_status["evdev_supported"],
                "last_button": ev_status["last_button"],
                "last_button_seq": ev_status["last_button_seq"],
            }
        elif b_fresh:
            ctrl = {
                "connected": True,
                "device_name": b_name or "Browser Gamepad",
                "transport": "browser",
                "evdev_supported": ev_status["evdev_supported"],
                "last_button": b_btn,
                "last_button_seq": b_seq,
            }
        else:
            ctrl = {
                "connected": False,
                "device_name": "",
                "transport": "none",
                "evdev_supported": ev_status["evdev_supported"],
                "last_button": "",
                "last_button_seq": 0,
            }

        t = self._manual_timer
        live_elapsed = float(t["elapsed_s"])
        if t["running"] and t["started_mono"] is not None:
            live_elapsed += time.monotonic() - float(t["started_mono"])

        return {
            "active": self.state == "MANUAL",
            "telemetry_mode": mapping_copy.get("telemetry_mode", "suspended"),
            "controller": ctrl,
            "bt_pairing": self._bt_manager.status_dict(),
            "mapping": mapping_copy,
            "z_guard_tripped": self.serial.last_teleop_z_guard,
            "z_delta_g": round(self.serial.last_teleop_delta_g, 1),
            "solenoid_thermal_pct": self.serial.last_teleop_thermal_pct,
            "rumble_seq": self._rumble_seq,
            "rumble": self._rumble_payload,
            "timer": {
                "running": bool(t["running"]),
                "elapsed_s": round(live_elapsed, 2),
                "blocks_placed": int(t["blocks_placed"]),
                "best_run_s": t["best_run_s"],
                "runs": list(t["runs"]),
            },
            "last_auto_cycle_s": self._last_auto_cycle_s,
            "last_auto_blocks": self._last_auto_blocks,
        }

    def _enter_manual_mode(self, source: str = "dashboard") -> None:
        if self.state == "MANUAL":
            return
        with self._teleop_cycle_lock:
            self._estop_active = False
            self._pause_requested = False
            self._start_requested = False
            self._manual_interrupt_requested = False
            self.serial.clear_emergency_stop()
            try:
                self.serial.ensure_awake()
                if not self._homed_known:
                    self.log_event("MANUAL entry: homing axes first to establish soft limits...")
                    self.serial.home_all()
                    self.serial.move_to(
                        y_mm=self.config.servo_safe_y_mm,
                        z_mm=10.0,
                        angle_deg=0.0,
                        feedrate_mm_s=self.config.feedrate_z_mm_s,
                    )
                    self._auto_tare_scale("MANUAL entry")
                    self._homed_known = True
                try:
                    self.serial.play_chime("MANUAL_ON")
                except Exception as exc:
                    logger.debug("MANUAL_ON chime skipped: %s", exc)
                self._teleop_mapper.reset()
                self.serial.last_teleop_z_guard = False
                self.state = "MANUAL"
                self.log_event(
                    f"Entered MANUAL Xbox mode ({source}). Live vision paused; telemetry={self._gamepad_mapping.get('telemetry_mode')}."
                )
            except Exception as exc:
                self.state = "ERROR"
                self.log_event(f"Failed to enter MANUAL mode: {exc}")
        self.publish_telemetry()

    def _exit_manual_mode(self, source: str = "dashboard") -> None:
        if self.state != "MANUAL":
            return
        with self._teleop_cycle_lock:
            self._stop_manual_timer()
            self.state = "IDLE"
            try:
                self.serial.stop_velocity()
                if not self.serial.pose.grip_closed:
                    self.serial.set_gripper(closed=True)
                self.planner.clear()
                self._held_block_id = None
                try:
                    self.serial.play_chime("MANUAL_OFF")
                except Exception as exc:
                    logger.debug("MANUAL_OFF chime skipped: %s", exc)
            except Exception as exc:
                logger.warning("Cleanup while exiting MANUAL mode: %s", exc)
            self.log_event(f"Exited MANUAL Xbox mode ({source}) -> IDLE. Re-scan before next auto run.")
        self.publish_telemetry()

    def _active_gamepad_snapshot(self) -> tuple[GamepadSnapshot, str]:
        if self._gamepad_reader.is_connected:
            snap = self._gamepad_reader.get_snapshot()
            return snap, snap.source
        with self._manual_lock:
            b_snap = self._browser_gp_snapshot
        if b_snap is not None and (time.monotonic() - b_snap.timestamp_mono) < 0.45:
            return b_snap, "browser"
        return GamepadSnapshot(), "none"

    def _teleop_loop(self) -> None:
        period_s = 1.0 / max(10.0, float(self.config.teleop_hz))
        last_pub_mono = 0.0
        last_thermal_rumble_mono = 0.0
        prev_z_guard = False
        last_thermal_band = 0
        last_grip_desired_closed: Optional[bool] = None
        last_grip_cmd_mono = 0.0

        while self._running:
            loop_t0 = time.monotonic()
            snap, src = self._active_gamepad_snapshot()

            if self.state != "MANUAL":
                last_grip_desired_closed = None
                prev_z_guard = False
                if src != "none" and self.state in ("IDLE", "COMPLETE", "PAUSED", "ERROR"):
                    with self._manual_lock:
                        mapping_copy = dict(self._gamepad_mapping)
                    cmd = self._teleop_mapper.map_input(snap, mapping_copy)
                    if cmd.trigger_toggle_mode:
                        self._on_user_command("TOGGLE_MANUAL_MODE", {"source": f"controller:{src}"})
                time.sleep(0.08)
                continue

            if self._estop_active:
                time.sleep(0.08)
                continue

            with self._manual_lock:
                mapping_copy = dict(self._gamepad_mapping)
            cmd = self._teleop_mapper.map_input(snap, mapping_copy)

            if cmd.trigger_toggle_mode:
                self._on_user_command("TOGGLE_MANUAL_MODE", {"source": f"controller:{src}"})
                time.sleep(0.10)
                continue

            should_publish = False
            with self._teleop_cycle_lock:
                if self.state != "MANUAL" or self._estop_active:
                    continue
                try:
                    if cmd.trigger_soft_estop:
                        self.serial.stop_velocity()
                        self.serial.set_gripper(closed=True, nowait=True)
                        last_grip_desired_closed = True
                        self.log_event("Manual soft-halt (controller) executed.")
                        should_publish = True
                    elif cmd.trigger_weigh and not cmd.any_motion:
                        self.serial.stop_velocity()
                        measured = self.serial.weigh_block()
                        self._set_mass_reading(round(measured, 2), None)
                        cls = self.config.classify_mass(measured)
                        self.log_event(f"Manual weigh: {measured:.2f} g -> [{cls.upper()}].")
                        should_publish = True
                    elif cmd.trigger_home and not cmd.any_motion:
                        self.serial.stop_velocity()
                        self.serial.home_all()
                        self._homed_known = True
                        self.log_event("Manual re-home (controller) complete.")
                        should_publish = True
                    elif src != "none":
                        desired_closed = not cmd.grip_open_desired
                        now_m = time.monotonic()
                        if last_grip_desired_closed is None:
                            last_grip_desired_closed = self.serial.pose.grip_closed
                        if desired_closed != last_grip_desired_closed:
                            self.serial.set_gripper(closed=desired_closed, nowait=True)
                            last_grip_desired_closed = desired_closed
                            last_grip_cmd_mono = now_m
                            should_publish = True

                        if cmd.any_motion and not self._manual_timer["running"] and self._manual_timer["elapsed_s"] == 0.0:
                            self._start_manual_timer()
                            should_publish = True

                        info = self.serial.send_velocity(
                            vx_mm_s=cmd.vx_mm_s,
                            vy_mm_s=cmd.vy_mm_s,
                            vz_mm_s=cmd.vz_mm_s,
                            va_deg_s=cmd.va_deg_s,
                            z_max_mm=self.config.travel_z_bottom_mm,
                            z_guard_enabled=bool(mapping_copy.get("z_guard_enabled", True)),
                            z_guard_force_g=float(mapping_copy.get("z_guard_force_g", 150.0)),
                            notify_pose=False,
                        )
                        if last_grip_desired_closed is False and self.serial.pose.grip_closed and (now_m - last_grip_cmd_mono) >= 0.20:
                            self._teleop_mapper.latch_grip_closed_until_next_input()
                            last_grip_desired_closed = True
                            self._trigger_rumble(0.5, 0.85, 220)
                            self.log_event("Solenoid thermal watchdog tripped — gripper toggled OFF until next input.")
                            should_publish = True
                        zg_now = bool(info.get("z_guard", False))
                        th_pct = int(info.get("thermal_pct", 0))
                        if zg_now and not prev_z_guard:
                            self._trigger_rumble(0.85, 0.45, 200)
                            self.log_event(
                                f"HX711 Z-Crash Guard triggered ({info.get('delta_g', 0.0):.1f} g) — downward Z blocked; lift up to clear."
                            )
                            should_publish = True
                        elif not zg_now and prev_z_guard:
                            should_publish = True
                        prev_z_guard = zg_now

                        if th_pct >= 70 and (now_m - last_thermal_rumble_mono) >= 1.5:
                            self._trigger_rumble(0.35, 0.75, 160)
                            last_thermal_rumble_mono = now_m
                        th_band = 2 if th_pct >= 98 else (1 if th_pct >= 70 else 0)
                        if th_band != last_thermal_band:
                            last_thermal_band = th_band
                            should_publish = True
                except Exception as exc:
                    self.log_event(f"Manual teleop error: {exc}")
                    should_publish = True

            t_mode = str(mapping_copy.get("telemetry_mode", "suspended"))
            now_m = time.monotonic()
            if t_mode == "low_rate":
                low_period = 1.0 / max(1.0, float(self.config.teleop_low_rate_hz))
                if (now_m - last_pub_mono) >= low_period:
                    should_publish = True

            if should_publish:
                last_pub_mono = now_m
                self.publish_telemetry()

            elapsed = time.monotonic() - loop_t0
            if elapsed < period_s:
                time.sleep(period_s - elapsed)
