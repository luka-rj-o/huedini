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
import logging
import math
import queue
import threading
import time
from typing import Deque, Optional

from .config import GantryConfig
from .kinematics import BlockRecord, EyeInHandKinematics, WorldCatalogAndPlanner
from .report_logger import PickActionRecord, RunReportRecorder
from .serial_bridge import ESP32SerialBridge, GantryPose
from .telemetry_server import TelemetryServer
from .vision import VisionPipeline

logger = logging.getLogger("gantry.fsm")


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
        self._cmd_queue: queue.Queue = queue.Queue()
        self._cmd_thread: Optional[threading.Thread] = None
        self._latest_live_fiducials = []
        self._latest_live_blocks = []
        self._worker_thread: Optional[threading.Thread] = None
        self._live_vision_thread: Optional[threading.Thread] = None

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

    def stop(self) -> None:
        self._running = False
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
            },
            "blocks": [b.to_dict() for b in self.planner.blocks],
            "live_fiducials": list(self._latest_live_fiducials),
            "live_blocks": list(self._latest_live_blocks),
            "live_cv_enabled": self._live_cv_enabled,
            "logs": list(self._logs),
            "vision_frame_b64": self.vision.last_annotated_jpeg_b64,
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

    def _on_user_command(self, cmd: str, data: dict) -> None:
        """
        Fast non-blocking command receiver called by the WebSocket server thread.
        ESTOP and RESET unblock any waiting commands and drain stale queues immediately;
        all other motion and configuration commands are queued sequentially.
        """
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

        if cmd in ("WEIGH_BLOCK", "SAMPLE_MASS", "TARE_SCALE", "HOME_ALL", "START_SCAN"):
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
            self.serial.ensure_awake()
            if self.mock_hardware:
                self.vision.reset_simulated_course()
            self.planner.clear()
            self._set_mass_reading(None, None)
            self.state = "IDLE"
            self.log_event("System reset to IDLE and motors re-energized.")
        elif cmd == "HOME_ALL":
            if self.state in ("IDLE", "COMPLETE", "ERROR", "ESTOPPED"):
                self._estop_active = False
                self.serial.clear_emergency_stop()
                self.state = "CALIBRATING"
                self.publish_telemetry()
                self.log_event("Supervisor manual HOME_ALL triggered for Z, X, Y...")
                try:
                    self.serial.ensure_awake()
                    self.serial.home_all()
                    self._auto_tare_scale("HOME_ALL")
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
        elif cmd in ("FREE_MOTORS", "SLEEP_MOTORS"):
            try:
                self._estop_active = False
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

    def _check_pause_or_abort(self) -> None:
        if self._pause_requested and not self._estop_active and self._running:
            saved_state = self.state
            self.state = "PAUSED"
            self.publish_telemetry()
            while self._pause_requested and not self._estop_active and self._running:
                time.sleep(0.15)
            if not self._estop_active and self._running:
                self.state = saved_state
                self.publish_telemetry()

        if self._estop_active or not self._running:
            raise RuntimeError("ERR:EMERGENCY_STOP_ACTIVE")

    def _live_vision_loop(self) -> None:
        """
        Background loop providing semi-live (~2.5-3 FPS) video feed to the Tier 1 Dashboard
        whenever the robot is stationary / manual / idle / paused / errored.
        Pauses during automated multi-axis motion / scan cycles so as not to contend for camera.
        """
        last_frame_time = 0.0
        target_period_s = 0.35  # ~2.8 FPS - optimal for Pi Zero W single-core CPU
        while self._running:
            can_stream = (
                self._live_stream_enabled
                and not self._start_requested
                and self.state in ("IDLE", "PAUSED", "COMPLETE", "ERROR", "ESTOPPED")
            )
            if not can_stream:
                time.sleep(0.1)
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
            try:
                self.reports.start_new_run(mock_hardware=self.mock_hardware)
                self._run_full_cycle()
                self.reports.finish_run(status="COMPLETED")
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

    def _auto_tare_scale(self, context_label: str) -> bool:
        """Automatically zeroes the HX711 scale after homing/before picking."""
        try:
            self.serial.tare_scale()
            self._set_mass_reading(0.00, None)
            self.log_event(f"{context_label}: HX711 scale auto-tared (0.00 g).")
            return True
        except Exception as exc:
            self.log_event(f"Notice: {context_label} auto-tare skipped ({exc}).")
            return False

    def _run_full_cycle(self) -> None:
        # ==============================================================
        # PHASE 2: Homing & 4-ArUco Ground-Truth Calibration (CALIBRATING)
        # ==============================================================
        self.state = "CALIBRATING"
        self.log_event("Phase 2 [CALIBRATING]: Homing Z, X, Y axes via TMC2209 StallGuard4...")
        self._check_pause_or_abort()
        self.serial.home_all()
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
                self.planner.merge_detection(det.color, wx, wy, comp_angle)
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
        pick_queue = self.planner.compute_pick_sequence(
            self.serial.pose.x_mm,
            self.serial.pose.y_mm,
            stack_zones_mm=self.config.stack_zones_mm,
        )
        total_blocks = len(pick_queue)
        self.log_event(
            f"Phase 4 [PLANNING]: Computed laydown-first, sequential pick order for {total_blocks} blocks."
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
                    stack_zones_mm=self.config.stack_zones_mm,
                    priority_block_id=staged_blocker_id,
                    deferred_block_ids={block.id},
                )

        # Return to home park position: retract Z first to avoid collisions, park clear of Y blanking stop
        self.serial.move_to(z_mm=self.config.z_scan_mm, feedrate_mm_s=self.config.feedrate_z_mm_s)
        self.serial.move_to(x_mm=20.0, y_mm=30.0, angle_deg=0.0)
        self.state = "COMPLETE"
        self.log_event(
            f"All {total_blocks} blocks processed and sorted! Returning to IDLE/COMPLETE."
        )
        try:
            self.serial.play_chime("TADA")
            self.log_event("Played completion chime (Windows 3.1 Tada!).")
        except Exception as exc:
            logger.debug("Completion chime skipped: %s", exc)

    def _safe_transit_z_down(self, clearance_margin_mm: float = 11.0) -> float:
        """
        Computes safe transit Z_down clearance.
        At z_hover_mm (25.0 mm), the end-effector naturally clears 1-high (26.35 mm)
        and 2-high (46.35 mm) RGB stacks as well as 1-high Aluminum (35.9 mm).
        Only raise transit Z above z_hover_mm once a 3rd+ cube stack (> 48.0 mm top)
        exists so 1- and 2-cube stacks do not trigger unnecessary Z travel.
        """
        max_top_z_up = self.planner.max_stack_top_z_up_mm(self.config.stack_zones_mm)
        if max_top_z_up <= 48.0:
            return self.config.z_hover_mm
        req_z_down = self.config.base_plate_z_down_mm - (max_top_z_up + clearance_margin_mm)
        return round(max(0.0, min(self.config.z_hover_mm, req_z_down)), 2)

    def _press_cube_into_surface(
        self,
        surface_z_mm: float,
        max_press_mm: float,
        extra_unload_g: float = 0.0,
        force_g: Optional[float] = None,
        approach_offset_mm: Optional[float] = None,
        feedrate_press_mm_s: Optional[float] = None,
    ) -> float:
        """
        Fast-descends to the guarded approach height above the nominal surface, then presses
        until HX711 compressive force reaches threshold or max_press_mm is reached.
        Returns the achieved motor Z_down position.
        """
        offset_mm = (
            self.config.press_approach_offset_mm
            if approach_offset_mm is None
            else float(approach_offset_mm)
        )
        approach_z = max(0.0, float(surface_z_mm) - offset_mm)
        base_force_g = self.config.press_force_g if force_g is None else float(force_g)
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
        )

    def _capture_and_refine_target_block(
        self,
        block: BlockRecord,
        cam_z_mm: float,
        settle: bool = False,
    ) -> Optional[tuple]:
        """Moves camera above `block`, captures a stationary frame, and returns best matching detection."""
        cam_mx, cam_my = self.kinematics.plate_to_camera_motor(block.x_mm, block.y_mm)
        cam_mx = max(0.0, min(self.config.travel_x_mm, cam_mx))
        cam_my = max(
            self.config.servo_safe_y_mm if cam_z_mm > 2.0 else 0.0,
            min(self.config.travel_y_mm, cam_my),
        )
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

        target_h = self.config.block_top_height_mm_at(block.x_mm, block.y_mm)
        candidates = []
        for det in v_dets:
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
            candidates.append((dist, det, rx, ry))

        within_radius = [c for c in candidates if c[0] <= 25.0]
        pool = within_radius if within_radius else candidates
        err_mm, best, refined_x, refined_y = min(pool, key=lambda item: item[0])
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
        On misalignment retry, settles before capture, escalates to wider-FOV z_scan_mm=0.0
        if unseen, and performs a second centering capture if residual offset > 2.5 mm.
        """
        action_word = "Re-scanning" if is_retry else "Moving camera above"
        cam_mx, cam_my = self.kinematics.plate_to_camera_motor(block.x_mm, block.y_mm)
        cam_mx = max(0.0, min(self.config.travel_x_mm, cam_mx))
        cam_my = max(self.config.servo_safe_y_mm, min(self.config.travel_y_mm, cam_my))
        pick_record.camera_motor_x = round(cam_mx, 2)
        pick_record.camera_motor_y = round(cam_my, 2)
        self.log_event(
            f"Phase 5 [{'RETRY ALIGN' if is_retry else 'PICKING'}]: {action_word} Block #{block.id} "
            f"({block.color.upper()}) at plate ({block.x_mm:.1f}, {block.y_mm:.1f}) mm "
            f"-> motor ({cam_mx:.1f}, {cam_my:.1f}, Z={scan_z_mm:.1f}) mm..."
        )

        match = self._capture_and_refine_target_block(block, scan_z_mm, settle=is_retry)
        if match is None and is_retry and scan_z_mm > self.config.z_scan_mm + 0.5:
            self.log_event(
                f"Retry align for Block #{block.id}: elevating to Z_scan={self.config.z_scan_mm:.1f} mm for wider FOV..."
            )
            match = self._capture_and_refine_target_block(
                block, self.config.z_scan_mm, settle=True
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
            second = self._capture_and_refine_target_block(block, scan_z_mm, settle=True)
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
            # --- PHASE 5A: Camera Fine Align + Guarded Pick with Early Force-Spike Retry ---
            self.state = "PICKING"
            block.status = "PICKING"
            transit_z = self._safe_transit_z_down()
            max_attempts = max(1, int(self.config.max_pick_misalign_retries))
            weigh_z = transit_z

            for attempt in range(1, max_attempts + 1):
                self._fine_align_block_with_camera(
                    block,
                    pick_record,
                    scan_z_mm=transit_z,
                    is_retry=(attempt > 1),
                )

                # Translate plate coordinate to gripper center target (clamped clear of rear stop)
                grip_mx, grip_my = self.kinematics.plate_to_gripper_motor(block.x_mm, block.y_mm)
                grip_mx = max(0.0, min(self.config.travel_x_mm, grip_mx))
                grip_my = max(self.config.servo_safe_y_mm, min(self.config.travel_y_mm, grip_my))
                servo_angle = self.kinematics.servo_angle_for_plate_angle(block.angle_deg)
                pick_record.gripper_motor_x = round(grip_mx, 2)
                pick_record.gripper_motor_y = round(grip_my, 2)
                pick_record.servo_angle_deg = servo_angle

                pick_z_down = self.config.surface_pick_z_down_mm(block.x_mm, block.y_mm)
                weigh_z = min(self.config.z_hover_mm, max(10.0, pick_z_down - 35.0))
                if attempt == 1 and pick_z_down < self.config.z_pick_mm - 0.1:
                    self.log_event(
                        f"Topology [PICKING]: Block #{block.id} rests on raised aluminum platform -> picking at Z_down={pick_z_down:.2f} mm."
                    )

                if self.serial.pose.z_mm > weigh_z + 0.1:
                    self.serial.move_to(z_mm=weigh_z, feedrate_mm_s=self.config.feedrate_z_mm_s)
                self.serial.move_to(
                    x_mm=grip_mx,
                    y_mm=grip_my,
                    z_mm=weigh_z,
                    angle_deg=servo_angle,
                    feedrate_mm_s=self.config.feedrate_fine_mm_s,
                )

                # Tare HX711 at hover height before opening jaws and starting guarded descent
                pick_record.tare_success = self._auto_tare_scale(f"Pre-pick Block #{block.id}")

                self.serial.set_gripper(closed=False)  # Energize relay to open NC jaws
                actual_z = self._press_cube_into_surface(
                    surface_z_mm=pick_z_down,
                    max_press_mm=self.config.pick_press_max_mm,
                    approach_offset_mm=self.config.pick_guard_offset_mm,
                    feedrate_press_mm_s=self.config.feedrate_pick_guard_mm_s,
                )

                early_limit_z = pick_z_down - self.config.early_force_z_margin_mm
                if actual_z < early_limit_z:
                    spike_g = abs(self.serial.last_press_delta_g)
                    self.log_event(
                        f"Misalignment [PICKING]: Early force spike ({spike_g:.1f} g) on Block #{block.id} "
                        f"at Z_down={actual_z:.2f} mm (expected >= {early_limit_z:.2f} mm) "
                        f"[attempt {attempt}/{max_attempts}]."
                    )
                    self.serial.move_to(z_mm=transit_z, feedrate_mm_s=self.config.feedrate_z_mm_s)
                    self.serial.set_gripper(closed=True)
                    if attempt >= max_attempts:
                        raise RuntimeError(
                            f"Pick failed after {max_attempts} consecutive early force spikes on Block #{block.id} "
                            f"({block.color.upper()}): last contact at Z_down={actual_z:.2f} mm (< {early_limit_z:.2f} mm)."
                        )
                    continue

                self.serial.set_gripper(closed=True)   # De-energize relay so NC solenoid grips block
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

            # Lift to consistent hover height (clear of Z=0 top stop) to weigh accurately before full transit retract
            self.serial.move_to(z_mm=weigh_z, feedrate_mm_s=self.config.feedrate_z_mm_s)
            if not self.mock_hardware:
                time.sleep(0.15)

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

            if transit_z < weigh_z - 0.1:
                self.serial.move_to(z_mm=transit_z, feedrate_mm_s=self.config.feedrate_z_mm_s)

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
                    min_safe_y_mm=self.config.servo_safe_y_mm,
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
            drop_my = max(self.config.servo_safe_y_mm, min(self.config.travel_y_mm, drop_my))
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

            # Travel horizontally at safe clearance height BEFORE descending vertically onto the stack
            pre_drop_z_down = min(transit_z, max(0.0, drop_z_down - 10.0))
            if self.serial.pose.z_mm > pre_drop_z_down + 0.1:
                self.serial.move_to(z_mm=pre_drop_z_down, feedrate_mm_s=self.config.feedrate_z_mm_s)
            self.serial.move_to(
                x_mm=drop_mx,
                y_mm=drop_my,
                z_mm=pre_drop_z_down,
                angle_deg=self.kinematics.servo_angle_for_plate_angle(0.0),
                feedrate_mm_s=self.config.feedrate_travel_mm_s,
            )
            self._press_cube_into_surface(
                surface_z_mm=drop_z_down,
                max_press_mm=self.config.drop_press_max_mm,
                extra_unload_g=max(0.0, float(block.mass_g or 0.0)),
                force_g=self.config.drop_press_force_g,
            )
            self.serial.set_gripper(closed=False)  # Release block while seated on surface
            self.serial.set_simulated_held_mass(None)
            if staged_blocker_id is None:
                self.planner.commit_stack_placement(zone_key)
            # Retract straight UP high enough to clear the newly heightened stack BEFORE any X/Y move
            post_drop_z_down = min(self._safe_transit_z_down(), max(0.0, drop_z_down - 10.0))
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
