"""
Core 6-Phase Operational State Machine for the HUE-DINI Gantry Robot:
  Phase 1: IDLE         — Wait for START_SCAN trigger from PC GUI or ESP32 button
  Phase 2: CALIBRATING  — StallGuard4 homing + Eye-in-Hand ArUco Auto-Scale routine
  Phase 3: SCANNING     — Multi-point stop-and-capture global course scan
  Phase 4: PLANNING     — Deduplicated block catalog & clearance-based pick order
  Phase 5: PICKING & WEIGHING — Fine visual alignment, angle match, NC solenoid grip, HX711 weigh
  Phase 6: STACKING     — Sort by color & mass (rejecting out-of-spec weights) and stack
"""

from collections import deque
import logging
import queue
import threading
import time
from typing import Deque, Optional

from .config import GantryConfig
from .kinematics import BlockRecord, EyeInHandKinematics, WorldCatalogAndPlanner
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

        self.kinematics = EyeInHandKinematics(
            frame_width=config.frame_width,
            frame_height=config.frame_height,
            kx_mm_per_px=config.default_kx_mm_per_px,
            ky_mm_per_px=config.default_ky_mm_per_px,
            offset_x_mm=config.camera_to_gripper_offset_x_mm,
            offset_y_mm=config.camera_to_gripper_offset_y_mm,
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
        )
        self.telemetry = TelemetryServer(
            host=ws_host,
            port=ws_port,
            command_callback=self._on_user_command,
        )

        self._start_requested = False
        self._pause_requested = False
        self._estop_active = False
        self._running = True
        self._live_stream_enabled = True
        self._live_cv_enabled = False  # Eco stream mode by default in IDLE to save ~70% CPU on Pi Zero
        self._current_mass_g: Optional[float] = None
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
        self.publish_telemetry()

    def start(self, auto_start_scan: bool = False) -> None:
        self.telemetry.start()
        self.vision.open_camera()
        self.serial.connect()

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
            "steps_per_mm": {
                "x": round(self.config.steps_per_mm_x, 2),
                "y": round(self.config.steps_per_mm_y, 2),
                "z": round(self.config.steps_per_mm_z, 2),
            },
            "calibration": {
                "calibrated": self.kinematics.calibrated,
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

    def _on_hardware_event(self, event_name: str) -> None:
        if event_name == "START_SCAN":
            self.log_event("Hardware START button pressed on ESP32.")
            self._start_requested = True

    def _on_user_command(self, cmd: str, data: dict) -> None:
        """
        Fast non-blocking command receiver called by the WebSocket server thread.
        ESTOP is handled immediately for safety; all other motion and configuration
        commands are queued to a background worker thread so the WebSocket server never blocks.
        """
        if cmd == "EMERGENCY_STOP":
            self._estop_active = True
            self._start_requested = False
            self.state = "ESTOPPED"
            self.serial.trigger_emergency_stop()
            self.log_event("EMERGENCY STOP triggered! All steppers disabled.")
            # Clear any pending commands in queue
            while not self._cmd_queue.empty():
                try:
                    self._cmd_queue.get_nowait()
                    self._cmd_queue.task_done()
                except queue.Empty:
                    break
            self.publish_telemetry()
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
                self.log_event("Received START_SCAN trigger from Tier 1 Dashboard.")
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
            self._current_mass_g = None
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
                self.serial.send_command("SLEEP")
                self.log_event("Motors unpowered (FREE). Gantry can be moved freely by hand.")
            except Exception as exc:
                self.log_event(f"Free motors failed: {exc}")
        elif cmd in ("WAKE_MOTORS", "SETUP_MOTORS"):
            try:
                self.serial.send_command("SETUP")
                self.log_event("Motors re-energized (SETUP). Drivers ready.")
            except Exception as exc:
                self.log_event(f"Wake motors failed: {exc}")
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
                self._current_mass_g = 0.00
                self.log_event("HX711 scale tared (zeroed) successfully.")
            except Exception as exc:
                self.log_event(f"Tare scale failed: {exc}")
            self.publish_telemetry()
        elif cmd in ("WEIGH_BLOCK", "SAMPLE_MASS"):
            try:
                measured = self.serial.weigh_block()
                self._current_mass_g = round(measured, 2)
                cls = self.config.classify_mass(measured)
                self.log_event(f"HX711 sample: {measured:.2f} g -> Class [{cls.upper()}].")
            except Exception as exc:
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
                self._run_full_cycle()
            except Exception as exc:
                if not self._estop_active:
                    self.state = "ERROR"
                    exc_str = str(exc)
                    if "CRASH_DETECTED" in exc_str or "STALL" in exc_str:
                        self.kinematics.calibrated = False
                        self.log_event(f"CRASH / STALL DETECTED on end-stop! {exc_str}. Motors powered down safely. Re-home before continuing.")
                    else:
                        self.log_event(f"Workflow aborted: {exc}")

    def _run_full_cycle(self) -> None:
        # ==============================================================
        # PHASE 2: Homing & Eye-in-Hand Self-Calibration (CALIBRATING)
        # ==============================================================
        self.state = "CALIBRATING"
        self.log_event("Phase 2 [CALIBRATING]: Homing Z, X, Y axes via TMC2209 StallGuard4...")
        self._check_pause_or_abort()
        self.serial.home_all()

        # Move to base fiducial marker at scanning height Z_scan
        fx, fy = self.config.fiducial_base_x_mm, self.config.fiducial_base_y_mm
        z_scan = self.config.z_scan_mm
        self.log_event(f"Moving camera above base ArUco fiducial at ({fx:.1f}, {fy:.1f}, {z_scan:.1f}) mm...")
        self.serial.move_to(x_mm=fx, y_mm=fy, z_mm=z_scan, angle_deg=0.0)
        self._check_pause_or_abort()

        # Stop-and-Capture Frame A
        frame_a = self.vision.capture_stationary_frame(
            self.serial.pose.x_mm, self.serial.pose.y_mm, self.serial.pose.z_mm
        )
        fid_a = self.vision.detect_aruco_fiducial(frame_a)

        # Step a known physical distance (+dx, +dy) and capture Frame B
        step_x = self.config.calibration_step_x_mm
        step_y = self.config.calibration_step_y_mm
        self.serial.move_to(x_mm=fx + step_x, y_mm=fy + step_y, z_mm=z_scan)
        self._check_pause_or_abort()

        frame_b = self.vision.capture_stationary_frame(
            self.serial.pose.x_mm, self.serial.pose.y_mm, self.serial.pose.z_mm
        )
        fid_b = self.vision.detect_aruco_fiducial(frame_b)

        if fid_a is not None and fid_b is not None:
            try:
                kx, ky = self.kinematics.compute_auto_scale(
                    fid_a[0], fid_a[1], fid_b[0], fid_b[1], step_x, step_y
                )
                self.log_event(
                    f"Auto-Scale calibrated! Kx={kx:.4f} mm/px, Ky={ky:.4f} mm/px at Z={z_scan:.0f} mm."
                )
            except Exception as exc:
                self.kinematics.calibrated = True
                self.log_event(
                    f"Auto-scale calculation failed ({exc}); falling back to default scale Kx={self.kinematics.kx_mm_per_px:.4f} mm/px."
                )
        else:
            self.kinematics.calibrated = True
            self.log_event(
                f"Fiducial not in view; using default scale Kx={self.kinematics.kx_mm_per_px:.4f} mm/px."
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

            for det in detections:
                wx, wy = self.kinematics.pixel_to_world_mm(
                    det.u_px, det.v_px, gx, gy, z_scan, z_scan
                )
                self.planner.merge_detection(det.color, wx, wy, det.angle_deg)

            self.log_event(
                f"Scan waypoint {idx}/{len(self.config.scan_grid_points_mm)} ({gx:.0f}, {gy:.0f} mm): "
                f"{len(detections)} detections ({len(self.planner.blocks)} unique blocks cataloged)."
            )

        # ==============================================================
        # PHASE 4: Path & Sequence Optimization (PLANNING)
        # ==============================================================
        self.state = "PLANNING"
        self._check_pause_or_abort()
        pick_queue = self.planner.compute_pick_sequence(
            self.serial.pose.x_mm, self.serial.pose.y_mm
        )
        self.log_event(
            f"Phase 4 [PLANNING]: Computed clearance-optimized pick order for {len(pick_queue)} blocks."
        )

        # ==============================================================
        # PHASE 5 & 6: Pick, Align, Weigh (PICKING / WEIGHING) & Sort/Stack (STACKING)
        # ==============================================================
        for block in pick_queue:
            self._check_pause_or_abort()
            self._execute_pick_weigh_and_stack(block)

        # Return to home park position: retract Z first to avoid collisions
        self.serial.move_to(z_mm=self.config.z_scan_mm, feedrate_mm_s=self.config.feedrate_z_mm_s)
        self.serial.move_to(x_mm=20.0, y_mm=20.0, angle_deg=0.0)
        self.state = "COMPLETE"
        self.log_event(
            f"All {len(pick_queue)} blocks processed and sorted! Returning to IDLE/COMPLETE."
        )

    def _execute_pick_weigh_and_stack(self, block: BlockRecord) -> None:
        # --- PHASE 5A: Move Camera Above Block & Stop-and-Check Fine Alignment ---
        self.state = "PICKING"
        block.status = "PICKING"
        self.log_event(
            f"Phase 5 [PICKING]: Moving camera above Block #{block.id} ({block.color.upper()}) "
            f"at ({block.x_mm:.1f}, {block.y_mm:.1f}) mm..."
        )
        self.serial.move_to(
            x_mm=block.x_mm,
            y_mm=block.y_mm,
            z_mm=self.config.z_hover_mm,
            angle_deg=0.0,
        )
        self._check_pause_or_abort()

        # Capture single verification frame for residual positional error
        v_frame = self.vision.capture_stationary_frame(
            self.serial.pose.x_mm, self.serial.pose.y_mm, self.serial.pose.z_mm
        )
        v_dets = [d for d in self.vision.detect_blocks_hsv(v_frame) if d.color == block.color]
        if v_dets:
            # Pick detection closest to image center
            cx, cy = self.config.frame_width / 2.0, self.config.frame_height / 2.0
            best = min(v_dets, key=lambda d: (d.u_px - cx) ** 2 + (d.v_px - cy) ** 2)
            refined_x, refined_y = self.kinematics.pixel_to_world_mm(
                best.u_px,
                best.v_px,
                self.serial.pose.x_mm,
                self.serial.pose.y_mm,
                self.serial.pose.z_mm,
                self.config.z_scan_mm,
            )
            err_mm = ((refined_x - block.x_mm) ** 2 + (refined_y - block.y_mm) ** 2) ** 0.5
            block.x_mm, block.y_mm, block.angle_deg = refined_x, refined_y, best.angle_deg
            self.log_event(
                f"Fine alignment check on Block #{block.id}: residual offset {err_mm:.2f} mm, "
                f"angle {block.angle_deg:.1f} deg."
            )

        # Translate camera coordinate to gripper center target & rotate 9g servo to block angle
        grip_x, grip_y = self.kinematics.gripper_target_for_block(block.x_mm, block.y_mm)
        self.serial.move_to(
            x_mm=grip_x,
            y_mm=grip_y,
            z_mm=self.config.z_hover_mm,
            angle_deg=block.angle_deg,
            feedrate_mm_s=self.config.feedrate_fine_mm_s,
        )

        # Tare HX711 load cell before picking, open NC solenoid gripper (GRIP 0), descend, close (GRIP 1)
        try:
            self.serial.tare_scale()
            self._current_mass_g = 0.00
        except Exception as exc:
            self.log_event(f"Notice: Tare scale skipped ({exc}).")

        self.serial.set_gripper(closed=False)  # Energize relay to open NC jaws
        self.serial.move_to(z_mm=self.config.z_pick_mm, feedrate_mm_s=self.config.feedrate_z_mm_s)
        self.serial.set_gripper(closed=True)   # De-energize relay so NC solenoid grips block

        # Mark closest ground-truth block as picked in synthetic renderer if in mock mode
        if self.mock_hardware:
            unpicked = [sb for sb in self.vision.simulated_blocks if not sb.picked]
            if unpicked:
                nearest_sb = min(
                    unpicked,
                    key=lambda sb: (sb.x_mm - block.x_mm) ** 2 + (sb.y_mm - block.y_mm) ** 2,
                )
                nearest_sb.picked = True
                self.serial.set_simulated_held_mass(nearest_sb.mass_g)

        # Lift to hover height
        self.serial.move_to(z_mm=self.config.z_hover_mm, feedrate_mm_s=self.config.feedrate_z_mm_s)

        # --- PHASE 5B: Weigh Block on HX711 Load Cell & Classify Mass ---
        self.state = "WEIGHING"
        self._check_pause_or_abort()
        try:
            measured_mass_g = self.serial.weigh_block()
        except Exception as exc:
            self.log_event(f"Notice: Load cell sample failed ({exc}); assigning default nominal mass 10.0g.")
            measured_mass_g = 10.0

        self._current_mass_g = round(measured_mass_g, 2)
        block.mass_g = measured_mass_g
        block.mass_class = self.config.classify_mass(measured_mass_g)
        block.status = "WEIGHED"
        self.log_event(
            f"Phase 5 [WEIGHING]: Block #{block.id} ({block.color.upper()}) mass = {measured_mass_g:.2f} g "
            f"-> class [{block.mass_class.upper()}]."
        )

        # --- PHASE 6: Sort & Stack (Drop-off at X=224.5, Y=44.5, Z=+12.6 mm -> Z_down=64.11 mm) ---
        self.state = "STACKING"
        self._check_pause_or_abort()
        drop_x, drop_y, drop_z_down, is_rejected = self.planner.get_stack_target(
            color=block.color,
            mass_class=block.mass_class,
            stack_zones_mm=self.config.stack_zones_mm,
            base_pick_z_down_mm=self.config.z_dropoff_base_down_mm,
        )
        zone_label = (
            "REJECT ZONE"
            if is_rejected
            else f"{block.mass_class.upper()} / {block.color.upper()} STACK"
        )
        self.log_event(
            f"Phase 6 [STACKING]: Routing Block #{block.id} to {zone_label} "
            f"at ({drop_x:.1f}, {drop_y:.1f}, Z_down={drop_z_down:.1f}) mm..."
        )

        # Travel at hover height (Z_down = z_hover_mm, closer to 0=top) to target stack coordinate
        travel_z_down = min(self.config.z_hover_mm, max(0.0, drop_z_down - 25.0))
        self.serial.move_to(
            x_mm=drop_x,
            y_mm=drop_y,
            z_mm=travel_z_down,
            angle_deg=0.0,
            feedrate_mm_s=self.config.feedrate_travel_mm_s,
        )
        self.serial.move_to(z_mm=drop_z_down, feedrate_mm_s=self.config.feedrate_z_mm_s)
        self.serial.set_gripper(closed=False)  # Release block
        self.serial.set_simulated_held_mass(None)
        self.serial.move_to(z_mm=travel_z_down, feedrate_mm_s=self.config.feedrate_z_mm_s)
        self.serial.set_gripper(closed=True)   # Return NC solenoid to de-energized rest state
        self._current_mass_g = 0.00

        block.x_mm = drop_x
        block.y_mm = drop_y
        block.angle_deg = 0.0
        block.status = "REJECTED" if is_rejected else "STACKED"
        self.publish_telemetry()
