"""
Diagnostic Run Report Recorder for HUE-DINI Gantry Robot.
Records comprehensive diagnostics for every automated run session:
  - Homing & ArUco fiducial calibration (origin, auto-scale Kx/Ky, camera skew angle)
  - Scan waypoints, raw detections, and cataloged virtual world blocks
  - Fine alignment verification deltas, contour areas, and residual offsets
  - Pick, tare, weigh (HX711 mass & class), and drop-off stack actions
  - Full chronological motor command history (TX / RX)
  - Complete event logs and error traces
Persists reports to JSON and Markdown on disk, and provides API access.
"""

from dataclasses import asdict, dataclass, field
from datetime import datetime
import json
import logging
from pathlib import Path
import threading
import time
from typing import Any, Dict, List, Optional

logger = logging.getLogger("gantry.reports")


@dataclass
class CalibrationRecord:
    timestamp: float = 0.0
    calibrated: bool = False
    origin_calibrated: bool = False
    marker_id: Optional[int] = None
    u_px: Optional[float] = None
    v_px: Optional[float] = None
    marker_angle_deg: Optional[float] = None
    cam_skew_deg: float = 0.0
    kx_mm_per_px: float = 0.25
    ky_mm_per_px: float = 0.25
    cam_origin_x_mm: float = 32.5
    cam_origin_y_mm: float = 48.1
    offset_x_mm: float = 0.0
    offset_y_mm: float = -54.1


@dataclass
class ScanWaypointRecord:
    waypoint_index: int
    motor_x_mm: float
    motor_y_mm: float
    motor_z_mm: float
    raw_detections: List[Dict[str, Any]] = field(default_factory=list)
    unique_blocks_count: int = 0


@dataclass
class PickActionRecord:
    block_id: int
    color: str
    target_plate_x: float
    target_plate_y: float
    target_angle_deg: float
    # Phase 5A: Fine alignment check
    fine_align_checked: bool = False
    fine_align_found: bool = False
    fine_u_px: Optional[float] = None
    fine_v_px: Optional[float] = None
    fine_contour_area_px: Optional[float] = None
    fine_residual_err_mm: Optional[float] = None
    refined_plate_x: Optional[float] = None
    refined_plate_y: Optional[float] = None
    refined_angle_deg: Optional[float] = None
    # Motor targets
    camera_motor_x: float = 0.0
    camera_motor_y: float = 0.0
    gripper_motor_x: float = 0.0
    gripper_motor_y: float = 0.0
    servo_angle_deg: float = 0.0
    # Phase 5B: Weighing
    tare_success: bool = False
    measured_mass_g: Optional[float] = None
    mass_class: Optional[str] = None
    # Phase 6: Drop-off
    drop_plate_x: float = 0.0
    drop_plate_y: float = 0.0
    drop_z_down_mm: float = 0.0
    stack_zone: str = ""
    status: str = "PENDING"  # PENDING, SUCCESS, FAILED
    error: Optional[str] = None


@dataclass
class MotorCommandRecord:
    timestamp: float
    command: str
    replies: List[str] = field(default_factory=list)
    duration_ms: float = 0.0


class RunReportRecorder:
    """Manages active run session logging and persists structured diagnostics."""

    def __init__(self, reports_dir: Optional[Path] = None):
        if reports_dir is None:
            # Default to gantry_robot/reports/
            self.reports_dir = Path(__file__).resolve().parent.parent / "reports"
        else:
            self.reports_dir = Path(reports_dir)

        try:
            self.reports_dir.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
            logger.warning("Could not create reports directory %s: %exc", self.reports_dir, exc)

        self._lock = threading.Lock()
        self._current_report: Optional[Dict[str, Any]] = None
        self._history_cache: List[Dict[str, Any]] = []
        self._reload_history()

    def _reload_history(self) -> None:
        """Loads metadata for recent reports from disk."""
        if not self.reports_dir.exists():
            return
        summaries: List[Dict[str, Any]] = []
        for file in sorted(self.reports_dir.glob("run_*.json"), reverse=True)[:50]:
            try:
                data = json.loads(file.read_text(encoding="utf-8"))
                summaries.append({
                    "run_id": data.get("run_id", file.stem),
                    "start_time": data.get("start_time", ""),
                    "duration_s": data.get("duration_s", 0.0),
                    "status": data.get("status", "UNKNOWN"),
                    "blocks_cataloged": len(data.get("cataloged_blocks", [])),
                    "blocks_picked": len([p for p in data.get("pick_actions", []) if p.get("status") == "SUCCESS"]),
                    "cam_skew_deg": data.get("calibration", {}).get("cam_skew_deg", 0.0),
                    "file_name": file.name,
                })
            except Exception:
                continue
        self._history_cache = summaries

    def start_new_run(self, mock_hardware: bool = False) -> str:
        with self._lock:
            now = datetime.now()
            run_id = f"run_{now.strftime('%Y%m%d_%H%M%S')}"
            self._current_report = {
                "run_id": run_id,
                "start_time": now.isoformat(),
                "start_epoch": time.time(),
                "end_time": None,
                "duration_s": 0.0,
                "mock_hardware": mock_hardware,
                "status": "RUNNING",
                "calibration": asdict(CalibrationRecord()),
                "scan_waypoints": [],
                "cataloged_blocks": [],
                "pick_actions": [],
                "motor_commands": [],
                "event_logs": [],
                "error": None,
            }
            logger.info("Started diagnostic run recorder: %s", run_id)
            return run_id

    def record_calibration(
        self,
        calibrated: bool,
        origin_calibrated: bool,
        marker_id: Optional[int],
        u_px: Optional[float],
        v_px: Optional[float],
        marker_angle_deg: Optional[float],
        cam_skew_deg: float,
        kx: float,
        ky: float,
        cam_origin_x: float,
        cam_origin_y: float,
        offset_x: float,
        offset_y: float,
    ) -> None:
        with self._lock:
            if not self._current_report:
                return
            rec = CalibrationRecord(
                timestamp=time.time(),
                calibrated=calibrated,
                origin_calibrated=origin_calibrated,
                marker_id=marker_id,
                u_px=round(u_px, 1) if u_px is not None else None,
                v_px=round(v_px, 1) if v_px is not None else None,
                marker_angle_deg=round(marker_angle_deg, 1) if marker_angle_deg is not None else None,
                cam_skew_deg=round(cam_skew_deg, 2),
                kx_mm_per_px=round(kx, 4),
                ky_mm_per_px=round(ky, 4),
                cam_origin_x_mm=round(cam_origin_x, 2),
                cam_origin_y_mm=round(cam_origin_y, 2),
                offset_x_mm=round(offset_x, 2),
                offset_y_mm=round(offset_y, 2),
            )
            self._current_report["calibration"] = asdict(rec)

    def record_scan_waypoint(
        self,
        waypoint_idx: int,
        gx: float,
        gy: float,
        gz: float,
        raw_detections: List[Dict[str, Any]],
        blocks_count: int,
    ) -> None:
        with self._lock:
            if not self._current_report:
                return
            rec = ScanWaypointRecord(
                waypoint_index=waypoint_idx,
                motor_x_mm=round(gx, 1),
                motor_y_mm=round(gy, 1),
                motor_z_mm=round(gz, 1),
                raw_detections=raw_detections,
                unique_blocks_count=blocks_count,
            )
            self._current_report["scan_waypoints"].append(asdict(rec))

    def record_cataloged_blocks(self, blocks: List[Dict[str, Any]]) -> None:
        with self._lock:
            if not self._current_report:
                return
            self._current_report["cataloged_blocks"] = list(blocks)

    def record_pick_action(self, action: PickActionRecord) -> None:
        with self._lock:
            if not self._current_report:
                return
            self._current_report["pick_actions"].append(asdict(action))

    def record_motor_command(
        self,
        cmd: str,
        replies: List[str],
        duration_ms: float = 0.0,
    ) -> None:
        with self._lock:
            if not self._current_report:
                return
            rec = MotorCommandRecord(
                timestamp=round(time.time(), 3),
                command=cmd,
                replies=replies,
                duration_ms=round(duration_ms, 1),
            )
            self._current_report["motor_commands"].append(asdict(rec))

    def record_event_log(self, message: str) -> None:
        with self._lock:
            if not self._current_report:
                return
            self._current_report["event_logs"].append(message)

    def finish_run(self, status: str = "COMPLETED", error: Optional[str] = None) -> Optional[str]:
        with self._lock:
            if not self._current_report:
                return None
            now = datetime.now()
            start_epoch = self._current_report.get("start_epoch", time.time())
            duration_s = round(time.time() - start_epoch, 2)

            self._current_report["end_time"] = now.isoformat()
            self._current_report["duration_s"] = duration_s
            self._current_report["status"] = status
            if error:
                self._current_report["error"] = error

            run_id = self._current_report["run_id"]
            self._save_report_files(self._current_report)

            # Update cache summary
            summary = {
                "run_id": run_id,
                "start_time": self._current_report["start_time"],
                "duration_s": duration_s,
                "status": status,
                "blocks_cataloged": len(self._current_report.get("cataloged_blocks", [])),
                "blocks_picked": len([p for p in self._current_report.get("pick_actions", []) if p.get("status") == "SUCCESS"]),
                "cam_skew_deg": self._current_report.get("calibration", {}).get("cam_skew_deg", 0.0),
                "file_name": f"{run_id}.json",
            }
            self._history_cache.insert(0, summary)
            if len(self._history_cache) > 50:
                self._history_cache.pop()

            report_copy = self._current_report
            self._current_report = None
            logger.info("Saved diagnostic run report: %s (status: %s, duration: %.1fs)", run_id, status, duration_s)
            return run_id

    def _save_report_files(self, report: Dict[str, Any]) -> None:
        try:
            self.reports_dir.mkdir(parents=True, exist_ok=True)
            run_id = report["run_id"]
            json_path = self.reports_dir / f"{run_id}.json"
            json_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

            # Also generate human-readable Markdown summary
            md_path = self.reports_dir / f"{run_id}.md"
            md_text = self._generate_markdown_summary(report)
            md_path.write_text(md_text, encoding="utf-8")
        except Exception as exc:
            logger.error("Failed to write report files to %s: %s", self.reports_dir, exc)

    def _generate_markdown_summary(self, r: Dict[str, Any]) -> str:
        cal = r.get("calibration", {})
        lines = [
            f"# HUE-DINI Gantry Robot Run Report: `{r.get('run_id')}`",
            "",
            f"- **Status**: `{r.get('status')}`",
            f"- **Start Time**: {r.get('start_time')}",
            f"- **Duration**: {r.get('duration_s')} seconds",
            f"- **Hardware Mode**: `{'MOCK' if r.get('mock_hardware') else 'PHYSICAL ESP32'}`",
            "",
            "## 1. Homing & Fiducial Calibration",
            f"- **Marker ID**: `{cal.get('marker_id')}`",
            f"- **Detected Pixel Position**: `({cal.get('u_px')}, {cal.get('v_px')}) px`",
            f"- **Marker Angle**: `{cal.get('marker_angle_deg')}°`",
            f"- **Camera Skew Angle**: `{cal.get('cam_skew_deg')}°` (Physical Tilt Compensation)",
            f"- **Pixel Scale Kx / Ky**: `{cal.get('kx_mm_per_px')} / {cal.get('ky_mm_per_px')} mm/px`",
            f"- **Calibrated Camera Origin**: `({cal.get('cam_origin_x_mm')}, {cal.get('cam_origin_y_mm')}) mm` wrt plate",
            f"- **Gripper Offset**: `({cal.get('offset_x_mm')}, {cal.get('offset_y_mm')}) mm`",
            "",
            "## 2. Global Course Scan & Block Catalog",
            f"- **Total Waypoints Visited**: {len(r.get('scan_waypoints', []))}",
            f"- **Cataloged Blocks**: {len(r.get('cataloged_blocks', []))}",
            "",
            "| ID | Color | World Plate (X, Y) mm | Angle | Views |",
            "|---|---|---|---|---|",
        ]
        for b in r.get("cataloged_blocks", []):
            lines.append(f"| #{b.get('id')} | {b.get('color').upper()} | ({b.get('x_mm'):.1f}, {b.get('y_mm'):.1f}) | {b.get('angle_deg'):.1f}° | {b.get('confidence_views', 1)} |")

        lines.extend([
            "",
            "## 3. Pick, Fine Alignment & Weighing Actions",
            "",
            "| Block | Fine Align Found | Contour Area | Fine Residual Err | Measured Mass | Mass Class | Stack Target | Result |",
            "|---|---|---|---|---|---|---|---|",
        ])
        for p in r.get("pick_actions", []):
            found_str = "YES" if p.get("fine_align_found") else "NO (Coarse Fallback)"
            area_str = f"{p.get('fine_contour_area_px'):.0f} px" if p.get("fine_contour_area_px") else "N/A"
            err_str = f"{p.get('fine_residual_err_mm'):.2f} mm" if p.get("fine_residual_err_mm") is not None else "N/A"
            mass_str = f"{p.get('measured_mass_g'):.2f} g" if p.get("measured_mass_g") is not None else "N/A"
            lines.append(
                f"| #{p.get('block_id')} ({p.get('color').upper()}) | {found_str} | {area_str} | {err_str} | {mass_str} | `{p.get('mass_class', '')}` | ({p.get('drop_plate_x'):.1f}, {p.get('drop_plate_y'):.1f}) | `{p.get('status')}` |"
            )

        lines.extend([
            "",
            "## 4. Hardware Command Summary",
            f"- Total serial commands dispatched: {len(r.get('motor_commands', []))}",
            "",
            "## 5. Event Logs",
            "```text",
        ])
        for log in r.get("event_logs", []):
            lines.append(log)
        lines.extend(["```", ""])
        return "\n".join(lines)

    def get_recent_reports(self) -> List[Dict[str, Any]]:
        with self._lock:
            return list(self._history_cache)

    def get_report(self, run_id: str) -> Optional[Dict[str, Any]]:
        with self._lock:
            if self._current_report and self._current_report.get("run_id") == run_id:
                return dict(self._current_report)
            path = self.reports_dir / f"{run_id}.json"
            if path.exists():
                try:
                    return json.loads(path.read_text(encoding="utf-8"))
                except Exception:
                    pass
            return None

    def get_latest_report(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            if self._current_report:
                return dict(self._current_report)
            if self._history_cache:
                latest_id = self._history_cache[0]["run_id"]
                return self.get_report(latest_id)
            return None
