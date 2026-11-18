"""
CLI Entry Point for the HUE-DINI Gantry Robot Tier 2 Orchestrator (Raspberry Pi Zero W).

Usage:
  # Run in simulation / mock-hardware mode (laptop or Pi testing without hardware):
  python -m pi_controller.main --mock-hardware --auto-start

  # Run on Raspberry Pi Zero W with physical UVC USB camera & USB-C ESP32:
  python -m pi_controller.main --port /dev/ttyUSB0 --camera-index 0
"""

import argparse
import logging
import os
from pathlib import Path
import signal
import sys
import threading
import time

try:
    from .config import GantryConfig
    from .state_machine import GantryStateMachine
except (ImportError, ValueError):
    # Support running script directly (e.g. python pi_controller/main.py)
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from pi_controller.config import GantryConfig
    from pi_controller.state_machine import GantryStateMachine


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="HUE-DINI 3-Axis Cartesian Block-Sorting Gantry Orchestrator"
    )
    parser.add_argument(
        "--mock-hardware",
        action="store_true",
        help="Simulate ESP32 serial motion and generate synthetic OpenCV camera frames.",
    )
    parser.add_argument(
        "--port",
        type=str,
        default="/dev/ttyUSB0",
        help="Serial port for ESP32 (default: /dev/ttyUSB0).",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate for ESP32 serial link (default: 115200).",
    )
    parser.add_argument(
        "--camera-index",
        type=int,
        default=0,
        help="V4L2 UVC USB camera index (default: 0).",
    )
    parser.add_argument(
        "--ws-host",
        type=str,
        default="0.0.0.0",
        help="WebSocket & HTTP dashboard bind address (default: 0.0.0.0).",
    )
    parser.add_argument(
        "--ws-port",
        type=int,
        default=8765,
        help="WebSocket & HTTP dashboard port (default: 8765).",
    )
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="Automatically trigger START_SCAN upon initialization.",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable DEBUG logging output.",
    )
    return parser.parse_args()


def print_cli_help() -> None:
    print(
        "\n--- HUE-DINI CLI Commands ---\n"
        "  start / scan   - Trigger 6-phase scan & sort workflow\n"
        "  pause          - Pause running motion/scan\n"
        "  resume         - Resume paused motion/scan\n"
        "  stop / estop   - Emergency stop (disable all steppers immediately)\n"
        "  reset          - Reset state machine to IDLE\n"
        "  home           - Execute 3-axis StallGuard4 sensorless homing\n"
        "  chime          - Play Windows 3.1 Tada! victory chime on X/Z steppers\n"
        "  status         - Show current state, gantry coordinates, and status\n"
        "  blocks         - List cataloged blocks\n"
        "  raw <cmd>      - Send manual command to ESP32 (e.g. 'raw STATUS', 'raw s1 500')\n"
        "  help / ?       - Show this command reference\n"
        "  quit / exit    - Cleanly shut down orchestrator\n"
        "-----------------------------\n"
    )


def interactive_cli(fsm: GantryStateMachine) -> None:
    """Provides an interactive REPL when run in a terminal."""
    time.sleep(0.5)
    print_cli_help()
    while fsm._running:
        try:
            line = input("huedini> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not line:
            continue

        parts = line.split(maxsplit=1)
        cmd = parts[0].lower()
        arg = parts[1] if len(parts) > 1 else ""

        if cmd in ("quit", "exit"):
            break
        elif cmd in ("start", "scan"):
            fsm._on_user_command("START_SCAN", {})
        elif cmd == "pause":
            fsm._on_user_command("PAUSE", {})
        elif cmd == "resume":
            fsm._on_user_command("RESUME", {})
        elif cmd in ("stop", "estop"):
            fsm._on_user_command("EMERGENCY_STOP", {})
        elif cmd == "reset":
            fsm._on_user_command("RESET", {})
        elif cmd == "home":
            if fsm.state in ("IDLE", "COMPLETE", "ERROR", "ESTOPPED"):
                fsm.serial.clear_emergency_stop()
                try:
                    if arg:
                        fsm.log_event(f"CLI manual homing '{arg}' triggered...")
                        replies = fsm.serial.send_command(f"home {arg}")
                        fsm.log_event(f"CLI manual homing completed: {', '.join(replies) if replies else 'OK'}")
                    else:
                        fsm.log_event("CLI manual homing triggered for all axes...")
                        fsm.serial.home_all()
                        fsm.log_event("CLI manual homing completed successfully.")
                except Exception as exc:
                    fsm.log_event(f"CLI manual homing failed: {exc}")
            else:
                print(f"[WARN] Cannot home while state is {fsm.state}. Pause or Stop first.")
        elif cmd in ("s1", "s2", "s3", "servo", "relay"):
            raw_line = f"{cmd} {arg}".strip()
            fsm._on_user_command("RAW_SERIAL", {"line": raw_line})
        elif cmd == "chime":
            fsm._on_user_command("PLAY_CHIME", {"tune": arg.upper() if arg else "TADA"})
        elif cmd == "status":
            pose = fsm.serial.pose
            print(
                f"State: {fsm.state} | Mode: {'MOCK' if fsm.mock_hardware else 'HARDWARE'}\n"
                f"Pose: X={pose.x_mm:.2f} mm, Y={pose.y_mm:.2f} mm, Z={pose.z_mm:.2f} mm, A={pose.angle_deg:.1f} deg\n"
                f"Gripper: {'CLOSED (NC)' if pose.grip_closed else 'OPEN'}\n"
                f"Calibrated: {fsm.kinematics.calibrated} (Kx={fsm.kinematics.kx_mm_per_px:.4f}, Ky={fsm.kinematics.ky_mm_per_px:.4f})\n"
                f"Blocks Cataloged: {len(fsm.planner.blocks)}"
            )
        elif cmd == "blocks":
            if not fsm.planner.blocks:
                print("No blocks cataloged yet.")
            else:
                print(f"{'ID':<4} | {'Color':<8} | {'Position (mm)':<18} | {'Angle':<7} | {'Mass':<10} | {'Status'}")
                print("-" * 65)
                for b in fsm.planner.blocks:
                    print(f"{b.id:<4} | {b.color.upper():<8} | ({b.x_mm:6.1f}, {b.y_mm:6.1f})   | {b.angle_deg:5.1f}  | {b.mass_g:5.1f}g    | {b.status}")
        elif cmd in ("raw", "tx"):
            if not arg:
                print("Usage: raw <command> (e.g. raw STATUS, raw s1 500)")
            else:
                fsm._on_user_command("RAW_SERIAL", {"line": arg})
        elif cmd in ("help", "?"):
            print_cli_help()
        else:
            print(f"Unknown command: '{cmd}'. Type 'help' for available commands.")


def main() -> None:
    args = parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    cfg = GantryConfig(
        serial_port=args.port,
        serial_baud=args.baud,
        camera_index=args.camera_index,
    )

    fsm = GantryStateMachine(
        config=cfg,
        mock_hardware=args.mock_hardware,
        ws_host=args.ws_host,
        ws_port=args.ws_port,
    )

    def handle_sigint(sig, frame):
        logging.info("Shutting down GantryStateMachine...")
        fsm.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)
    fsm.start(auto_start_scan=args.auto_start)

    # If connected to an interactive TTY terminal, run CLI REPL; otherwise sleep
    if sys.stdin.isatty():
        try:
            interactive_cli(fsm)
        finally:
            logging.info("Exiting CLI...")
            fsm.stop()
    else:
        while fsm._running:
            time.sleep(0.5)


if __name__ == "__main__":
    main()
