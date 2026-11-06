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
import signal
import sys
import time

from .config import GantryConfig
from .state_machine import GantryStateMachine


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
    return parser.parse_args()


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    args = parse_args()

    cfg = GantryConfig(
        serial_port=args.port,
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

    while True:
        time.sleep(0.5)


if __name__ == "__main__":
    main()
