#!/usr/bin/env python3
"""
TMC2209 Sensorless Homing StallGuard Sensitivity Sweeper for HUE-DINI Gantry Robot.

Usage examples:
  # Sweep X-axis (S1) backwards from sensitivity 25 down to 1:
  python3 sweep_homing.py --axis s1 --dir - --start 25 --end 1 --step 2 --backoff 1000

  # Sweep Y-axis (S2) forward (default home direction for Y) from 15 down to 1:
  python3 sweep_homing.py --axis s2 --dir + --start 15 --end 1 --step 1 --backoff 200

  # Interactive mode (prompts user after each attempt to record if it reached the end stop):
  python3 sweep_homing.py --axis s1 --dir - --interactive
"""

import argparse
import sys
import time
from typing import Optional, Tuple

try:
    import serial
except ImportError:
    print("[ERROR] pyserial is required. Run 'pip install pyserial'.")
    sys.exit(1)


class StallGuardSweeper:
    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None

    def connect(self) -> None:
        print(f"Connecting to ESP32 on {self.port} @ {self.baudrate} baud...")
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        time.sleep(1.5)  # Allow reset on DTR toggle
        self.ser.reset_input_buffer()
        print("Connected. Running initial 'setup'...")
        res = self.send_command("setup", wait_sec=2.0)
        print("Setup reply:\n  " + "\n  ".join(res))

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            try:
                self.send_command("sleep", wait_sec=1.0)
            except Exception:
                pass
            self.ser.close()
            print("Serial port closed. Steppers placed in sleep mode.")

    def send_command(self, cmd: str, wait_sec: float = 0.5, stop_on_marker: Optional[list] = None) -> list[str]:
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial port not open.")

        cmd_clean = cmd.strip()
        self.ser.write((cmd_clean + "\n").encode("ascii"))
        self.ser.flush()

        lines: list[str] = []
        start = time.time()
        last_rx = start

        while time.time() - start < wait_sec:
            raw = self.ser.readline()
            if raw:
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    lines.append(line)
                    last_rx = time.time()
                    if stop_on_marker and any(m in line for m in stop_on_marker):
                        break
                    if line == "OK":
                        break
            else:
                # If no data for 250ms and we have some response, don't wait forever
                if lines and (time.time() - last_rx > 0.25) and not stop_on_marker:
                    break
        return lines

    def move_relative(self, axis: str, steps: int) -> bool:
        """Moves relative steps away from the stop to clear the end stop before homing."""
        print(f"-> Backing off {axis} by {steps} steps...")
        replies = self.send_command(f"{axis} {steps}", wait_sec=10.0, stop_on_marker=["move complete", "stall detected"])
        for r in replies:
            if "move complete" in r:
                return True
            if "stall detected" in r:
                print(f"   [WARN] Stall during backoff: {r}")
                return False
        return True

    def test_homing(
        self, axis: str, direction: str, sensitivity: int, timeout_s: float = 15.0
    ) -> Tuple[str, float, Optional[int], Optional[int]]:
        """
        Sets homing sensitivity and triggers home command.
        Returns (status, elapsed_seconds, sg_result, tstep).
        """
        # Set sensitivity
        self.send_command(f"{axis} hs{sensitivity}", wait_sec=1.0)

        # Build home command: 'home s1 -' or 'home s1'
        home_cmd = f"home {axis}"
        if direction.strip() == "-":
            home_cmd += " -"

        print(f"\n--- Testing {axis.upper()} @ SGTHRS={sensitivity} ({home_cmd}) ---")
        t0 = time.time()
        replies = self.send_command(
            home_cmd,
            wait_sec=timeout_s,
            stop_on_marker=["Position set to 0.", "timed out", "cancelled", "[ERROR]"],
        )
        elapsed = time.time() - t0

        outcome = "UNKNOWN"
        sg_res = None
        tstep = None

        for line in replies:
            print(f"   ESP32: {line}")
            if "homed" in line and "Position set to 0." in line:
                outcome = "HOMED"
                # Parse SG_RESULT=..., TSTEP=...
                try:
                    if "SG_RESULT=" in line:
                        part = line.split("SG_RESULT=")[1]
                        sg_res = int(part.split(",")[0].split(")")[0].strip())
                    if "TSTEP=" in line:
                        part = line.split("TSTEP=")[1]
                        tstep = int(part.split(")")[0].split(".")[0].strip())
                except Exception:
                    pass
            elif "timed out" in line:
                outcome = "TIMEOUT"
            elif "stall detected" in line:
                outcome = "PREMATURE_STALL"
            elif "cancelled" in line:
                outcome = "CANCELLED"

        return outcome, round(elapsed, 2), sg_res, tstep


def main():
    parser = argparse.ArgumentParser(description="Sweep TMC2209 homing sensitivity on ESP32.")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--axis", type=str, default="s1", choices=["s1", "s2", "s3"], help="Axis (s1=X, s2=Y, s3=Z)")
    parser.add_argument("--dir", type=str, default="-", choices=["+", "-"], help="Homing direction: '-' backward, '+' forward")
    parser.add_argument("--start", type=int, default=25, help="Starting sensitivity (default: 25)")
    parser.add_argument("--end", type=int, default=1, help="Ending sensitivity (default: 1)")
    parser.add_argument("--step", type=int, default=2, help="Step decrement per test (default: 2)")
    parser.add_argument("--backoff", type=int, default=1000, help="Steps to back off away from stop before each test")
    parser.add_argument("--speed", type=float, default=None, help="Optional homing speed override in steps/s")
    parser.add_argument("--interactive", action="store_true", help="Prompt after each test for user feedback")

    args = parser.parse_args()

    # Direction sanity check based on project spec:
    # S1 (X): homeReverse = true (-)
    # S2 (Y): homeReverse = false (+)
    # S3 (Z): homeReverse = true (-)
    if args.axis == "s2" and args.dir == "-":
        print("[NOTE] Default homing direction for S2 (Y) is '+' (forward).")

    sweeper = StallGuardSweeper(port=args.port)
    try:
        sweeper.connect()

        if args.speed is not None:
            print(f"Setting homing speed on {args.axis} to {args.speed} steps/s...")
            sweeper.send_command(f"{args.axis} hv{int(args.speed)}")

        # Step direction for backoff is opposite to homing direction
        # If homing is '-', backoff with '+' steps
        backoff_steps = abs(args.backoff) if args.dir == "-" else -abs(args.backoff)

        sensitivities = list(range(args.start, args.end - 1, -abs(args.step)))
        print(f"\n=======================================================")
        print(f" Starting StallGuard Sweep for {args.axis.upper()} ({args.dir})")
        print(f" Range: {sensitivities}")
        print(f" Backoff per cycle: {backoff_steps} steps")
        print(f" Note: Higher SGTHRS = More sensitive (trips earlier)")
        print(f"       Lower SGTHRS  = Less sensitive (needs more stall force)")
        print(f"=======================================================\n")

        results = []

        for sens in sensitivities:
            # 1. Back off motor
            if not sweeper.move_relative(args.axis, backoff_steps):
                print("   [WARN] Backoff movement failed. Continuing with caution...")

            time.sleep(0.3)

            # 2. Test homing
            outcome, duration, sg_val, tstep = sweeper.test_homing(args.axis, args.dir, sens)

            user_eval = "N/A"
            if args.interactive:
                print(f"   >>> Result: {outcome} in {duration}s (SG_RESULT={sg_val}, TSTEP={tstep})")
                ans = input("   Did it reach the physical end stop? [y=yes, p=premature trip, g=grinding/timeout, q=quit]: ").strip().lower()
                if ans == "y":
                    user_eval = "PERFECT_HOMED"
                    results.append((sens, outcome, duration, sg_val, user_eval))
                    print(f"   [SUCCESS] Working sensitivity found at SGTHRS={sens}!")
                    break
                elif ans == "p":
                    user_eval = "PREMATURE_TRIP"
                elif ans == "g":
                    user_eval = "NO_STALL_GRINDING"
                elif ans == "q":
                    break
            else:
                user_eval = outcome

            results.append((sens, outcome, duration, sg_val, user_eval))
            time.sleep(0.5)

        print("\n================== SWEEP RESULTS ==================")
        print(f"{'SGTHRS':<8} | {'Outcome':<16} | {'Time (s)':<10} | {'SG_RESULT':<10} | {'Evaluation'}")
        print("-" * 65)
        for s, o, d, sg, ev in results:
            sg_str = str(sg) if sg is not None else "-"
            print(f"{s:<8} | {o:<16} | {d:<10.2f} | {sg_str:<10} | {ev}")
        print("===================================================\n")

    except KeyboardInterrupt:
        print("\nSweep interrupted by user.")
    finally:
        sweeper.close()


if __name__ == "__main__":
    main()
