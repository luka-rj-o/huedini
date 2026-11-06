#!/usr/bin/env python3
import serial
import time
import sys
import argparse

def main():
    parser = argparse.ArgumentParser(description="Test homing for X or Z axis")
    parser.add_argument("--axis", choices=["s1", "s3"], default="s1", help="Axis to test (s1=X, s3=Z)")
    parser.add_argument("--current", type=int, default=None, help="Current in mA (e.g. 300, 400, 500)")
    parser.add_argument("--speed", type=float, default=None, help="Homing speed in steps/s (e.g. 500, 600, 700)")
    parser.add_argument("--sens", type=int, default=None, help="Homing SGTHRS sensitivity (e.g. 20, 15, 10, 5)")
    parser.add_argument("--jog_steps", type=int, default=0, help="Steps to jog in + direction before homing")
    args = parser.parse_args()

    s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
    time.sleep(1.5)
    s.reset_input_buffer()

    def execute(cmd_str, timeout=15.0):
        s.reset_input_buffer()
        s.write((cmd_str + '\n').encode())
        t0 = time.time()
        lines = []
        while time.time() - t0 < timeout:
            raw = s.readline()
            if not raw:
                continue
            line = raw.decode('utf-8', errors='replace').strip()
            if line:
                lines.append(line)
                if line == 'OK':
                    break
        elapsed = time.time() - t0
        return lines, elapsed

    print(f"=== Testing Axis {args.axis.upper()} ===")
    lines, _ = execute("setup", 3.0)
    print("Setup done.")

    if args.current is not None:
        lines, _ = execute(f"{args.axis} c{args.current}", 2.0)
        print(f"Set current: {lines}")
    if args.speed is not None:
        lines, _ = execute(f"{args.axis} hv{args.speed}", 2.0)
        print(f"Set home speed: {lines}")
    if args.sens is not None:
        lines, _ = execute(f"{args.axis} hs{args.sens}", 2.0)
        print(f"Set home sensitivity: {lines}")

    if args.jog_steps > 0:
        print(f"Jogging {args.axis.upper()} +{args.jog_steps} steps...")
        lines, t_jog = execute(f"{args.axis} {args.jog_steps}", timeout=10.0)
        print(f"Jog finished in {t_jog:.2f}s: {lines}")
        time.sleep(0.5)

    print(f"Homing {args.axis.upper()} backward ('home {args.axis} -')...")
    lines_home, t_home = execute(f"home {args.axis} -", timeout=20.0)

    execute("sleep", 1.0)
    s.close()

    print(f"\n--- Homing Results for {args.axis.upper()} ---")
    print(f"Elapsed Time: {t_home:.2f} s")
    for l in lines_home:
        print(f"  {l}")

if __name__ == "__main__":
    main()
