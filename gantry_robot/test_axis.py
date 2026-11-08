#!/usr/bin/env python3
import serial
import time
import sys
import argparse

def main():
    parser = argparse.ArgumentParser(description="Test homing for X, Y, or Z axis")
    parser.add_argument("--axis", choices=["s1", "s2", "s3"], default="s1", help="Axis to test (s1=X, s2=Y, s3=Z)")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--current", type=int, default=None, help="Current in mA (e.g. 300, 350, 700)")
    parser.add_argument("--speed", type=float, default=None, help="Homing speed in steps/s (e.g. 500, 700, 800)")
    parser.add_argument("--sens", type=int, default=None, help="Homing SGTHRS sensitivity (e.g. 3, 20, 60)")
    parser.add_argument("--jog_steps", type=int, default=0, help="Steps to jog in + direction before homing")
    parser.add_argument("--home_timeout", type=float, default=15.0, help="Max seconds to wait for homing before aborting")
    parser.add_argument("--steps_per_mm", type=float, default=None, help="Steps per mm (default: 50.0 for X/Z with T8x8, 10.0 for Y belt)")
    args = parser.parse_args()

    # Auto-fallback to candidate ports on Windows/Linux if preferred port not openable
    port = args.port
    try:
        s = serial.Serial(port, 115200, timeout=0.1)
    except Exception:
        import serial.tools.list_ports
        available = [p.device for p in serial.tools.list_ports.comports()]
        if available:
            port = available[0]
            print(f"Preferred port {args.port} not available; using {port}")
            s = serial.Serial(port, 115200, timeout=0.1)
        else:
            raise

    time.sleep(1.5)
    s.reset_input_buffer()

    def execute(cmd_str, timeout=10.0):
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
                if line == 'OK' or "Position set to 0." in line or line.startswith("ERR:"):
                    break
        elapsed = time.time() - t0
        return lines, elapsed

    print(f"=== Testing Axis {args.axis.upper()} on {port} ===", flush=True)
    lines, _ = execute("setup", 3.0)
    print("Setup done.", flush=True)

    if args.current is not None:
        lines, _ = execute(f"{args.axis} c{args.current}", 2.0)
        print(f"Set current ({args.current} mA): {lines}", flush=True)
    if args.speed is not None:
        lines, _ = execute(f"{args.axis} hv{args.speed}", 2.0)
        print(f"Set home speed ({args.speed} steps/s): {lines}", flush=True)
    if args.sens is not None:
        lines, _ = execute(f"{args.axis} hs{args.sens}", 2.0)
        print(f"Set home sensitivity (SGTHRS={args.sens}): {lines}", flush=True)

    default_spm = 10.0 if args.axis == "s2" else 50.0
    steps_per_mm = args.steps_per_mm if args.steps_per_mm is not None else default_spm
    if args.jog_steps > 0:
        jog_mm = args.jog_steps / steps_per_mm
        print(f"Jogging {args.axis.upper()} +{args.jog_steps} steps (+{jog_mm:.1f} mm)...", flush=True)
        lines, t_jog = execute(f"{args.axis} {args.jog_steps}", timeout=10.0)
        print(f"Jog complete in {t_jog:.2f}s.", flush=True)
        time.sleep(0.4)

    print(f"Homing {args.axis.upper()} backward ('home {args.axis} -')...", flush=True)
    lines_home, t_home = execute(f"home {args.axis} -", timeout=args.home_timeout)

    # Immediately sleep steppers so carriage is safe and unpowered
    execute("sleep", 1.0)
    s.close()

    print(f"\n--- Homing Results for {args.axis.upper()} ---", flush=True)
    print(f"Elapsed Time: {t_home:.2f} s", flush=True)
    homed = False
    sg_result = None
    for l in lines_home:
        print(f"  {l}", flush=True)
        if "Position set to 0." in l:
            homed = True
            if "SG_RESULT=" in l:
                try:
                    sg_result = int(l.split("SG_RESULT=")[1].split(",")[0].strip())
                except Exception:
                    pass

    if homed:
        print(f"Status: TRIP DETECTED (SG_RESULT={sg_result}) in {t_home:.2f}s", flush=True)
    else:
        print(f"Status: NO TRIP (Timed out after {t_home:.2f}s)", flush=True)

if __name__ == "__main__":
    main()
