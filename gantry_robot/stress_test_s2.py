import serial, time

s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
time.sleep(2.0)
s.reset_input_buffer()

def execute(cmd_str, timeout=12.0):
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

print("=== 10-CYCLE STRESS TEST (700 mA Profile) ===")
print("Initializing TMC drivers...")
execute("setup", 3.0)

# Apply 700 mA profile dynamically via serial commands:
execute("s2 c700", 2.0)   # Current: 700 mA
execute("s2 hv700", 2.0)  # Homing speed: 700 steps/s
execute("s2 hs3", 2.0)    # Homing sensitivity: 3 (SGTHRS=3)

print("Active Configuration: S2 | Current: 700 mA | Home Speed: 700 steps/s | SGTHRS: 3")

results = []
jog_mm = 70.0
jog_steps = int(jog_mm * 10)  # 700 steps @ 10 steps/mm

for cycle in range(1, 11):
    # 1. Jog forward 70 mm
    lines_jog, t_jog = execute(f"s2 {jog_steps}", timeout=6.0)
    time.sleep(0.3)

    # 2. Home backward to 0
    lines_home, t_home = execute("home s2 -", timeout=8.0)

    sg_val = None
    tstep = None
    status = "FAILED"

    for l in lines_home:
        if "Position set to 0." in l:
            status = "PASS"
            try:
                if "SG_RESULT=" in l:
                    sg_val = int(l.split("SG_RESULT=")[1].split(",")[0].strip())
                if "TSTEP=" in l:
                    tstep = int(l.split("TSTEP=")[1].split(")")[0].split(".")[0].strip())
            except Exception:
                pass

    if status == "PASS" and t_home < 0.90:
        status = "PREMATURE_TRIP"

    results.append((cycle, jog_mm, t_home, sg_val, tstep, status))
    print(f"Cycle {cycle:2d}/10: Jog 70mm ({t_jog:.2f}s) -> Home in {t_home:4.2f}s | SG_RESULT={sg_val} | Status: {status}")
    time.sleep(0.4)

execute("sleep", 0.5)
s.close()

print("\n=== FINAL 10-CYCLE STRESS TEST SUMMARY (700 mA) ===")
print(f"{'Cycle':<6} | {'Jog (mm)':<10} | {'Home Time (s)':<14} | {'SG_RESULT':<10} | {'Status'}")
print("-" * 55)
for c, j, el, sg, ts, st in results:
    print(f"{c:<6} | {j:<10.1f} | {el:<14.2f} | {str(sg):<10} | {st}")

passed = sum(1 for r in results if r[5] == "PASS")
print(f"\nFinal Result: {passed}/10 Passed ({(passed/10)*100:.0f}% Success Rate)")
