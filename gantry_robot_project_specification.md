# Project Specification: Autonomous 3-Axis Cartesian Block-Sorting Gantry Robot

## 1. System Overview
An autonomous Cartesian gantry robot designed to locate, weigh, pick, orient, and stack randomly scattered colored blocks on a fixed course platform. 

* **Mechanical Drive:** 3x NEMA 11 open-loop stepper motors ($1.8^\circ$, $200\text{ steps/rev}$) driven by **TMC2209** stepper drivers ($2\times$ microstepping) across $X$, $Y$, and $Z$ lead-screw axes ($2.0\text{ mm/rev}$ pitch $\rightarrow 200.0\text{ microsteps/mm}$), plus a 9g PWM wrist servo ($\theta$) and a Normally-Closed (NC) solenoid gripper.
* **Course & Fiducial Geometry:**
  * **Inner Playable Course:** $254.0\text{ mm}$ long ($X$, $10\text{ in}$) $\times 152.4\text{ mm}$ wide ($Y$, $6\text{ in}$), with its back edge coincident with the fiducial plate back edge at $Y = 0.0\text{ mm}$ (spanning $X \in [15.0, 269.0]\text{ mm}$ and $Y \in [0.0, 152.4]\text{ mm}$).
  * **Outer Fiducial Border:** $284.0\text{ mm} \times 167.4\text{ mm}$ total footprint (Origin $(0, 0, 0)$ at bottom-left corner) with four $15 \times 15\text{ mm}$ square ArUco fiducial markers (`DICT_4X4_50`) at the corners.
  * **Motor Travel Strokes (from Homed $0, 0, 0$):** $X = 236.0\text{ mm}$, $Y = 128.0\text{ mm}$, $Z = 84.0\text{ mm}$ ($Z=0$ at top when homed, $76.71\text{ mm}$ downward travel to course floor).
  * **Homed Positions (in Fiducial Plate Coordinates $0, 0, 0$):**
    * **Homed Datum:** $(20.0,\; -43.5,\; 127.5)\text{ mm}$.
    * **Homed Square End-Effector Top:** $(28.0,\; +6.0,\; 97.6)\text{ mm}$ (reachable envelope $X \in [28.0,\; 264.0]\text{ mm}$, $Y \in [6.0,\; 134.0]\text{ mm}$).
    * **Homed Camera Optical Center:** $(27.8,\; 60.1,\; 135.3)\text{ mm}$ (reachable envelope $X \in [27.8,\; 263.8]\text{ mm}$, $Y \in [60.1,\; 188.1]\text{ mm}$).
  * **Drop-Off Target (wrt Fiducial Plate Origin):** $(X = 224.5\text{ mm},\; Y = 44.5\text{ mm},\; Z = +12.6\text{ mm up})$ (corresponding to motor $Z_{\text{down}} = 76.71 - 12.6 = 64.11\text{ mm}$).
  * **Block Classes ($20 \times 20\text{ mm}$):**
    * **Aluminum:** $\sim 21.6\text{ g}$
    * **High Infill:** $\sim 15.0\text{ g}$
    * **Medium Infill:** $\sim 10.0\text{ g}$
    * **Low Infill:** $\sim 5.0\text{ g}$
* **Perception (Eye-in-Hand):** A UVC USB-A camera mounted on the moving end-effector, vertically aligned with the gripper in $X$ ($\Delta X_{\text{offset}} = 0.0\text{ mm}$) and offset $35.0\text{ mm}$ forwards in $Y$ ($\Delta Y_{\text{offset}} = +35.0\text{ mm}$).
* **Mass Sensing:** An HX711 load cell integrated into the end-effector to weigh blocks during pickup for sorting and quality verification.

---

## 2. Three-Tier Distributed Architecture

The system separates high-level visualization, computer vision/orchestration, and deterministic real-time step generation across three hardware tiers:

```text
[ Tier 1: Host PC / Laptop ]  <-- (WebSockets: Wi-Fi wlan0 or USB OTG usb0) -->  [ Tier 2: Raspberry Pi Zero W + MakerSpot 4-Port USB Hub HAT ]  <-- (USB-A to USB-C Serial) -->  [ Tier 3: ESP32 + 3x TMC2209 ]
 * 2D/3D Virtual Environment GUI                                                  * Autonomous System Brain                                                                           * 3x NEMA 11 Stepper Control
 * Live Telemetry & Mass Plots                                                    * UVC USB-A Camera & OpenCV Pipeline                                                                * Limit Switch / StallGuard Homing
 * High-Level User Triggers                                                       * Coordinate Math & Path Planner                                                                    * HX711 Load Cell & Gripper I/O
```

### Tier 1: Host PC / Laptop (Supervisor & GUI Engine)
* **Role:** Optional external visualizer and control dashboard.
* **Connectivity Options:**
  * **Wireless (`wlan0` on Pi Zero W — Primary Run Mode):** Hardware inspection confirmed the board is a **Raspberry Pi Zero W Rev 1.1** with onboard 2.4 GHz Wi-Fi (`wlan0`). Connecting `wlan0` to the local Wi-Fi network allows full SSH, code sync (`scp`/`rsync`), and live WebSocket telemetry to the Host PC while the Pi's single USB controller stays permanently in **USB Host mode (`dr_mode=host`)** driving the MakerSpot 4-port USB Hub HAT (UVC camera + ESP32).
  * **Wired USB OTG (`usb0` Gadget Mode — Provisioning / Direct Tether):** Connects via a single Micro-USB cable using `rpi-usb-gadget` / `g_ncm` (`dr_mode=peripheral`, hostname `huedini`, static IPs `169.254.100.2/16` and `192.168.137.2/24`, configured with SSH alias `ssh huedini`). Note: Because the Pi Zero W has a single USB data bus shared between the Micro-USB port and the MakerSpot HAT's 4 pogo pins, USB Gadget mode (`dr_mode=peripheral`) and MakerSpot 4-port USB Host mode (`dr_mode=host`) are mutually exclusive.
  * **Disconnected (Headless):** The robot runs autonomously without the PC attached; the PC can connect or disconnect mid-operation without interrupting the state machine.
* **Responsibilities:**
  * Renders the 2D/3D virtual environment map, block positions, orientations, and sorting progress using lightweight JSON telemetry packets from the Pi.
  * Displays real-time stats (cycle times, mass distribution, state logs).
  * Sends high-level user commands (`START_SCAN`, `PAUSE`, `RESUME`, `EMERGENCY_STOP`).

### Tier 2: Raspberry Pi Zero W + MakerSpot 4-Port USB Hub HAT (Orchestrator & Vision Engine)
* **Role:** Central system brain, classical computer vision processor, and telemetry server.
* **Hardware:** Raspberry Pi Zero W Rev 1.1 (`armv6l`, 512 MB RAM, onboard `wlan0` Wi-Fi) running Raspberry Pi OS Lite (Kernel 6.18, headless).
* **USB Expansion:** MakerSpot 4-Port Stackable USB Hub HAT connected via 4 pogo pins (`+5V`, `GND`, `D+`, `D-`) underneath the Pi Zero W, providing 4 USB-A ports when `dwc2` is in Host mode (`dtoverlay=dwc2,dr_mode=host`).
* **Camera Interface:** UVC USB-A Camera plugged into a USB-A port on the MakerSpot HAT (`/dev/video0` via V4L2) capturing on-demand frames at $640 \times 480$ or $320 \times 240$ resolution.
* **ESP32 Interface:** USB-A (on the MakerSpot HAT) to USB-C (on the ESP32) serial connection (`/dev/ttyUSB0` or `/dev/ttyACM0` at 115200 baud) — clean USB cabling with no GPIO UART jumpers required.
* **Responsibilities:**
  * **Classical Computer Vision (No Neural Networks):** Uses OpenCV ArUco detection for fiducials, HSV color space thresholding, `cv2.findContours()`, and `cv2.minAreaRect()` to determine block centroids $(u, v)$ and rotation angles ($\theta$).
  * **Eye-in-Hand Coordinate Math:** Translates camera pixel offsets into physical gantry millimeter coordinates $(X_{\text{mm}}, Y_{\text{mm}})$ using calibrated scale factors $(K_x, K_y)$ and the fixed physical vector offset between the camera lens and gripper center $(\Delta X_{\text{offset}}, \Delta Y_{\text{offset}})$.
  * **Virtual World Modeling & Path Planning:** Catalogs all detected blocks during the initial scan, deduplicates overlapping views, and computes an optimized pick sequence that avoids colliding with adjacent blocks.
  * **WebSocket Telemetry Server:** Broadcasts lightweight JSON state updates to any connected Tier 1 client.
  * **Serial Master:** Sends line-terminated ASCII G-code commands over USB Serial (`/dev/ttyUSB0` or `/dev/ttyACM0`) to the ESP32 and waits for deterministic completion replies.

### Tier 3: ESP32 + 3x TMC2209 (Real-Time Motion & I/O Controller)
* **Role:** Deterministic hardware execution layer (see schematic `HUE-DINI-NOV3.pdf` and baseline test sketch `HUE-DINI-NOV3.txt`).
* **Environment:** C++ (Arduino / PlatformIO) using `AccelStepper`, `TMCStepper`, `ESP32Servo`, and `HX711`.
* **Hardware Pinout & Configuration:**
  * **Stepper Drivers (3x TMC2209 on shared `HardwareSerial(1)` UART bus, `R_SENSE = 0.11f`):**
    * **Shared UART:** `PIN_TMC_TX = 17`, `PIN_TMC_RX = 16` (115200 baud, 1K resistor between TX/RX).
    * **Shared Driver Enable:** `PIN_EN = 23` (`LOW` = enabled, `HIGH` = disabled/sleep).
    * **Axis 1 (`S1` / X-Axis):** `PIN_STEP_1 = 25`, `PIN_DIR_1 = 26`, `PIN_DIAG_1 = 32`, UART address `0b00` (`MS1=LOW, MS2=LOW`).
    * **Axis 2 (`S2` / Y-Axis):** `PIN_STEP_2 = 27`, `PIN_DIR_2 = 14`, `PIN_DIAG_2 = 33`, UART address `0b01` (`MS1=HIGH, MS2=LOW`).
    * **Axis 3 (`S3` / Z-Axis):** `PIN_STEP_3 = 18`, `PIN_DIR_3 = 19`, `PIN_DIAG_3 = 22`, UART address `0b10` (`MS1=LOW, MS2=HIGH`).
  * **Homing & Stall Protection:** Sensorless homing via TMC2209 **StallGuard4** (`DIAG` pins `32`, `33`, `22`) with configurable homing speed (`homeSpeed`), homing StallGuard threshold (`homeSensitivity`), blanking time (`300 ms`), and cruise-speed stall detection during normal moves.
  * **End-Effector Rotation ($\theta$ Axis):** 9g Hobby PWM Servo on `PIN_SERVO = 13` (`500..2400 us` pulse width, `0..180 deg`).
  * **End-Effector Gripper:** 12V Solenoid (`SOLENOID1`) wired in the **Normally Closed (NC)** configuration, switched via a relay (`RELAY1`) driven by `PIN_RELAY = 4`.
  * **HX711 Load Cell Amplifier:** `PIN_HX711_DT = 35` (input-only), `PIN_HX711_SCK = 12`.
* **Responsibilities:**
  * Initializes TMC2209 drivers over UART (`rms_current`, `microsteps`, StealthChop + `SGTHRS` StallGuard threshold).
  * Generates smooth step pulses across $X, Y, Z$ with `AccelStepper` and monitors `DIAG` pins for unexpected stalls.
  * Executes sensorless StallGuard4 homing sequences (`HOME`) to establish machine origin $(0, 0, 0)$.
  * Controls the 9g wrist rotation servo (`A<deg>`) and solenoid gripper relay (`GRIP <0|1>`), and samples the HX711 load cell (`TARE`, `WEIGH`).
  * Parses both the automated Pi Zero ASCII protocol (`SETUP`, `HOME`, `MOVE`, `GRIP`, `TARE`, `WEIGH`, `SLEEP`) and interactive human debug commands (`setup`, `s1 500`, `home s1`, `servo 90`, `relay 2`, `l`, `help`).

---

## 3. Core Operational Workflow (State Machine)

### Phase 1: Boot & Idle (`IDLE`)
1. Pi Zero boots headless and starts the orchestrator service automatically via `systemd`.
2. Pi initializes the UVC USB-A camera (`/dev/video0`), opens the USB serial link (`/dev/ttyUSB0` or `/dev/ttyACM0`) to the ESP32, and starts the WebSocket telemetry server.
3. System waits for a `START_SCAN` trigger from either the PC GUI or a physical start button wired to the ESP32.

### Phase 2: Homing & Eye-in-Hand Self-Calibration (`CALIBRATING`)
1. Pi commands ESP32 to home all axes against mechanical limit switches (`HOME`) to establish absolute $(0, 0, 0)$ in motor space.
2. Gantry moves to a fixed scanning height ($Z_{\text{scan}}$) directly above a known base fiducial marker on the platform.
3. **Auto-Scale Routine:**
   * Pi captures Frame A and records fiducial center $(u_1, v_1)$ and orientation relative to the gantry axes.
   * Pi commands ESP32 to step a known physical distance (e.g., $+20\text{ mm}$ in $X$ and $+20\text{ mm}$ in $Y$).
   * Pi captures Frame B and records the new fiducial center $(u_2, v_2)$.
   * Pi computes the exact pixel-to-millimeter scale factors at height $Z_{\text{scan}}$:
     $$K_x = \frac{\Delta X_{\text{mm}}}{u_2 - u_1}, \quad K_y = \frac{\Delta Y_{\text{mm}}}{v_2 - v_1}$$

### Phase 3: Global Course Scan (`SCANNING`)
1. Gantry steps through a predefined multi-point grid across the course bed at fixed height $Z_{\text{scan}}$.
2. **Stop-and-Capture Rule:** At each grid point, the gantry comes to a complete mechanical stop before the Pi captures a single static frame (eliminating motion blur, rolling shutter distortion, and continuous video CPU load).
3. OpenCV extracts block colors, centroids, and bounding-box angles ($\theta$), converting them to global millimeter coordinates and merging duplicates across overlapping frames to build the virtual environment model.

### Phase 4: Path & Sequence Optimization (`PLANNING`)
1. Pi evaluates the virtual block catalog and computes an optimal pick order based on color grouping and spatial clearance around each block so the end-effector does not clip neighboring cubes.
2. Optional 2D/3D bin-packing or stacking logic computes target drop-off coordinates $(X_{\text{drop}}, Y_{\text{drop}}, Z_{\text{stack}})$.

### Phase 5: Pick, Align, and Weigh (`PICKING` & `WEIGHING`)
1. Gantry moves open-loop to position the camera above the target block.
2. **Stop-and-Check Fine Alignment:** Pi captures a single verification frame, calculates any residual positional error, and issues a small relative correction move so the gripper center aligns with the block centroid.
3. End-effector rotates to match block angle $\theta$, tares the HX711 load cell (`TARE`), descends in $Z$, closes the gripper (`GRIP 1`), and lifts to hover height.
4. ESP32 samples the HX711 load cell (`WEIGH`) and returns the measured mass in grams to the Pi Zero.

### Phase 6: Sort & Stack (`STACKING`)
1. Pi determines the final stacking destination based on measured mass and block color (diverting out-of-spec weights to a reject zone if needed).
2. Gantry moves to the target stack coordinate, places the block, updates the virtual model, broadcasts the updated state via WebSockets, and loops to the next block in the queue.

---

## 4. Communication Protocols

### A. Pi Zero $\leftrightarrow$ ESP32 (USB-A to USB-C Serial @ 115200 Baud)
Line-terminated ASCII protocol (`\n`) over `/dev/ttyUSB0` or `/dev/ttyACM0`:

| Direction | Command / Format | Description |
| :--- | :--- | :--- |
| **Pi $\rightarrow$ ESP32** | `HOME\n` | Home $X, Y, Z, \theta$ axes using limit switches. |
| **Pi $\rightarrow$ ESP32** | `MOVE X<f> Y<f> Z<f> A<f> F<f>\n` | Coordinated linear move to target coordinates (mm/deg) at feedrate $F$. |
| **Pi $\rightarrow$ ESP32** | `GRIP <0\|1>\n` | Open (`0`) or close (`1`) end-effector gripper. |
| **Pi $\rightarrow$ ESP32** | `TARE\n` | Zero the HX711 load cell offset. |
| **Pi $\rightarrow$ ESP32** | `WEIGH\n` | Read averaged mass from HX711. |
| **ESP32 $\rightarrow$ Pi** | `OK\n` | Deterministic acknowledgment sent only after motion/action completes. |
| **ESP32 $\rightarrow$ Pi** | `WEIGHT:<float>\n` | Measured block mass in grams (e.g., `WEIGHT:51.4`). |
| **ESP32 $\rightarrow$ Pi** | `EVENT:START_BTN\n` | Hardware start button pressed on the machine. |
| **ESP32 $\rightarrow$ Pi** | `ERR:<string>\n` | Limit fault, stall, or timeout error. |

### B. Pi Zero $\leftrightarrow$ Host PC (WebSockets JSON Schema)
State broadcast from Pi Zero to Host PC:
```json
{
  "timestamp": 1730590000.12,
  "state": "SCANNING",
  "gantry_pose": {
    "x_mm": 120.5,
    "y_mm": 45.0,
    "z_mm": 100.0,
    "angle_deg": 45.0
  },
  "calibration": {
    "calibrated": true,
    "kx_mm_per_px": 0.245,
    "ky_mm_per_px": 0.245
  },
  "blocks": [
    {
      "id": 1,
      "color": "red",
      "x_mm": 140.2,
      "y_mm": 80.1,
      "angle_deg": 32.5,
      "mass_g": 52.1,
      "status": "STACKED"
    }
  ]
}
```

---

## 5. Development Structure & Agent Implementation Rules

### Repository Layout
```text
gantry_robot/
├── esp32_firmware/          # PlatformIO / Arduino C++ firmware (Serial parser, AccelStepper, HX711)
├── pi_controller/           # Python 3 package running on Raspberry Pi Zero
│   ├── main.py              # Entry point & CLI flags (--mock-hardware, --port)
│   ├── state_machine.py     # Core operational workflow & error recovery
│   ├── vision.py            # OpenCV ArUco calibration, HSV thresholding & contour orientation
│   ├── kinematics.py        # Pixel-to-mm transformations, camera-to-gripper offsets & path planner
│   ├── serial_bridge.py     # Blocking/async UART communication with ESP32 (plus mock driver)
│   └── telemetry_server.py  # FastAPI / WebSockets server broadcasting JSON state
└── pc_dashboard/            # Web frontend (HTML5 Canvas / Three.js) or Python GUI client
```

### Critical Constraints for Code Generation
1. **Single-Core Pi Zero Optimization:** Never run continuous `while True` frame-processing loops while the gantry is moving. Capture single on-demand frames only after receiving `OK` from the ESP32 confirming the gantry is stationary.
2. **Graceful Headless Operation:** The `telemetry_server.py` module must handle zero connected WebSocket clients without blocking or throwing exceptions in the main state machine.
3. **Simulation / Mock Mode:** Implement a `--mock-hardware` mode in `serial_bridge.py` and `vision.py` (using synthetic test frames and simulated serial delays) so the entire state machine and PC dashboard can be developed and tested on a laptop prior to hardware deployment.

---

## 6. Hardware Troubleshooting & Issue Resolution Log

### Issue #1: Axis 2 (Y-Axis) TMC2209 Sensorless StallGuard Homing Inconsistency & Premature Trips
* **Date:** November 2026
* **Component:** Axis 2 (`S2`, Y-Axis, NEMA 11 + TMC2209 over UART, GT2 20T belt drive carrying X+Z gantry bridge).
* **Symptoms:**
  * Axis 2 sensorless homing was erratic: the carriage tripped prematurely mid-travel before reaching the mechanical bumper, or failed to trip when hitting the stop.
  * Manual CLI commands (`s2 500`, `home s2`, `s2 hs...`) caused 15-second timeouts in `serial_bridge.py` due to missing `OK` replies from the ESP32.
  * Physical homing direction was inverted; $+Y$ commands drove the carriage backward into the end stop rather than forward into the course.
* **Root Cause Analysis:**
  1. **Direction Inversion:** Mechanical end stop is at the back datum ($Y = 0$). In the original firmware, `INVERT_DIR_Y = false`, causing positive steps to move toward the bumper and negative steps into the course. Setting `INVERT_DIR_Y = true` and `DEFAULT_HOME_DIR_REVERSE_Y = true` aligned motor space with physical space.
  2. **Insufficient Back-EMF at Low Speed:** The baseline homing speed of 500 steps/s (1.25 rev/s) generated weak back-EMF, causing free-flight `SG_RESULT` to float between 0 and 8 (dangerously close to the stall threshold). Increasing homing velocity to 700 steps/s (70.0 mm/s, 1.75 rev/s) elevated free-motion `SG_RESULT` into the 14–20 range, establishing a clear dynamic range.
  3. **Belt Settling Oscillation & Acceleration Dip:** Carrying the heavy X+Z gantry bridge over a GT2 belt causes mechanical vibration during the 350 ms acceleration ramp, creating a transient dip in `SG_RESULT` to ~6–8 for up to 800 ms. Setting `HOMING_BLANKING_MS = 800` completely ignored this startup transient.
  4. **StallGuard4 Stall Condition Physics:** The TMC2209 asserts stall when $\text{SG\_RESULT} \le 2 \times \text{SGTHRS}$. At sensitivity `hs3` (`SGTHRS = 3`), stall asserts when $\text{SG\_RESULT} \le 6$.
  5. **Current Optimization:** Tuning current from 500 mA to 700 mA (`SETUP_CURRENT_MA_Y = 700`) provided optimal belt stiffness and motor torque without causing excessive impact against the mechanical bumper.
  6. **Interactive CLI Synchronization:** Added `Serial.println("OK");` to all manual command handlers (`home`, `s1/2/3`, `servo`, `relay`, `l`, `help`) in `esp32_firmware.ino` so that serial bridges synchronize without timeouts.
* **Verification & Stress Test Results:**
  * Completed automated 10-cycle stress test (70.0 mm jog forward, home backward to 0):
    * Cycles 3 through 10 completed with 100% repeatability at **1.21 – 1.22 seconds** per return.
    * Recorded $\text{SG\_RESULT} = 6$ on bumper impact with zero lag and zero premature trips.
* **Locked Production Configuration (Axis 2 / Y-Axis):**
  * `SETUP_CURRENT_MA_Y = 700` (700 mA RMS)
  * `SETUP_MICROSTEPS = 2` (10.0 steps/mm on 20T GT2 belt pulley)
  * `DEFAULT_HOME_SPEED_Y = 700.0f` (70.0 mm/s)
  * `DEFAULT_HOME_SENS_Y = 3` (`SGTHRS = 3`)
  * `DEFAULT_HOME_DIR_REVERSE_Y = true`
  * `INVERT_DIR_Y = true`
  * `HOMING_RAMP_MS = 350`
  * `HOMING_BLANKING_MS = 800`

### Issue #2: Axis 1 (X-Axis) and Axis 3 (Z-Axis) Lead Screw Sensorless StallGuard Lag & Failure to Trip
* **Date:** November 2026
* **Component:** Axis 1 (`S1`, X-Axis) and Axis 3 (`S3`, Z-Axis), NEMA 11 + TMC2209 over UART, 2.0 mm/rev lead screw drives ($200.0\text{ microsteps/mm}$ at $2\times$ microstepping).
* **Symptoms:**
  * When homing X or Z, the carriages reached the mechanical end stops but failed to trip StallGuard, continuing to power and grind against the stops for the full 20–30s timeout window.
  * Homing travel speed was excessively slow ($2.5\text{ mm/s}$ at $500\text{ steps/s}$), causing travel across normal offsets to approach or exceed the timeout.
* **Root Cause Analysis:**
  1. **Lead Screw Kinematics vs. Belt:** A 2.0 mm pitch lead screw delivers a $20\times$ higher mechanical advantage than the 40 mm/rev GT2 belt. Motor rotational velocity per linear mm is 20 times higher ($200\text{ steps/mm}$ vs $10\text{ steps/mm}$).
  2. **Stalled SG_RESULT Floor:** Due to high mechanical reduction and residual elastic deflection, the motor's back-EMF does not collapse to zero upon striking the stop. Telemetry showed stalled values of $\text{SG\_RESULT} = 56$ on X and $\text{SG\_RESULT} = 104$ on Z (lifting the end-effector against gravity).
  3. **Baseline Sensitivity Deficit:** The original default `DEFAULT_HOME_SENS = 20` set the stall threshold to $\text{SG\_RESULT} \le 40$ ($2 \times 20$). Because X stalled at 56 and Z stalled at 104, neither axis could ever drop below 40, rendering stall detection physically impossible!
  4. **Low Speed Back-EMF:** Operating at 500 steps/s ($1.25\text{ rev/s} = 75\text{ RPM}$) at 300 mA produced marginal back-EMF. Increasing homing speed to 800 steps/s ($4.0\text{ mm/s} = 2.0\text{ rev/s} = 120\text{ RPM}$) and current to 350 mA elevated back-EMF and provided a wide dynamic range.
* **Verification & Testing Results:**
  * Setting `DEFAULT_HOME_SENS = 60` establishes a stall threshold of $\text{SG\_RESULT} \le 120$ ($2 \times 60$).
  * **Axis 1 (X) Test:** Jogged $+10.0\text{ mm}$ ($+2000\text{ steps}$) in $2.47\text{ s}$, homed backward in $2.72\text{ s}$ (ideal theoretical $2.66\text{ s}$). Zero premature trip; instant trip upon impact ($\text{SG\_RESULT} = 56$, lag $< 60\text{ ms}$).
  * **Axis 3 (Z) Test:** Jogged $+10.0\text{ mm}$ down in $2.47\text{ s}$, homed backward up in $2.92\text{ s}$. Zero premature trip; instant trip upon top bumper impact ($\text{SG\_RESULT} = 104$, lag $< 100\text{ ms}$).
* **Locked Production Configuration (Axes 1 & 3 / X & Z Lead Screws):**
  * `SETUP_CURRENT_MA_XZ = 350` (350 mA RMS)
  * `SETUP_MICROSTEPS = 2` (200.0 steps/mm on 2.0 mm pitch lead screws)
  * `DEFAULT_HOME_SPEED_XZ = 800.0f` (4.0 mm/s)
  * `DEFAULT_HOME_SENS = 60` (`SGTHRS = 60`, stall asserted when $\text{SG\_RESULT} \le 120$)
  * `DEFAULT_HOME_DIR_REVERSE_X = true`, `DEFAULT_HOME_DIR_REVERSE_Z = true`
  * `HOMING_RAMP_MS = 350`, `HOMING_BLANKING_MS = 800`

