/*
  HUE-DINI Gantry Robot — Tier 3 ESP32 Real-Time Motion & I/O Firmware
  --------------------------------------------------------------------
  Hardware (per HUE-DINI-NOV3 schematic & test sketch):
    - 3x NEMA 11 steppers driven by TMC2209 drivers:
        * Shared UART bus on HardwareSerial(1) (RX=16, TX=17, 1K resistor)
        * StallGuard4 sensorless homing & stall detection on DIAG pins (32, 33, 22)
        * Shared hardware enable on PIN_EN = 23 (LOW = enabled)
        * S1 (X-Axis): STEP=25, DIR=26, DIAG=32, UART addr 0b00
        * S2 (Y-Axis): STEP=27, DIR=14, DIAG=33, UART addr 0b01
        * S3 (Z-Axis): STEP=18, DIR=19, DIAG=22, UART addr 0b10
    - End-Effector Rotation (Theta / A-Axis):
        * 9g Hobby PWM Servo on PIN_SERVO = 13 (500..2400 us, 0..180 deg)
    - End-Effector Gripper:
        * 12V Solenoid wired Normally Closed (NC) switched via Relay on PIN_RELAY = 4
    - Mass Sensing:
        * HX711 Load Cell Amplifier on PIN_HX711_DT = 35, PIN_HX711_SCK = 12

  Supports BOTH:
    1. Tier 2 Raspberry Pi Zero W deterministic ASCII protocol:
         SETUP
         HOME
         MOVE X<mm> Y<mm> Z<mm> A<deg> F<mm_s>
         GRIP <0|1>
         TARE
         WEIGH
         STATUS
         STEPS X<spm> Y<spm> Z<spm>
         CHIME [TADA]
         SLEEP
         ESTOP
    2. Interactive manual testing commands from HUE-DINI-NOV3.txt:
         setup, sleep, s1 <steps>, s1 c/m/s/v/a/r/hv/hs, home s1 [-],
         servo <deg>, relay <sec>, l, chime, help
*/

#include <Arduino.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <TMCStepper.h>
#include <HX711.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ============================================================
//  PIN DEFINITIONS
// ============================================================
const int PIN_STEP_1 = 25, PIN_DIR_1 = 26, PIN_DIAG_1 = 32; // S1 (X-Axis)
const int PIN_STEP_2 = 27, PIN_DIR_2 = 14, PIN_DIAG_2 = 33; // S2 (Y-Axis)
const int PIN_STEP_3 = 18, PIN_DIR_3 = 19, PIN_DIAG_3 = 22; // S3 (Z-Axis)

const int PIN_EN     = 23;   // Shared enable for all drivers, LOW = enabled
const int PIN_SERVO  = 13;   // 9g PWM servo for end-effector angle (theta)
const int PIN_RELAY  = 4;    // Relay controlling Normally-Closed (NC) solenoid gripper
const int PIN_TMC_TX = 17;   // TMC2209 shared UART TX
const int PIN_TMC_RX = 16;   // TMC2209 shared UART RX

// HX711 Load Cell Pins
const int PIN_HX711_DT  = 35; // Input-only pin on ESP32
const int PIN_HX711_SCK = 12;

// Hardware switches per HUE-DINI-NOV3 schematic (LS_1..LS_3 wired to GND, no hardware RC filters):
//   LS_1 (LIMIT_1, ESP32 pin 25) -> GPIO 21 -> Start
//   LS_2 (LIMIT_2, ESP32 pin 29) -> GPIO 5  -> E-Stop
//   LS_3 (LIMIT_3, ESP32 pin 34) -> GPIO 2  -> Chime
const int PIN_SWITCH_START = 21;
const int PIN_SWITCH_ESTOP = 5;
const int PIN_SWITCH_CHIME = 2;

const unsigned long SWITCH_DEBOUNCE_MS = 50UL;
const unsigned long ESTOP_DEBOUNCE_MS  = 20UL;

// ============================================================
//  SETTINGS & KINEMATIC DEFAULTS
// ============================================================
const float R_SENSE = 0.11f;

// Values applied to steppers by the "setup" command
const int SETUP_CURRENT_MA_XZ = 350; // Calibrated 350mA for X and Z lead screws
const int SETUP_CURRENT_MA_Y  = 700; // Optimal 700mA for Y-axis carrying X+Z gantry bridge on belt
const int SETUP_MICROSTEPS    = 2;   // Y & Z keep tuned 2x; X uses 8x for smooth, quiet motion
const int SETUP_MICROSTEPS_X  = 8;
// Crash (stall) detection SGTHRS used during MOVE / manual jogs.
// Defaults reuse the homing thresholds because MOVE speeds are capped near the calibrated homing speed
// (see MAX_FEED_SPEED_*), and the check is gated to cruise speed + settle time + debounce (see isStalled()).
// Set to 0 with "s1 s0" (etc.) to disable crash detection on an axis.
const int SETUP_STALL_SENS_XZ = 60;  // == DEFAULT_HOME_SENS
const int SETUP_STALL_SENS_Y  = 2;   // Avoid false stalls on Y MOVE decel/cruise (free-air SG dips to 6)

// Per-axis defaults (speeds in microsteps/s, acceleration in microsteps/s^2)
const float DEFAULT_MAX_SPEED_X   = 5000.0f;  // 25.0 mm/s on 8mm-lead screw @ 8x (200 steps/mm)
const float DEFAULT_ACCEL_X       = 10000.0f; // 50.0 mm/s^2 on 8mm-lead screw @ 8x
const float DEFAULT_HOME_SPEED_X  = 3200.0f;  // 16.0 mm/s @ 8x (same physical speed & StallGuard4 back-EMF)

const float DEFAULT_MAX_SPEED_XZ  = 1250.0f; // 25.0 mm/s on 8mm-lead screw @ 2x (50 steps/mm)
const float DEFAULT_ACCEL_XZ      = 2500.0f; // 50.0 mm/s^2 on 8mm-lead screw @ 2x
const float DEFAULT_HOME_SPEED_XZ = 800.0f;  // 16.0 mm/s @ 2x (calibrated for healthy back-EMF)

const float DEFAULT_MAX_SPEED_Y   = 700.0f;  // 70.0 mm/s on 40mm/rev GT2 20T belt
const float DEFAULT_ACCEL_Y       = 1000.0f; // 100.0 mm/s^2 on 40mm/rev GT2 20T belt
const float DEFAULT_HOME_SPEED_Y  = 700.0f;  // 70.0 mm/s (1.75 rev/s for healthy back-EMF)

const int   DEFAULT_HOME_SENS     = 60;      // Homing sensitivity threshold for X and Z lead screws (SGTHRS=60)
const int   DEFAULT_HOME_SENS_Y   = 3;       // Homing sensitivity threshold for Y belt pulley (SGTHRS=3)

// X: Lead screw 8.0 mm/rev @ 8x microsteps -> 200.0 microsteps/mm.
// Z: Lead screw 8.0 mm/rev @ 2x microsteps -> 50.0 microsteps/mm.
// Y: GT2 belt (2.0 mm pitch) x 20-tooth pulley = 40.0 mm/rev -> 10.0 microsteps/mm @ 2x microsteps.
const float LEAD_SCREW_LEAD_MM     = 8.0f;
const float BELT_PULLEY_MM_REV_Y   = 40.0f; // 20 teeth * 2.0 mm GT2 pitch
const float MOTOR_FULL_STEPS_REV   = 200.0f;
const float DEFAULT_STEPS_PER_MM_X = (MOTOR_FULL_STEPS_REV * SETUP_MICROSTEPS_X) / LEAD_SCREW_LEAD_MM;   // 200.0f
const float DEFAULT_STEPS_PER_MM_Y = (MOTOR_FULL_STEPS_REV * SETUP_MICROSTEPS)   / BELT_PULLEY_MM_REV_Y; // 10.0f
const float DEFAULT_STEPS_PER_MM_Z = (MOTOR_FULL_STEPS_REV * SETUP_MICROSTEPS)   / LEAD_SCREW_LEAD_MM;   // 50.0f

// Homing direction per axis (true = reverse/negative towards 0, false = forward)
const bool DEFAULT_HOME_DIR_REVERSE_X = true;
const bool DEFAULT_HOME_DIR_REVERSE_Y = true;  // Reverse backward toward the back end stop at Y=0
const bool DEFAULT_HOME_DIR_REVERSE_Z = true;

// Motor shaft direction inversion (via TMC2209 shaft() UART register):
// S1 (X): +steps = +X (Right, away from home)  -> invertDir = false
// S2 (Y): +steps = +Y (Fore, into course)      -> invertDir = true (flips shaft so +steps moves forward)
// S3 (Z): +steps = +Z_down (Down toward floor) -> invertDir = false (raw + was Down)
const bool INVERT_DIR_X = false;
const bool INVERT_DIR_Y = true;
const bool INVERT_DIR_Z = false;

// Load cell calibrated scale factor (raw counts per gram):
// Calibrated using 21.6 g aluminum cube after empty tare (19.92 g avg at 682.3 -> 682.3 * 19.92 / 21.6 = 629.2).
float hx711ScaleFactor = 629.2f;

// Solenoid gripper relay logic:
// Because the solenoid is wired Normally Closed (NC):
//   - GRIP 1 (closed/gripping) -> relay LOW (de-energized, NC holds grip without heating solenoid)
//   - GRIP 0 (open/released)   -> relay HIGH (energized, opens gripper jaws)
// If your mechanical linkage works in the opposite sense, toggle `gripOpenRelayHigh`.
bool gripOpenRelayHigh = true;

// Hard thermal failsafe for the 12V solenoid relay (normal open pulses take < 5s).
// Forces PIN_RELAY LOW after 15s continuous energization and on any fault/sleep/abort
// to prevent >90C coil overheating.
const unsigned long RELAY_MAX_ON_MS = 15000UL;
bool relayOutputHigh = false;
unsigned long relayOnSinceMs = 0;

const unsigned long HOMING_RAMP_MS     = 250;    // Smooth linear acceleration ramp (250ms)
const unsigned long HOMING_BLANKING_MS = 350;    // Ignore DIAG during ramp & initial settle (350ms)
const unsigned long HOMING_TIMEOUT_MS  = 30000;  // minimum homing timeout; the real per-axis timeout scales with
                                                 // stroke/homeSpeed (X needs ~60 s for a full 227 mm stroke at 4 mm/s)

// Physical strokes from the homed origin (mm). Used for MOVE soft limits and forward-homing position.
const float TRAVEL_X_MM = 227.0f;
const float TRAVEL_Y_MM = 128.0f;
const float TRAVEL_Z_MM = 76.7f;

// Safe pull-away distances before homing so axes against bumpers have runway to accelerate
const float HOME_BACKOFF_MM_XZ = 5.0f;
const float HOME_BACKOFF_MM_Y  = 25.0f;
const unsigned long HOME_PRE_SETTLE_MS = 50;

// Per-axis cap applied to MOVE feedrates (microsteps/s). StallGuard thresholds are only valid near the speed they
// were calibrated at; the old firmware let F15 drive X/Z at 3000 steps/s (3.75x the calibrated 800 steps/s).
const float MAX_FEED_SPEED_X  = DEFAULT_MAX_SPEED_X;  // 5000 steps/s = 25.0 mm/s @ 8x
const float MAX_FEED_SPEED_XZ = DEFAULT_MAX_SPEED_XZ; // 1250 steps/s = 25.0 mm/s @ 2x
const float MAX_FEED_SPEED_Y  = DEFAULT_MAX_SPEED_Y;  // 700 steps/s = 70 mm/s

// Stall detection during moves only runs near full (cruise) speed
const float STALL_CHECK_SPEED_RATIO = 0.9f;
// ...and only when cruise speed is close to the calibrated homing speed (back-EMF too weak below this)
const float STALL_MIN_SPEED_RATIO_OF_HOME = 0.85f;
// ...and only after cruising for this long (Y belt settling transient after the accel ramp)
const unsigned long STALL_SETTLE_MS_XZ = 100;
const unsigned long STALL_SETTLE_MS_Y  = 450;
// ...and DIAG must be seen HIGH at least twice within this window (rejects single-sample glitches; DIAG may
// pulse once per full step while stalled, so we do not require it to be continuously HIGH)
const unsigned long STALL_CONFIRM_WINDOW_MS = 15;

// Allow the TMC2209 StealthChop automatic tuning (AT#1, standstill at run current) to settle after the
// drivers are (re-)enabled, before any motion that relies on StallGuard.
const unsigned long DRIVER_ENABLE_SETTLE_MS = 200;

// Wrist servo mechanical zero trim (degrees added to logical 0..180 commands)
// Tuned for MG90S replacement (-3 deg from prior 12.0f trim)
float servoZeroOffsetDeg = 9.0f;

// ============================================================
//  OBJECTS & STATE
// ============================================================
HardwareSerial TMCSerial(1);

TMC2209Stepper driver1(&TMCSerial, R_SENSE, 0b00);
TMC2209Stepper driver2(&TMCSerial, R_SENSE, 0b01);
TMC2209Stepper driver3(&TMCSerial, R_SENSE, 0b10);

AccelStepper stepper1(AccelStepper::DRIVER, PIN_STEP_1, PIN_DIR_1);
AccelStepper stepper2(AccelStepper::DRIVER, PIN_STEP_2, PIN_DIR_2);
AccelStepper stepper3(AccelStepper::DRIVER, PIN_STEP_3, PIN_DIR_3);

Servo servo;
HX711 scale;

struct Axis {
  const char*     name;
  char            coordCode;        // 'X', 'Y', or 'Z'
  int             diagPin;
  AccelStepper*   stepper;
  TMC2209Stepper* driver;
  bool            ready;            // true after a successful "setup"
  float           maxSpeed;
  float           acceleration;
  float           homeSpeed;
  int             moveSensitivity;  // SGTHRS for normal moves
  int             homeSensitivity;  // SGTHRS while homing
  float           stepsPerMm;       // Microsteps per physical millimeter
  float           mmPerRev;         // Millimeters per full motor revolution
  bool            homeReverse;      // Direction for automated HOME sequence
  bool            invertDir;        // Invert motor direction via TMC2209 shaft()
  int             currentMa;        // RMS current limit in mA
  float           maxTravelMm;      // Soft-limit stroke from homed origin
  float           homeBackoffMm;    // Pull-away distance before protocol HOME
  float           maxFeedSpeed;     // Cap for MOVE feedrates (microsteps/s)
  unsigned long   stallSettleMs;    // Cruise time before crash detection is armed
  int             microsteps;       // Microstep setting re-applied by setupAxis()

  // Runtime state (not part of the initializer lists below)
  bool            homed = false;           // Position is referenced to the homed origin
  unsigned long   cruiseSinceMs = 0;       // When the axis reached cruise speed (0 = not cruising)
  unsigned long   diagFirstHighMs = 0;     // First DIAG HIGH sample in the current confirm window (0 = none)
};

const int NUM_AXES = 3;
Axis axes[NUM_AXES] = {
  { "S1", 'X', PIN_DIAG_1, &stepper1, &driver1, false, DEFAULT_MAX_SPEED_X,  DEFAULT_ACCEL_X,  DEFAULT_HOME_SPEED_X,  SETUP_STALL_SENS_XZ, DEFAULT_HOME_SENS,   DEFAULT_STEPS_PER_MM_X, LEAD_SCREW_LEAD_MM,   DEFAULT_HOME_DIR_REVERSE_X, INVERT_DIR_X, SETUP_CURRENT_MA_XZ, TRAVEL_X_MM, HOME_BACKOFF_MM_XZ, MAX_FEED_SPEED_X,  STALL_SETTLE_MS_XZ, SETUP_MICROSTEPS_X },
  { "S2", 'Y', PIN_DIAG_2, &stepper2, &driver2, false, DEFAULT_MAX_SPEED_Y,  DEFAULT_ACCEL_Y,  DEFAULT_HOME_SPEED_Y,  SETUP_STALL_SENS_Y,  DEFAULT_HOME_SENS_Y, DEFAULT_STEPS_PER_MM_Y, BELT_PULLEY_MM_REV_Y, DEFAULT_HOME_DIR_REVERSE_Y, INVERT_DIR_Y, SETUP_CURRENT_MA_Y,  TRAVEL_Y_MM, HOME_BACKOFF_MM_Y,  MAX_FEED_SPEED_Y,  STALL_SETTLE_MS_Y,  SETUP_MICROSTEPS },
  { "S3", 'Z', PIN_DIAG_3, &stepper3, &driver3, false, DEFAULT_MAX_SPEED_XZ, DEFAULT_ACCEL_XZ, DEFAULT_HOME_SPEED_XZ, SETUP_STALL_SENS_XZ, DEFAULT_HOME_SENS,   DEFAULT_STEPS_PER_MM_Z, LEAD_SCREW_LEAD_MM,   DEFAULT_HOME_DIR_REVERSE_Z, INVERT_DIR_Z, SETUP_CURRENT_MA_XZ, TRAVEL_Z_MM, HOME_BACKOFF_MM_XZ, MAX_FEED_SPEED_XZ, STALL_SETTLE_MS_XZ, SETUP_MICROSTEPS },
};

// Set when an ESTOP line or hardware E-STOP switch triggers
bool estopLatched = false;
String pendingSerialCommand = "";

// Non-blocking millis()-based debounced switch state (active-LOW via INPUT_PULLUP)
struct DebouncedSwitch {
  int           pin;
  unsigned long debounceMs;
  bool          stablePressed;
  bool          lastRawPressed;
  unsigned long lastChangeMs;
};

DebouncedSwitch swStart = { PIN_SWITCH_START, SWITCH_DEBOUNCE_MS, false, false, 0 };
DebouncedSwitch swEstop = { PIN_SWITCH_ESTOP, ESTOP_DEBOUNCE_MS,  false, false, 0 };
DebouncedSwitch swChime = { PIN_SWITCH_CHIME, SWITCH_DEBOUNCE_MS, false, false, 0 };

struct ChimeSegment {
  float freqXHz;
  float freqYHz;
  float freqZHz;
  unsigned long durationMs;
  int dirX;
  int dirY;
  int dirZ;
};

struct SavedAxisSnapshot {
  bool     hasRegs;
  uint32_t gconf;
  uint32_t chopconf;
  uint32_t pwmconf;
  uint32_t iholdIrun;
  float    maxSpeed;
  float    acceleration;
  float    stepsPerMm;
  float    maxTravelMm;
  float    maxFeedSpeed;
  int      currentMa;
  int      microsteps;
  int      moveSensitivity;
  int      homeSensitivity;
  long     stepperPos;
};

float currentAngleDeg = 0.0f;
int   currentGripState = 1; // 1 = gripping (NC default), 0 = open

void writeServoLogicalAngle(float logicalDeg) {
  logicalDeg = constrain(logicalDeg, 0.0f, 180.0f);
  currentAngleDeg = logicalDeg;
  int physDeg = (int)roundf(constrain(logicalDeg + servoZeroOffsetDeg, 0.0f, 180.0f));
  servo.write(physDeg);
}

enum MotionResult { MOTION_DONE, MOTION_STALLED, MOTION_CANCELLED };

// Forward declarations
void printHelp();
void handleCommand(String command);
void initSwitch(DebouncedSwitch& sw);
void initSwitches();
bool pollSwitchPressedEdge(DebouncedSwitch& sw);
bool checkHardwareEstop();
void pollPanelSwitches();
bool setupAll(bool quietProtocol = false);
bool setupAxis(Axis& axis);
void sleepAll();
bool homeAxis(Axis& axis, bool reverse, bool withBackoff = true);
bool moveAxis(Axis& axis, long steps);
bool isStalled(Axis& axis);
void resetStallMonitor(Axis& axis);
bool pollCancelInput();
unsigned long homingTimeoutMs(Axis& axis, bool knownDistance);
MotionResult runAxisTo(Axis& axis, long target);
void handleMoveGcode(String params);
void handleHomeAllGcode();
void handleGripGcode(String params);
void handleTareGcode();
void handleWeighGcode();
void handleStatusGcode();
void handleRegsGcode();
void handleStepsGcode(String params);
void handleChimeGcode(String params, bool emitProtocolReply = true);
void applyAxisDriverSettings(Axis& axis);
bool restoreAxisFromChime(Axis& axis);
void captureAxisSnapshot(Axis& axis, SavedAxisSnapshot& snap);
bool restoreAxisSnapshot(Axis& axis, const SavedAxisSnapshot& snap);
void handleStepperCommand(Axis& axis, String param);
void handleHomeSettings(Axis& axis, String param);
void handleHomeCommand(String param);
void handleServoCommand(String param);
void handleRelayCommand(String param);
void handleLoadCellCommand();
void setRelayOutput(bool high, const char* reason = nullptr);
void checkRelayWatchdog();
void setCurrent(Axis& axis, int mA);
void setMicrosteps(Axis& axis, int microsteps);
void setSensitivity(Axis& axis, int sensitivity);
void setMaxSpeed(Axis& axis, float speed);
void setAcceleration(Axis& axis, float accel);
void readStallGuard(Axis& axis);

// Stepper step loops run jitter-free; loop task watchdog is disabled via disableLoopWDT().
// yieldWdt also checks the solenoid thermal watchdog when the relay is energized (single bool check otherwise).
inline void yieldWdt(unsigned long& /*lastYieldMs*/, unsigned long /*intervalMs*/ = 30) {
  if (relayOutputHigh) checkRelayWatchdog();
}

// ============================================================
//  ARDUINO ENTRY POINTS
// ============================================================
void setup() {
  // Disable ESP32 hardware brownout detector to prevent nuisance reboots
  // caused by transient voltage dips during stepper motor current surges
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Disable loop task watchdog timer so long homing strokes never panic reset
  disableLoopWDT();

  Serial.begin(115200);
  delay(500);

  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, HIGH);         // Drivers disabled until "setup" or first motion command

  pinMode(PIN_RELAY, OUTPUT);
  setRelayOutput(false);              // Relay off (Normally Closed solenoid in default state)

  initSwitches();

  servo.attach(PIN_SERVO, 500, 2400);
  writeServoLogicalAngle(0.0f);

  TMCSerial.begin(115200, SERIAL_8N1, PIN_TMC_RX, PIN_TMC_TX);

  // Initialize HX711 load cell amplifier
  scale.begin(PIN_HX711_DT, PIN_HX711_SCK);
  scale.set_scale(hx711ScaleFactor);

  for (int i = 0; i < NUM_AXES; i++) {
    pinMode(axes[i].diagPin, INPUT_PULLDOWN);
    axes[i].stepper->setMaxSpeed(axes[i].maxSpeed);
    axes[i].stepper->setAcceleration(axes[i].acceleration);
  }

  Serial.println("READY");
}

void loop() {
  checkRelayWatchdog();
  pollPanelSwitches();

  if (pendingSerialCommand.length() > 0) {
    String cmd = pendingSerialCommand;
    pendingSerialCommand = "";
    handleCommand(cmd);
  } else if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() > 0) {
      handleCommand(command);
    }
  }
}

// ============================================================
//  DEBOUNCED HARDWARE SWITCHES (HUE-DINI-NOV3 LS_1..LS_3)
// ============================================================
void initSwitch(DebouncedSwitch& sw) {
  pinMode(sw.pin, INPUT_PULLUP);
  bool raw = (digitalRead(sw.pin) == LOW);
  sw.lastRawPressed = raw;
  sw.stablePressed = raw;
  sw.lastChangeMs = millis();
}

void initSwitches() {
  pinMode(PIN_SWITCH_START, INPUT_PULLUP);
  pinMode(PIN_SWITCH_ESTOP, INPUT_PULLUP);
  pinMode(PIN_SWITCH_CHIME, INPUT_PULLUP);
  delay(5); // Let internal pull-ups settle before sampling initial states

  initSwitch(swStart);
  initSwitch(swEstop);
  initSwitch(swChime);

  if (swEstop.stablePressed) {
    estopLatched = true;
    Serial.println("EVENT:ESTOP");
  }
}

// Non-blocking millis()-based stable-window debounce.
// Returns true once per debounced release -> press (HIGH -> LOW) transition.
bool pollSwitchPressedEdge(DebouncedSwitch& sw) {
  bool rawPressed = (digitalRead(sw.pin) == LOW);
  unsigned long nowMs = millis();

  if (rawPressed != sw.lastRawPressed) {
    sw.lastRawPressed = rawPressed;
    sw.lastChangeMs = nowMs;
  }

  if ((nowMs - sw.lastChangeMs) >= sw.debounceMs && rawPressed != sw.stablePressed) {
    sw.stablePressed = rawPressed;
    if (sw.stablePressed) {
      return true;
    }
  }
  return false;
}

bool checkHardwareEstop() {
  if (pollSwitchPressedEdge(swEstop)) {
    estopLatched = true;
    Serial.println("EVENT:ESTOP");
    return true;
  }
  return swEstop.stablePressed;
}

void pollPanelSwitches() {
  // Priority 1: Switch 2 -> E-Stop (checked first so it preempts Start/Chime)
  if (pollSwitchPressedEdge(swEstop)) {
    estopLatched = true;
    sleepAll();
    setRelayOutput(false, "hardware estop switch");
    Serial.println("EVENT:ESTOP");
    return;
  }

  // Priority 2: Switch 1 -> Start
  if (pollSwitchPressedEdge(swStart)) {
    if (swEstop.stablePressed) {
      Serial.println("[WARN] Start switch ignored while E-STOP switch is active.");
    } else {
      estopLatched = false;
      Serial.println("EVENT:START_BTN");
    }
  }

  // Priority 3: Switch 3 -> Chime (suppress protocol OK/ERR so it never desyncs Pi serial)
  if (pollSwitchPressedEdge(swChime)) {
    if (swEstop.stablePressed) {
      Serial.println("[WARN] Chime switch ignored while E-STOP switch is active.");
    } else {
      Serial.println("EVENT:CHIME_BTN");
      handleChimeGcode("TADA", false);
    }
  }
}

// ============================================================
//  COMMAND ROUTER (Supports both Pi Protocol & Manual CLI)
// ============================================================
Axis* getAxis(char id) {
  int index = id - '1';
  if (index < 0 || index >= NUM_AXES) return nullptr;
  return &axes[index];
}

void handleCommand(String rawCommand) {
  rawCommand.trim();
  if (rawCommand.length() == 0) return;

  // Silently ignore echoed replies, firmware debug output, comments, or stray OKs
  if (rawCommand.startsWith("->") || rawCommand.startsWith("[") ||
      rawCommand.startsWith("#") || rawCommand.startsWith("//") ||
      rawCommand.equalsIgnoreCase("OK")) {
    return;
  }

  // Also ignore echoed homing/status text if the "-> " prefix was stripped
  String lowerCheck = rawCommand;
  lowerCheck.toLowerCase();
  if (lowerCheck.indexOf(" homed") >= 0 || lowerCheck.indexOf(" pulling away") >= 0 ||
      lowerCheck.indexOf(" ready (") >= 0 || lowerCheck.indexOf(" position set to") >= 0) {
    return;
  }

  String upper = rawCommand;
  upper.toUpperCase();

  // --- Tier 2 (Raspberry Pi Zero W) Deterministic Protocol Commands ---
  if (upper == "PING") {
    Serial.println("OK");
    return;
  }
  if (upper == "HOME") {
    handleHomeAllGcode();
    return;
  }
  if (upper.startsWith("MOVE ") || upper == "MOVE") {
    handleMoveGcode(upper.substring(4));
    return;
  }
  if (upper.startsWith("GRIP ")) {
    handleGripGcode(upper.substring(5));
    return;
  }
  if (upper == "TARE") {
    handleTareGcode();
    return;
  }
  if (upper == "WEIGH") {
    handleWeighGcode();
    return;
  }
  if (upper == "STATUS") {
    handleStatusGcode();
    return;
  }
  if (upper == "REGS") {
    handleRegsGcode();
    return;
  }
  if (upper.startsWith("STEPS ")) {
    handleStepsGcode(upper.substring(6));
    return;
  }
  if (upper == "CHIME" || upper.startsWith("CHIME ")) {
    handleChimeGcode(upper.length() > 5 ? upper.substring(6) : "");
    return;
  }
  if (upper == "ESTOP") {
    sleepAll();
    setRelayOutput(false);
    Serial.println("OK");
    return;
  }

  // --- Interactive Manual Commands (from HUE-DINI-NOV3.txt) ---
  String command = rawCommand;
  command.toLowerCase();

  if (command == "setup") {
    if (setupAll(false)) {
      Serial.println("OK");
    } else {
      setRelayOutput(false, "setup failed");
      Serial.println("ERR:TMC_SETUP_FAILED");
    }
  }
  else if (command == "sleep") {
    sleepAll();
    Serial.println("OK");
  }
  else if (command == "help")            { printHelp(); Serial.println("OK"); }
  else if (command == "l")               { handleLoadCellCommand(); Serial.println("OK"); }
  else if (command == "chime" || command.startsWith("chime ")) { handleChimeGcode(command.length() > 5 ? command.substring(6) : ""); }
  else if (command.startsWith("home ") || command == "home")  { handleHomeCommand(command.length() > 5 ? command.substring(5) : ""); Serial.println("OK"); }
  else if (command.startsWith("servo ")) { handleServoCommand(command.substring(6)); Serial.println("OK"); }
  else if (command.startsWith("relay ")) { handleRelayCommand(command.substring(6)); Serial.println("OK"); }
  else if (command.length() > 3 && command.charAt(0) == 's' &&
           isDigit(command.charAt(1)) && command.charAt(2) == ' ') {
    Axis* axis = getAxis(command.charAt(1));
    if (axis) { handleStepperCommand(*axis, command.substring(3)); Serial.println("OK"); }
    else      Serial.println("ERR:UNKNOWN_STEPPER");
  }
  else {
    Serial.println("ERR:UNKNOWN_COMMAND");
  }
}

// ============================================================
//  PI ZERO PROTOCOL HANDLERS
// ============================================================
bool ensureDriversReady() {
  for (int i = 0; i < NUM_AXES; i++) {
    if (!axes[i].ready) {
      return setupAll(true);
    }
  }
  return true;
}

void handleHomeAllGcode() {
  if (checkHardwareEstop()) {
    setRelayOutput(false, "hardware estop");
    Serial.println("ERR:ESTOP");
    return;
  }
  if (!ensureDriversReady()) {
    setRelayOutput(false, "drivers not ready");
    Serial.println("ERR:TMC_NOT_READY");
    return;
  }

  // Guarantee every axis has its calibrated microsteps, interpolation, current, and StealthChop before homing
  bool restoredAny = false;
  for (int i = 0; i < NUM_AXES; i++) {
    if (axes[i].driver->microsteps() != axes[i].microsteps ||
        axes[i].driver->en_spreadCycle() ||
        !axes[i].driver->intpol()) {
      restoreAxisFromChime(axes[i]);
      restoredAny = true;
    }
  }
  if (restoredAny) {
    delay(DRIVER_ENABLE_SETTLE_MS);
  }

  // Home Z (S3) first so the end-effector is clear of obstacles, then X (S1) and Y (S2)
  estopLatched = false;
  int homeOrder[3] = { 2, 0, 1 };
  for (int k = 0; k < 3; k++) {
    Axis& ax = axes[homeOrder[k]];
    if (!homeAxis(ax, ax.homeReverse, true)) {
      if (estopLatched) {
        setRelayOutput(false, "homing estop");
        Serial.println("ERR:ESTOP");
        return;
      }
      setRelayOutput(false, "homing failed");
      Serial.print("ERR:HOME_FAILED_");
      Serial.println(ax.coordCode);
      return;
    }
  }
  Serial.println("OK");
}

bool parseCoordToken(const String& params, char key, float& outVal) {
  int idx = params.indexOf(key);
  if (idx < 0) return false;
  int start = idx + 1;
  int end = start;
  while (end < (int)params.length() && !isSpace(params.charAt(end))) {
    end++;
  }
  String numStr = params.substring(start, end);
  if (numStr.length() == 0) return false;
  outVal = numStr.toFloat();
  return true;
}

void handleMoveGcode(String params) {
  if (checkHardwareEstop()) {
    setRelayOutput(false, "hardware estop");
    Serial.println("ERR:ESTOP");
    return;
  }
  if (!ensureDriversReady()) {
    Serial.println("ERR:TMC_NOT_READY");
    return;
  }

  float feedrateMmPerSec = 0.0f;
  if (parseCoordToken(params, 'F', feedrateMmPerSec) && feedrateMmPerSec > 0.0f) {
    for (int i = 0; i < NUM_AXES; i++) {
      // Per-axis cap: F is in mm/s for every axis, but X/Z (200 steps/mm) and Y (10 steps/mm) differ 20x, so an
      // uncapped F15 asked for 3000 steps/s on X/Z (step loss risk, StallGuard uncalibrated) and 150 steps/s on Y
      // (too slow for StallGuard to work at all).
      float spdSteps = constrain(feedrateMmPerSec * axes[i].stepsPerMm, 10.0f, axes[i].maxFeedSpeed);
      axes[i].maxSpeed = spdSteps;
      axes[i].stepper->setMaxSpeed(spdSteps);
    }
  }

  // Guard EE rotation clearance: wait until Y >= 10.0 mm before rotating servo A
  // so the end-effector never swings into the rear end stop near Y=0.
  const float SERVO_SAFE_Y_MM = 10.0f;
  float targetAngle = currentAngleDeg;
  bool hasPendingAngle = parseCoordToken(params, 'A', targetAngle) &&
                         fabs(targetAngle - currentAngleDeg) > 0.1f;
  if (hasPendingAngle) {
    float startYmm = (float)axes[1].stepper->currentPosition() / axes[1].stepsPerMm;
    if (startYmm >= SERVO_SAFE_Y_MM) {
      writeServoLogicalAngle(targetAngle);
      hasPendingAngle = false;
    }
  }

  // Set target positions for X (S1), Y (S2), Z (S3) with soft-limit safety clamping
  for (int i = 0; i < NUM_AXES; i++) {
    float targetMm = 0.0f;
    if (parseCoordToken(params, axes[i].coordCode, targetMm)) {
      targetMm = constrain(targetMm, 0.0f, axes[i].maxTravelMm);
      long targetSteps = lroundf(targetMm * axes[i].stepsPerMm);
      axes[i].stepper->moveTo(targetSteps);
    } else {
      axes[i].stepper->moveTo(axes[i].stepper->currentPosition());
    }
    resetStallMonitor(axes[i]);
  }

  // Coordinated multi-axis step loop with StallGuard monitoring and responsive ESTOP
  unsigned long lastWdtYieldMs = millis();
  while (axes[0].stepper->distanceToGo() != 0 ||
         axes[1].stepper->distanceToGo() != 0 ||
         axes[2].stepper->distanceToGo() != 0) {
    yieldWdt(lastWdtYieldMs);
    if (hasPendingAngle) {
      float curYmm = (float)axes[1].stepper->currentPosition() / axes[1].stepsPerMm;
      if (curYmm >= SERVO_SAFE_Y_MM) {
        writeServoLogicalAngle(targetAngle);
        hasPendingAngle = false;
      }
    }
    if (checkHardwareEstop()) {
      for (int j = 0; j < NUM_AXES; j++) {
        axes[j].stepper->setCurrentPosition(axes[j].stepper->currentPosition());
      }
      sleepAll();
      setRelayOutput(false);
      Serial.println("ERR:ESTOP");
      return;
    }
    if (Serial.available() > 0) {
      String inCmd = Serial.readStringUntil('\n');
      inCmd.trim();
      inCmd.toUpperCase();
      if (inCmd == "ESTOP" || inCmd == "X" || inCmd == "SLEEP") {
        for (int j = 0; j < NUM_AXES; j++) {
          axes[j].stepper->setCurrentPosition(axes[j].stepper->currentPosition());
        }
        sleepAll();
        setRelayOutput(false);
        Serial.println("ERR:ESTOP");
        return;
      }
    }

    for (int i = 0; i < NUM_AXES; i++) {
      if (axes[i].stepper->distanceToGo() != 0) {
        axes[i].stepper->run();
        if (isStalled(axes[i])) {
          for (int j = 0; j < NUM_AXES; j++) {
            axes[j].stepper->setCurrentPosition(axes[j].stepper->currentPosition());
          }
          // Immediately cut power to steppers so motor does not sustain power against the stop!
          // (sleepAll() also invalidates the homed reference: the axis has lost steps.)
          sleepAll();
          Serial.print("ERR:STALL_");
          Serial.println(axes[i].coordCode);
          return;
        }
      }
    }
  }

  if (hasPendingAngle) {
    float endYmm = (float)axes[1].stepper->currentPosition() / axes[1].stepsPerMm;
    if (endYmm >= SERVO_SAFE_Y_MM) {
      writeServoLogicalAngle(targetAngle);
    } else {
      Serial.println("[WARN] Servo rotation deferred: Y < 10.0 mm clearance");
    }
  }

  Serial.println("OK");
}

void handleGripGcode(String params) {
  params.trim();
  if (params == "INVERT") {
    gripOpenRelayHigh = !gripOpenRelayHigh;
    Serial.println("OK");
    return;
  }

  int state = params.toInt(); // 1 = close/grip, 0 = open/release
  currentGripState = (state != 0) ? 1 : 0;

  // Normally Closed (NC) solenoid:
  // When gripOpenRelayHigh == true:
  //   GRIP 0 (open)  -> Relay HIGH (energize solenoid to pull open against spring)
  //   GRIP 1 (close) -> Relay LOW  (de-energize solenoid so spring/NC closes on block)
  bool relayHigh = (currentGripState == 0) ? gripOpenRelayHigh : !gripOpenRelayHigh;
  setRelayOutput(relayHigh);
  delay(150); // Allow mechanical jaw settling
  Serial.println("OK");
}

void handleTareGcode() {
  if (scale.wait_ready_timeout(500)) {
    scale.tare(10);
    Serial.println("OK");
  } else {
    Serial.println("ERR:HX711_NOT_READY");
  }
}

void handleWeighGcode() {
  if (scale.wait_ready_timeout(500)) {
    float massGrams = scale.get_units(10);
    Serial.print("WEIGHT:");
    Serial.println(massGrams, 2);
    Serial.println("OK");
  } else {
    Serial.println("ERR:HX711_NOT_READY");
  }
}

void handleStatusGcode() {
  float xMm = axes[0].stepper->currentPosition() / axes[0].stepsPerMm;
  float yMm = axes[1].stepper->currentPosition() / axes[1].stepsPerMm;
  float zMm = axes[2].stepper->currentPosition() / axes[2].stepsPerMm;
  Serial.print("POS X:");
  Serial.print(xMm, 2);
  Serial.print(" Y:");
  Serial.print(yMm, 2);
  Serial.print(" Z:");
  Serial.print(zMm, 2);
  Serial.print(" A:");
  Serial.print(currentAngleDeg, 1);
  Serial.print(" GRIP:");
  Serial.print(currentGripState);
  Serial.print(" HOMED:");
  for (int i = 0; i < NUM_AXES; i++) Serial.print(axes[i].homed ? axes[i].coordCode : '-');
  Serial.println();
  Serial.println("OK");
}

void handleRegsGcode() {
  for (int i = 0; i < NUM_AXES; i++) {
    Axis& ax = axes[i];
    Serial.print("REGS "); Serial.print(ax.name);
    Serial.print(" GCONF=0x"); Serial.print(ax.driver->GCONF(), HEX);
    Serial.print(" GSTAT=0x"); Serial.print(ax.driver->GSTAT(), HEX);
    Serial.print(" CHOP=0x"); Serial.print(ax.driver->CHOPCONF(), HEX);
    Serial.print(" PWM=0x"); Serial.print(ax.driver->PWMCONF(), HEX);
    Serial.print(" DRV=0x"); Serial.print(ax.driver->DRV_STATUS(), HEX);
    Serial.print(" STEALTH="); Serial.print(ax.driver->stealth());
    Serial.print(" CS_ACT="); Serial.print(ax.driver->cs_actual());
    Serial.print(" IRUN="); Serial.print(ax.driver->irun());
    Serial.print(" IHOLD="); Serial.print(ax.driver->ihold());
    Serial.print(" MS="); Serial.print(ax.driver->microsteps());
    Serial.print(" INTPOL="); Serial.print(ax.driver->intpol());
    Serial.print(" SPREAD="); Serial.print(ax.driver->en_spreadCycle());
    Serial.print(" VSENSE="); Serial.print(ax.driver->vsense());
    Serial.print(" MREG="); Serial.print(ax.driver->mstep_reg_select());
    Serial.print(" PWM_OFS_A="); Serial.print(ax.driver->pwm_ofs_auto());
    Serial.print(" PWM_GRAD_A="); Serial.print(ax.driver->pwm_grad_auto());
    Serial.print(" PWM_SUM="); Serial.print(ax.driver->pwm_scale_sum());
    Serial.print(" TPWM=0x"); Serial.print(ax.driver->TPWMTHRS(), HEX);
    Serial.print(" TCOOL=0x"); Serial.print(ax.driver->TCOOLTHRS(), HEX);
    Serial.print(" SGTHRS="); Serial.print(ax.driver->SGTHRS());
    Serial.print(" MSCNT="); Serial.print(ax.driver->MSCNT());
    Serial.print(" SPM="); Serial.print(ax.stepsPerMm, 1);
    Serial.print(" VMAX="); Serial.print(ax.maxSpeed, 0);
    Serial.print(" ACC="); Serial.print(ax.acceleration, 0);
    Serial.print(" LIM="); Serial.println(ax.maxTravelMm, 1);
  }
  Serial.println("OK");
}

void handleStepsGcode(String params) {
  for (int i = 0; i < NUM_AXES; i++) {
    float spm = 0.0f;
    if (parseCoordToken(params, axes[i].coordCode, spm) && spm > 0.0f) {
      axes[i].stepsPerMm = spm;
    }
  }
  Serial.println("OK");
}

// ============================================================
//  INTERACTIVE MANUAL CLI HANDLERS (HUE-DINI-NOV3.txt)
// ============================================================
void printHelp() {
  Serial.println("\n--- HUE-DINI ESP32 Controller ---");
  Serial.println("Automated Pi Zero Protocol:");
  Serial.println("  SETUP | HOME | MOVE X<mm> Y<mm> Z<mm> A<deg> F<mm_s>");
  Serial.println("  GRIP <0|1> | TARE | WEIGH | STATUS | STEPS X<f> Y<f> Z<f> | CHIME [TADA|WIN95] | ESTOP");
  Serial.println("Interactive Manual Commands (s1=X, s2=Y, s3=Z):");
  Serial.println("  setup       -> Enable all steppers, test UART, apply defaults");
  Serial.println("  sleep       -> Disable all steppers (run 'setup' to wake)");
  Serial.println("  s1 500      -> Move stepper forward 500 steps");
  Serial.println("  s1 -500     -> Move stepper backward 500 steps");
  Serial.println("  s1 c600     -> Set current limit (mA)");
  Serial.println("  s1 m2       -> Set microstepping (1,2,4,8,...,256)");
  Serial.println("  s1 s80      -> Set StallGuard sensitivity (0-255)");
  Serial.println("  s1 v1000    -> Set max speed (steps/s)");
  Serial.println("  s1 a2000    -> Set acceleration (steps/s^2)");
  Serial.println("  s1 r        -> Read StallGuard result");
  Serial.println("  s1 hv500    -> Set homing speed (steps/s)");
  Serial.println("  s1 hs80     -> Set homing sensitivity (0-255)");
  Serial.println("  home s1     -> Home forward until stall");
  Serial.println("  home s1 -   -> Home backward until stall");
  Serial.println("  servo 90    -> Move servo to angle (0-180)");
  Serial.println("  relay 2     -> Relay ON for 2 seconds");
  Serial.println("  l           -> Read load cell measurement");
  Serial.println("  chime       -> Play Windows 3.1 Tada! chime ('chime win95' for Win95 startup)");
  Serial.println("  help        -> Show this list");
  Serial.println("---------------------------------\n");
}

void handleStepperCommand(Axis& axis, String param) {
  param.trim();
  if (param.length() == 0) return;

  // Support "s1 home" and "s1 home -" as intuitive aliases for "home s1" / "home s1 -"
  if (param.startsWith("home")) {
    bool reverse = (param.indexOf('-') >= 0);
    homeAxis(axis, reverse);
    return;
  }
  // Ignore echoed status line if "s1 homed..." is parsed
  if (param.startsWith("homed")) {
    return;
  }

  char sub = param.charAt(0);
  String valueStr = param.substring(1);

  switch (sub) {
    case 'c': setCurrent(axis, valueStr.toInt());        break;
    case 'm': setMicrosteps(axis, valueStr.toInt());     break;
    case 's': setSensitivity(axis, valueStr.toInt());    break;
    case 'v': setMaxSpeed(axis, valueStr.toFloat());     break;
    case 'a': setAcceleration(axis, valueStr.toFloat()); break;
    case 'r': readStallGuard(axis);                      break;
    case 'h': handleHomeSettings(axis, valueStr);        break;
    default:  moveAxis(axis, param.toInt());             break;
  }
}

void handleHomeSettings(Axis& axis, String param) {
  param.trim();
  if (param.length() == 0) {
    Serial.print("-> "); Serial.print(axis.name);
    Serial.print(" homing speed: "); Serial.print(axis.homeSpeed);
    Serial.print(" steps/s, homing sensitivity: "); Serial.println(axis.homeSensitivity);
    return;
  }

  char sub = param.charAt(0);
  String valueStr = param.substring(1);

  if (sub == 'v') {
    axis.homeSpeed = constrain(valueStr.toFloat(), 10.0f, 20000.0f);
    Serial.print("-> "); Serial.print(axis.name);
    Serial.print(" homing speed: "); Serial.print(axis.homeSpeed);
    Serial.println(" steps/s");
  } else if (sub == 's') {
    axis.homeSensitivity = constrain((int)valueStr.toInt(), 0, 255);
    Serial.print("-> "); Serial.print(axis.name);
    Serial.print(" homing sensitivity: "); Serial.println(axis.homeSensitivity);
  } else if (param.startsWith("ome")) {
    // If called via "s1 home" or "s1 home -"
    bool reverse = (param.indexOf('-') >= 0);
    homeAxis(axis, reverse);
  } else {
    // Non-fatal informative readout instead of fatal [ERROR]
    Serial.print("-> "); Serial.print(axis.name);
    Serial.print(" homing speed: "); Serial.print(axis.homeSpeed);
    Serial.print(" steps/s, homing sensitivity: "); Serial.println(axis.homeSensitivity);
  }
}

void handleHomeCommand(String param) {
  param.trim();
  if (param.length() == 0 || param.equalsIgnoreCase("all")) {
    handleHomeAllGcode();
    return;
  }
  if (param.length() < 2 || (param.charAt(0) != 's' && param.charAt(0) != 'S')) {
    Serial.println("[ERROR] Use 'home s1' or 'home s1 -'.");
    return;
  }

  Axis* axis = getAxis(param.charAt(1));
  if (!axis) {
    Serial.println("[ERROR] Unknown stepper. Use s1, s2 or s3.");
    return;
  }

  String direction = param.substring(2);
  direction.trim();
  if (direction.length() == 0)  homeAxis(*axis, false);
  else if (direction == "-")    homeAxis(*axis, true);
  else Serial.println("[ERROR] Use 'home s1' or 'home s1 -'.");
}

void handleServoCommand(String param) {
  param.trim();
  if (param.startsWith("offset ")) {
    servoZeroOffsetDeg = constrain(param.substring(7).toFloat(), -45.0f, 45.0f);
    writeServoLogicalAngle(currentAngleDeg);
    Serial.print("-> Servo zero offset set to ");
    Serial.print(servoZeroOffsetDeg, 1);
    Serial.print(" deg (logical ");
    Serial.print(currentAngleDeg, 1);
    Serial.print(" deg -> physical ");
    Serial.print(constrain(currentAngleDeg + servoZeroOffsetDeg, 0.0f, 180.0f), 1);
    Serial.println(" deg)");
    return;
  }
  float angle = constrain(param.toFloat(), 0.0f, 180.0f);
  writeServoLogicalAngle(angle);
  Serial.print("-> Servo moved to logical ");
  Serial.print(angle, 1);
  Serial.print(" deg (physical ");
  Serial.print(constrain(angle + servoZeroOffsetDeg, 0.0f, 180.0f), 1);
  Serial.println(" deg)");
}

void handleRelayCommand(String param) {
  param.trim();
  if (param == "on") {
    setRelayOutput(true);
    Serial.println("-> Relay ON (15s thermal watchdog active)");
    return;
  }
  if (param == "off") {
    setRelayOutput(false);
    Serial.println("-> Relay OFF");
    return;
  }

  int seconds = param.toInt();
  if (seconds <= 0) seconds = 2;
  if (seconds > (int)(RELAY_MAX_ON_MS / 1000UL)) {
    seconds = (int)(RELAY_MAX_ON_MS / 1000UL);
  }

  Serial.print("-> Relay ON for ");
  Serial.print(seconds);
  Serial.println(" seconds...");

  setRelayOutput(true);
  unsigned long holdStartMs = millis();
  while (relayOutputHigh && (millis() - holdStartMs) < (unsigned long)seconds * 1000UL) {
    checkRelayWatchdog();
    if (pollCancelInput()) break;
    delay(20);
  }
  setRelayOutput(false);

  Serial.println("-> Relay OFF");
}

void handleLoadCellCommand() {
  if (scale.wait_ready_timeout(500)) {
    long rawValue = scale.read();
    float grams = scale.get_units(5);
    Serial.print("-> Load Cell Raw: ");
    Serial.print(rawValue);
    Serial.print(" | Calibrated: ");
    Serial.print(grams, 2);
    Serial.println(" g");
  } else {
    Serial.println("[ERROR] HX711 not found or not ready.");
  }
}

// ============================================================
//  SETUP & SLEEP (ALL STEPPERS)
// ============================================================
bool setupAll(bool quietProtocol) {
  if (checkHardwareEstop()) {
    digitalWrite(PIN_EN, HIGH);
    if (!quietProtocol) {
      Serial.println("[ERROR] Hardware E-STOP switch is active. Release it before setup.");
    }
    return false;
  }

  if (!quietProtocol) {
    Serial.println("-> Setting up steppers...");
  }

  digitalWrite(PIN_EN, LOW);           // Enable drivers

  int readyCount = 0;
  for (int i = 0; i < NUM_AXES; i++) {
    if (setupAxis(axes[i])) readyCount++;
  }

  if (readyCount == 0) {
    digitalWrite(PIN_EN, HIGH);        // Nothing responded, leave drivers disabled
    if (!quietProtocol) {
      Serial.println("[ERROR] No steppers are ready.");
    }
    return false;
  }

  estopLatched = false;

  // Let StealthChop AT#1 (standstill current regulation) settle before StallGuard-dependent motion.
  // Without this, a HOME right after re-enabling (e.g. after a stall/ESTOP put the drivers to sleep)
  // started homing on an untuned driver.
  delay(DRIVER_ENABLE_SETTLE_MS);

  if (!quietProtocol) {
    Serial.print("-> Setup done: ");
    Serial.print(readyCount);
    Serial.print(" of ");
    Serial.print(NUM_AXES);
    Serial.println(" steppers ready.");
  }
  return (readyCount == NUM_AXES);
}

static inline float axisHoldMultiplier(const Axis& axis) {
  // Y runs at 700 mA IRUN for belt inertia/StallGuard4; cut standstill IHOLD to 25% (~175 mA)
  // so its standby current and coil heating match X and Z (50% of 350 mA = ~175 mA).
  return (axis.coordCode == 'Y') ? 0.25f : 0.50f;
}

void applyAxisDriverSettings(Axis& axis) {
  // Required for UART control + StallGuard
  axis.driver->pdn_disable(true);
  axis.driver->mstep_reg_select(true);
  axis.driver->I_scale_analog(false);
  axis.driver->multistep_filt(true);
  axis.driver->shaft(axis.invertDir);
  axis.driver->toff(5);
  axis.driver->en_spreadCycle(false);  // StealthChop on (needed for StallGuard)
  axis.driver->pwm_autoscale(true);
  axis.driver->pwm_autograd(true);
  axis.driver->intpol(true);           // Enable TMC2209 internal 256x microstep interpolation for silent, smooth motion
  axis.driver->TCOOLTHRS(0xFFFFF);     // Enable StallGuard/DIAG at all speeds above ~0
  axis.driver->TPWMTHRS(0);

  // Restore default maxSpeed and acceleration so prior MOVE feedrate caps never taint SETUP/HOME
  axis.maxSpeed     = (axis.coordCode == 'Y') ? DEFAULT_MAX_SPEED_Y :
                      (axis.coordCode == 'X') ? DEFAULT_MAX_SPEED_X : DEFAULT_MAX_SPEED_XZ;
  axis.acceleration = (axis.coordCode == 'Y') ? DEFAULT_ACCEL_Y :
                      (axis.coordCode == 'X') ? DEFAULT_ACCEL_X : DEFAULT_ACCEL_XZ;
  axis.stepper->setMaxSpeed(axis.maxSpeed);
  axis.stepper->setAcceleration(axis.acceleration);

  // User-facing defaults
  axis.driver->rms_current(axis.currentMa, axisHoldMultiplier(axis));
  axis.driver->TPOWERDOWN(15); // ~327 ms standstill delay before dropping IRUN -> IHOLD
  axis.driver->iholddelay(4);  // Smooth standstill current ramp-down
  axis.driver->microsteps(axis.microsteps);   // was SETUP_MICROSTEPS: silently undid "s1 m<n>" while stepsPerMm kept the new value
  axis.driver->SGTHRS(axis.moveSensitivity);
}

void captureAxisSnapshot(Axis& axis, SavedAxisSnapshot& snap) {
  while (TMCSerial.available()) TMCSerial.read();
  snap.maxSpeed        = axis.maxSpeed;
  snap.acceleration    = axis.acceleration;
  snap.stepsPerMm      = axis.stepsPerMm;
  snap.maxTravelMm     = axis.maxTravelMm;
  snap.maxFeedSpeed    = axis.maxFeedSpeed;
  snap.currentMa       = axis.currentMa;
  snap.microsteps      = axis.microsteps;
  snap.moveSensitivity = axis.moveSensitivity;
  snap.homeSensitivity = axis.homeSensitivity;
  snap.stepperPos      = axis.stepper->currentPosition();

  uint32_t g = axis.driver->GCONF();
  bool gOk = !axis.driver->CRCerror;
  uint32_t c = axis.driver->CHOPCONF();
  bool cOk = !axis.driver->CRCerror && c != 0;
  uint32_t p = axis.driver->PWMCONF();
  bool pOk = !axis.driver->CRCerror && p != 0;
  snap.gconf     = g;
  snap.chopconf  = c;
  snap.pwmconf   = p;
  snap.iholdIrun = axis.driver->IHOLD_IRUN();
  snap.hasRegs   = (gOk && cOk && pOk);
}

bool restoreAxisSnapshot(Axis& axis, const SavedAxisSnapshot& snap) {
  axis.maxSpeed        = snap.maxSpeed;
  axis.acceleration    = snap.acceleration;
  axis.stepsPerMm      = snap.stepsPerMm;
  axis.maxTravelMm     = snap.maxTravelMm;
  axis.maxFeedSpeed    = snap.maxFeedSpeed;
  axis.currentMa       = snap.currentMa;
  axis.microsteps      = snap.microsteps;
  axis.moveSensitivity = snap.moveSensitivity;
  axis.homeSensitivity = snap.homeSensitivity;
  axis.stepper->setMaxSpeed(axis.maxSpeed);
  axis.stepper->setAcceleration(axis.acceleration);

  for (int attempt = 0; attempt < 4; attempt++) {
    while (TMCSerial.available()) TMCSerial.read();
    uint8_t gstat = axis.driver->GSTAT();
    if (!axis.driver->CRCerror && (gstat & 0x01)) {
      axis.driver->GSTAT(0x07); // Clear reset / UV flags after any transient droop
      axis.driver->begin();
    }

    uint8_t ifcntBefore = axis.driver->IFCNT();
    if (snap.hasRegs) {
      axis.driver->GCONF(snap.gconf);
      axis.driver->CHOPCONF(snap.chopconf);
      axis.driver->PWMCONF(snap.pwmconf);
      axis.driver->IHOLD_IRUN(snap.iholdIrun);
      axis.driver->TPOWERDOWN(15);
      axis.driver->TCOOLTHRS(0xFFFFF);
      axis.driver->TPWMTHRS(0);
      axis.driver->SGTHRS(axis.moveSensitivity);
    } else {
      applyAxisDriverSettings(axis);
      axis.maxSpeed     = snap.maxSpeed;
      axis.acceleration = snap.acceleration;
      axis.stepper->setMaxSpeed(axis.maxSpeed);
      axis.stepper->setAcceleration(axis.acceleration);
    }
    delay(3);
    uint8_t ifcntAfter = axis.driver->IFCNT();

    if (snap.hasRegs) {
      uint32_t gNow = axis.driver->GCONF();
      bool gOk = !axis.driver->CRCerror && (gNow == snap.gconf);
      uint32_t cNow = axis.driver->CHOPCONF();
      bool cOk = !axis.driver->CRCerror && (cNow == snap.chopconf);
      uint32_t pNow = axis.driver->PWMCONF();
      bool pOk = !axis.driver->CRCerror && (pNow == snap.pwmconf);
      if (gOk && cOk && pOk && ifcntAfter != ifcntBefore) {
        return true;
      }
    } else if (ifcntAfter != ifcntBefore &&
               axis.driver->microsteps() == axis.microsteps &&
               !axis.driver->en_spreadCycle() &&
               axis.driver->intpol()) {
      return true;
    }
    delay(5);
  }

  axis.driver->begin();
  applyAxisDriverSettings(axis);
  axis.maxSpeed     = snap.maxSpeed;
  axis.acceleration = snap.acceleration;
  axis.stepper->setMaxSpeed(axis.maxSpeed);
  axis.stepper->setAcceleration(axis.acceleration);
  return (axis.driver->microsteps() == axis.microsteps && !axis.driver->en_spreadCycle());
}

bool restoreAxisFromChime(Axis& axis) {
  SavedAxisSnapshot fallbackSnap;
  fallbackSnap.hasRegs         = false;
  fallbackSnap.maxSpeed        = axis.maxSpeed;
  fallbackSnap.acceleration    = axis.acceleration;
  fallbackSnap.stepsPerMm      = axis.stepsPerMm;
  fallbackSnap.maxTravelMm     = axis.maxTravelMm;
  fallbackSnap.maxFeedSpeed    = axis.maxFeedSpeed;
  fallbackSnap.currentMa       = axis.currentMa;
  fallbackSnap.microsteps      = axis.microsteps;
  fallbackSnap.moveSensitivity = axis.moveSensitivity;
  fallbackSnap.homeSensitivity = axis.homeSensitivity;
  fallbackSnap.stepperPos      = axis.stepper->currentPosition();
  return restoreAxisSnapshot(axis, fallbackSnap);
}

bool setupAxis(Axis& axis) {
  axis.driver->begin();

  uint8_t result = axis.driver->test_connection();
  if (result != 0) {
    axis.ready = false;
    Serial.print("[ERROR] "); Serial.print(axis.name);
    Serial.print(" UART test failed (code "); Serial.print(result);
    Serial.println(result == 1
      ? "): no response. Check UART wiring, driver address pins (MS1/MS2) and logic power."
      : "): driver not responding. Check motor supply (VM), address pins and UART wiring.");
    return false;
  }

  applyAxisDriverSettings(axis);

  axis.ready = true;
  Serial.print("-> "); Serial.print(axis.name);
  Serial.print(" ready (");
  Serial.print(axis.currentMa); Serial.print(" mA, microsteps ");
  Serial.print(axis.microsteps); Serial.print(", steps/mm ");
  Serial.print(axis.stepsPerMm, 1); Serial.println(")");
  return true;
}

void setRelayOutput(bool high, const char* reason) {
  if (high) {
    if (!relayOutputHigh) {
      unsigned long nowMs = millis();
      relayOnSinceMs = (nowMs == 0) ? 1UL : nowMs;
      relayOutputHigh = true;
    }
    digitalWrite(PIN_RELAY, HIGH);
  } else {
    bool wasHigh = relayOutputHigh;
    digitalWrite(PIN_RELAY, LOW);
    relayOutputHigh = false;
    relayOnSinceMs = 0;
    if (gripOpenRelayHigh) {
      currentGripState = 1; // NC solenoid returns to closed/safe state when de-energized
    }
    if (wasHigh && reason != nullptr) {
      Serial.print("[ALERT] Solenoid relay forced OFF (");
      Serial.print(reason);
      Serial.println(").");
    }
  }
}

void checkRelayWatchdog() {
  if (relayOutputHigh && relayOnSinceMs != 0) {
    if (millis() - relayOnSinceMs >= RELAY_MAX_ON_MS) {
      setRelayOutput(false, "15s thermal watchdog");
    }
  }
}

void sleepAll() {
  for (int i = 0; i < NUM_AXES; i++) {
    AccelStepper* s = axes[i].stepper;
    s->setCurrentPosition(s->currentPosition());
    axes[i].ready = false;
    axes[i].homed = false;   // unpowered axes can be back-driven / may have lost steps
  }
  digitalWrite(PIN_EN, HIGH);
  setRelayOutput(false, "motors asleep/fault");
  Serial.println("-> All steppers asleep (disabled). Run 'setup' to wake them.");
}

// ============================================================
//  PER-AXIS SETTINGS
// ============================================================
void setCurrent(Axis& axis, int mA) {
  mA = constrain(mA, 50, 2000);
  axis.currentMa = mA;
  axis.driver->rms_current(mA, axisHoldMultiplier(axis));
  Serial.print("-> "); Serial.print(axis.name);
  Serial.print(" current limit: "); Serial.print(mA); Serial.println(" mA");
}

void setMicrosteps(Axis& axis, int microsteps) {
  bool valid = microsteps >= 1 && microsteps <= 256 && (microsteps & (microsteps - 1)) == 0;
  if (!valid) {
    Serial.println("[ERROR] Microsteps must be 1, 2, 4, 8, 16, 32, 64, 128 or 256.");
    return;
  }
  axis.microsteps = microsteps;
  axis.driver->microsteps(microsteps);
  axis.stepsPerMm = (MOTOR_FULL_STEPS_REV * (float)microsteps) / axis.mmPerRev;
  Serial.print("-> "); Serial.print(axis.name);
  Serial.print(" microstepping: "); Serial.print(microsteps);
  Serial.print(" (steps/mm: "); Serial.print(axis.stepsPerMm, 1);
  Serial.println("). NOTE: speeds/StallGuard thresholds are in microsteps and were calibrated at 2x.");
}

void setSensitivity(Axis& axis, int sensitivity) {
  axis.moveSensitivity = constrain(sensitivity, 0, 255);
  axis.driver->SGTHRS(axis.moveSensitivity);
  Serial.print("-> "); Serial.print(axis.name);
  Serial.print(" StallGuard sensitivity: "); Serial.println(axis.moveSensitivity);
}

void setMaxSpeed(Axis& axis, float speed) {
  axis.maxSpeed = constrain(speed, 10.0f, 20000.0f);
  axis.stepper->setMaxSpeed(axis.maxSpeed);
  Serial.print("-> "); Serial.print(axis.name);
  Serial.print(" max speed: "); Serial.print(axis.maxSpeed); Serial.println(" steps/s");
}

void setAcceleration(Axis& axis, float accel) {
  axis.acceleration = constrain(accel, 10.0f, 100000.0f);
  axis.stepper->setAcceleration(axis.acceleration);
  Serial.print("-> "); Serial.print(axis.name);
  Serial.print(" acceleration: "); Serial.print(axis.acceleration); Serial.println(" steps/s^2");
}

void readStallGuard(Axis& axis) {
  Serial.print("-> "); Serial.print(axis.name);
  Serial.print(" SG_RESULT: "); Serial.print(axis.driver->SG_RESULT());
  Serial.print("  TSTEP: "); Serial.println(axis.driver->TSTEP());
}

// ============================================================
//  MOTION & HOMING
// ============================================================
bool moveAxis(Axis& axis, long steps) {
  if (!axis.ready) {
    Serial.print("[ERROR] "); Serial.print(axis.name);
    Serial.println(" not ready. Run 'setup' first.");
    return false;
  }

  Serial.print("-> "); Serial.print(axis.name);
  Serial.print(" moving "); Serial.print(steps); Serial.println(" steps...");

  AccelStepper* stepper = axis.stepper;
  stepper->move(steps);
  resetStallMonitor(axis);

  unsigned long lastWdtYieldMs = millis();
  while (stepper->distanceToGo() != 0) {
    yieldWdt(lastWdtYieldMs);
    if (checkHardwareEstop()) {
      stepper->setCurrentPosition(stepper->currentPosition());
      sleepAll();
      setRelayOutput(false);
      Serial.println("[ALERT] Hardware E-STOP triggered. Steppers disabled.");
      return false;
    }
    stepper->run();

    if (isStalled(axis)) {
      stepper->setCurrentPosition(stepper->currentPosition());
      sleepAll(); // Cut power immediately!
      Serial.print("[ALERT] "); Serial.print(axis.name);
      Serial.print(" stall detected (SG_RESULT="); Serial.print(axis.driver->SG_RESULT());
      Serial.println("). Steppers disabled (asleep). Position may be inaccurate.");
      return false;
    }
  }

  Serial.print("-> "); Serial.print(axis.name); Serial.println(" move complete.");
  return true;
}

void resetStallMonitor(Axis& axis) {
  axis.cruiseSinceMs = 0;
  axis.diagFirstHighMs = 0;
}

// Crash detection during MOVE / manual jogs. The old version (a) was disabled outright because
// moveSensitivity was 0, and (b) trusted DIAG from 25% of max speed (STALL_CHECK_SPEED_RATIO was declared
// but never used), where back-EMF is too weak and StallGuard false-trips — the reason it had been disabled.
bool isStalled(Axis& axis) {
  if (axis.moveSensitivity <= 0) return false;

  unsigned long nowMs = millis();
  bool cruising = fabs(axis.stepper->speed()) >= axis.maxSpeed * STALL_CHECK_SPEED_RATIO &&
                  axis.maxSpeed >= axis.homeSpeed * STALL_MIN_SPEED_RATIO_OF_HOME;
  if (!cruising) {
    resetStallMonitor(axis);
    return false;
  }
  if (axis.cruiseSinceMs == 0) {
    axis.cruiseSinceMs = (nowMs == 0) ? 1UL : nowMs; // never store 0 (sentinel), avoid |1UL underflow on even ms
    return false;
  }
  if (nowMs - axis.cruiseSinceMs < axis.stallSettleMs) return false;

  if (digitalRead(axis.diagPin) != HIGH) return false;

  // Require a second DIAG HIGH sample >= 1 ms after the first one, within STALL_CONFIRM_WINDOW_MS
  if (axis.diagFirstHighMs == 0 || nowMs - axis.diagFirstHighMs > STALL_CONFIRM_WINDOW_MS) {
    axis.diagFirstHighMs = (nowMs == 0) ? 1UL : nowMs;
    return false;
  }
  return (nowMs - axis.diagFirstHighMs) >= 1;
}

// Polls the hardware E-STOP switch and host link inside blocking loops.
// Hardware E-STOP or ESTOP/X/SLEEP serial input latches estopLatched so callers power drivers down.
bool pollCancelInput() {
  if (checkHardwareEstop()) return true;
  if (Serial.available() <= 0) return false;
  String in = Serial.readStringUntil('\n');
  in.trim();
  in.toUpperCase();
  if (in == "ESTOP" || in == "X" || in == "SLEEP") estopLatched = true;
  return true;
}

// Worst-case time to reach the stop. The fixed 30 s timeout was shorter than a full X stroke at the homing
// speed (236 mm / 4 mm/s = 59 s), so homing X from the far half of the axis always failed.
unsigned long homingTimeoutMs(Axis& axis, bool knownDistance) {
  float travelSteps = knownDistance
      ? fabs((float)axis.stepper->currentPosition()) + 10.0f * axis.stepsPerMm
      : (axis.maxTravelMm + axis.homeBackoffMm) * axis.stepsPerMm;
  unsigned long ms = (unsigned long)(1500.0f * travelSteps / axis.homeSpeed) + HOMING_RAMP_MS + HOMING_BLANKING_MS;
  unsigned long minMs = knownDistance ? 3000UL : HOMING_TIMEOUT_MS;
  return ms > minMs ? ms : minMs;
}

// Single-axis move used for the pre-home pull-away. Does NOT sleep the drivers on a stall.
MotionResult runAxisTo(Axis& axis, long target) {
  AccelStepper* s = axis.stepper;
  s->moveTo(target);
  resetStallMonitor(axis);
  unsigned long lastWdtYieldMs = millis();
  while (s->distanceToGo() != 0) {
    yieldWdt(lastWdtYieldMs);
    s->run();
    if (isStalled(axis)) {
      s->setCurrentPosition(s->currentPosition());
      return MOTION_STALLED;
    }
    if (pollCancelInput()) {
      s->setCurrentPosition(s->currentPosition());
      return MOTION_CANCELLED;
    }
  }
  return MOTION_DONE;
}

static uint8_t tmc2209Crc(const uint8_t* datagram, int len) {
  uint8_t crc = 0;
  for (int i = 0; i < len; i++) {
    uint8_t cur = datagram[i];
    for (int j = 0; j < 8; j++) {
      if ((crc >> 7) ^ (cur & 0x01)) crc = (uint8_t)((crc << 1) ^ 0x07);
      else crc = (uint8_t)(crc << 1);
      cur >>= 1;
    }
  }
  return crc;
}

bool homeAxis(Axis& axis, bool reverse, bool withBackoff) {
  if (!axis.ready) {
    Serial.print("[ERROR] "); Serial.print(axis.name);
    Serial.println(" not ready. Run 'setup' first.");
    return false;
  }

  AccelStepper* stepper = axis.stepper;
  bool towardOrigin = (reverse == axis.homeReverse);

  // --- Safe Pre-Homing Pull-Away ---
  // Only pull away if unhomed, or if homed but sitting right against the bumper (< backoff distance).
  // If already homed and positioned out on the axis, skip pull-away so it never drives toward the far end!
  bool needsBackoff = !axis.homed ||
                      (towardOrigin && stepper->currentPosition() < lroundf(axis.homeBackoffMm * axis.stepsPerMm));
  if (withBackoff && axis.homeBackoffMm > 0.0f && needsBackoff) {
    long backoffSteps = lroundf(axis.homeBackoffMm * axis.stepsPerMm);
    long awaySign = reverse ? 1 : -1;     // opposite to the homing direction
    long cur = stepper->currentPosition();
    long target = cur + awaySign * backoffSteps;

    Serial.print("-> "); Serial.print(axis.name);
    Serial.print(" pulling away "); Serial.print(axis.homeBackoffMm, 1);
    Serial.println(" mm before homing.");

    float savedMax = axis.maxSpeed;
    int savedMoveSens = axis.moveSensitivity;
    axis.maxSpeed = axis.homeSpeed;
    axis.moveSensitivity = 0;
    axis.driver->SGTHRS(0);
    stepper->setMaxSpeed(axis.maxSpeed);
    MotionResult r = runAxisTo(axis, target);
    axis.maxSpeed = savedMax;
    axis.moveSensitivity = savedMoveSens;
    axis.driver->SGTHRS(savedMoveSens);
    stepper->setMaxSpeed(savedMax);

    if (r == MOTION_CANCELLED) {
      if (estopLatched) { sleepAll(); setRelayOutput(false); }
      else { setRelayOutput(false, "homing cancelled"); }
      Serial.print("-> "); Serial.print(axis.name); Serial.println(" homing cancelled.");
      return false;
    }
    delay(HOME_PRE_SETTLE_MS);
  }

  unsigned long timeoutMs = homingTimeoutMs(axis, false);
  axis.homed = false;

  Serial.print("-> Homing "); Serial.print(axis.name);
  Serial.print(reverse ? " backward" : " forward");
  Serial.print(" at "); Serial.print(axis.homeSpeed);
  Serial.print(" steps/s, sensitivity "); Serial.print(axis.homeSensitivity);
  Serial.print(", timeout "); Serial.print(timeoutMs / 1000.0f, 1); Serial.println(" s");

  axis.driver->SGTHRS(axis.homeSensitivity);
  stepper->setMaxSpeed(axis.homeSpeed);

  // Clear any residual stall / load readings from driver registers
  axis.driver->SG_RESULT();

  // Start gently at 1.6 mm/s (80 steps/s @ 2x, 320 steps/s @ 8x) to eliminate inrush torque spike
  float startSpeed = min(40.0f * (float)axis.microsteps, axis.homeSpeed);
  float dirSign = reverse ? -1.0f : 1.0f;
  stepper->setSpeed(dirSign * startSpeed);

  enum { RUNNING, HOMED, TIMED_OUT, CANCELLED } outcome = RUNNING;
  unsigned long startMs = millis();
  unsigned long lastRampMs = startMs;
  unsigned long lastWdtYieldMs = startMs;
  unsigned long diagFirstHighMs = 0;
  bool diagHighActive = false;
  unsigned long diagHighStartUs = 0;
  // Y belt pulley shows a 2-full-step SG_RESULT square wave (~10..180); track a 60 ms rolling peak via
  // non-blocking UART polling so bumper impact (peak drops <= 104 from 126..144) triggers cleanly with zero step jitter.
  const int Y_SG_WINDOW = 12;
  const unsigned long Y_SG_POLL_MS = 5;
  const unsigned long Y_SG_WARMUP_MS = HOMING_RAMP_MS + 20;
  const uint16_t Y_SG_PEAK_THRESH = 104;
  uint16_t ySgRing[12] = {0};
  int ySgIdx = 0;
  int ySgCount = 0;
  unsigned long lastYSgPollMs = 0;
  bool ySgReqPending = false;
  uint8_t ySgReq[4] = {0x05, 0b01, 0x41, 0}; // S2 slave address 0b01, read SG_RESULT (0x41)
  ySgReq[3] = tmc2209Crc(ySgReq, 3);
  if (axis.coordCode == 'Y') {
    while (TMCSerial.available()) TMCSerial.read();
  }

  while (outcome == RUNNING) {
    yieldWdt(lastWdtYieldMs);
    stepper->runSpeed();

    unsigned long nowMs = millis();
    unsigned long elapsed = nowMs - startMs;

    // Smooth linear ramp over HOMING_RAMP_MS (250 ms) directly to full homing speed
    if (elapsed <= HOMING_RAMP_MS && (nowMs - lastRampMs) >= 5) {
      lastRampMs = nowMs;
      float frac = (float)elapsed / (float)HOMING_RAMP_MS;
      float curSpd = startSpeed + frac * (axis.homeSpeed - startSpeed);
      stepper->setSpeed(dirSign * curSpd);
    } else if (elapsed > HOMING_RAMP_MS && fabs(stepper->speed()) != axis.homeSpeed) {
      stepper->setSpeed(dirSign * axis.homeSpeed);
    }

    // Non-blocking UART SG_RESULT sampling for Y (60 ms rolling peak drop catches belt stall without step jitter)
    if (axis.coordCode == 'Y' && axis.homeSensitivity > 0 && elapsed >= Y_SG_WARMUP_MS && (nowMs - lastYSgPollMs >= Y_SG_POLL_MS)) {
      lastYSgPollMs = nowMs;
      if (ySgReqPending && TMCSerial.available() >= 12) {
        uint8_t buf[16];
        int n = 0;
        while (TMCSerial.available() && n < 16) buf[n++] = (uint8_t)TMCSerial.read();
        if (n >= 12 && buf[4] == 0x05 && buf[5] == 0xFF && buf[6] == 0x41 && buf[11] == tmc2209Crc(&buf[4], 7)) {
          uint16_t sg = ((uint16_t)(buf[9] & 0x03) << 8) | buf[10];
          ySgRing[ySgIdx] = sg;
          ySgIdx = (ySgIdx + 1) % Y_SG_WINDOW;
          if (ySgCount < Y_SG_WINDOW) ySgCount++;
          if (elapsed > HOMING_BLANKING_MS && ySgCount >= Y_SG_WINDOW) {
            uint16_t sgPeak = 0;
            for (int k = 0; k < Y_SG_WINDOW; k++) {
              if (ySgRing[k] > sgPeak) sgPeak = ySgRing[k];
            }
            if (sgPeak <= Y_SG_PEAK_THRESH) {
              outcome = HOMED;
            }
          }
        }
      }
      if (outcome == RUNNING) {
        while (TMCSerial.available()) TMCSerial.read();
        TMCSerial.write(ySgReq, 4);
        ySgReqPending = true;
      }
    }

    // Check DIAG after blanking window (350 ms).
    // X/Z lead screws emit brief (~2.8 ms) DIAG pulses at SGTHRS=60 on bumper impact, so confirm >= 150 us
    // to reject EMI spikes while stopping immediately; keep Y on 12 ms fallback alongside 60 ms SG_RESULT peak.
    if (outcome == RUNNING && elapsed > HOMING_BLANKING_MS) {
      if (axis.coordCode != 'Y') {
        if (digitalRead(axis.diagPin) == HIGH) {
          unsigned long nowUs = micros();
          if (!diagHighActive) {
            diagHighActive = true;
            diagHighStartUs = nowUs;
          } else if ((nowUs - diagHighStartUs) >= 150) {
            outcome = HOMED;
          }
        } else {
          diagHighActive = false;
        }
      } else {
        if (digitalRead(axis.diagPin) == HIGH) {
          if (diagFirstHighMs == 0) {
            diagFirstHighMs = (nowMs == 0) ? 1UL : nowMs;
          } else if ((nowMs - diagFirstHighMs) >= 12) {
            outcome = HOMED;
          }
        } else {
          if (diagFirstHighMs != 0 && (nowMs - diagFirstHighMs > 25)) {
            diagFirstHighMs = 0; // noise glitch cleared
          }
        }
      }
    }

    if (elapsed > timeoutMs) {
      outcome = TIMED_OUT;
    } else if (pollCancelInput()) {
      outcome = CANCELLED;
    }
  }

  if (axis.coordCode == 'Y') {
    delay(3);
    while (TMCSerial.available()) TMCSerial.read();
  }
  uint16_t stallSg = axis.driver->SG_RESULT();
  uint32_t stallTstep = axis.driver->TSTEP();

  axis.driver->SGTHRS(axis.moveSensitivity);
  stepper->setMaxSpeed(axis.maxSpeed);

  switch (outcome) {
    case HOMED: {
      // Homing toward the origin stop defines 0. Homing the other way ("home s1" with homeReverse=true) lands
      // on the far stop, which is +maxTravel in the MOVE frame — it used to be set to 0 as well, after which
      // every MOVE drove further into the far stop.
      long homePos = towardOrigin ? 0 : (long)(dirSign * lroundf(axis.maxTravelMm * axis.stepsPerMm));
      if (!axis.homeReverse) homePos = -homePos;
      stepper->setCurrentPosition(homePos);
      axis.homed = true;
      Serial.print("-> "); Serial.print(axis.name);
      Serial.print(" homed (SG_RESULT="); Serial.print(stallSg);
      Serial.print(", TSTEP="); Serial.print(stallTstep);
      Serial.print("). Position set to "); Serial.print(homePos);
      Serial.println(towardOrigin ? "." : " (far end).");
      return true;
    }
    case TIMED_OUT:
      stepper->setCurrentPosition(stepper->currentPosition());
      setRelayOutput(false, "homing timeout");
      Serial.print("[ERROR] "); Serial.print(axis.name);
      Serial.println(" homing timed out with no stall detected.");
      return false;
    default:
      stepper->setCurrentPosition(stepper->currentPosition());
      if (estopLatched) { sleepAll(); setRelayOutput(false); }
      else { setRelayOutput(false, "homing cancelled"); }
      Serial.print("-> "); Serial.print(axis.name);
      Serial.println(" homing cancelled.");
      return false;
  }
}

// ============================================================
//  STEPPER CHIMES (TADA & WIN95 via high-torque SpreadCycle)
// ============================================================
static const int CHIME_MICROSTEPS_XZ = 4;   // Coarser than 16x for stronger acoustic torque ripple
static const int CHIME_MICROSTEPS_Y  = 8;   // 40 mm/rev belt keeps excursion small at 8x
static const int CHIME_CURRENT_MA_XZ = 500; // Moderate chime boost (avoids rail droop / UVLO)
static const int CHIME_CURRENT_MA_Y  = 700; // Keeps Y in vsense=1 matching normal operation

static int chimeMicrostepsFor(const Axis& axis) {
  return (axis.coordCode == 'Y') ? CHIME_MICROSTEPS_Y : CHIME_MICROSTEPS_XZ;
}

static int chimeCurrentMaFor(const Axis& axis) {
  return (axis.coordCode == 'Y') ? CHIME_CURRENT_MA_Y : CHIME_CURRENT_MA_XZ;
}

static void setChimeDriverMode(Axis& axis, bool chimeActive) {
  if (chimeActive) {
    axis.driver->SGTHRS(0);
    axis.driver->en_spreadCycle(true);   // Crisp hysteresis current chopping for audible tones
    axis.driver->intpol(false);          // Disable 256x interpolation so step edges ring cleanly
    axis.driver->microsteps(chimeMicrostepsFor(axis));
    axis.driver->rms_current(chimeCurrentMaFor(axis), 1.0f); // Boost current + keep full hold across rests
  } else {
    restoreAxisFromChime(axis);
  }
}

static bool pollChimeCancelOrQueue() {
  if (checkHardwareEstop()) return true;
  if (Serial.available() <= 0) return false;
  String in = Serial.readStringUntil('\n');
  in.trim();
  if (in.length() == 0) return false;
  String upper = in;
  upper.toUpperCase();
  if (upper == "ESTOP" || upper == "X" || upper == "SLEEP") {
    estopLatched = true;
    return true;
  }
  // Preserve any serial command arriving mid-chime so it runs after driver restoration
  pendingSerialCommand = in;
  return true;
}

static int chooseChimeSegmentDir(int awayDir, long netSteps, float freqHz, unsigned long durationMs) {
  if (freqHz <= 1.0f) return awayDir;
  long segSteps = lroundf(freqHz * ((float)durationMs / 1000.0f));
  // Only reverse back toward the origin stop if we have >= segSteps of headroom,
  // guaranteeing netSteps never crosses 0 into a mechanical stop.
  if (awayDir > 0) {
    return (netSteps >= segSteps) ? -1 : 1;
  }
  return (-netSteps >= segSteps) ? 1 : -1;
}

static void pulseChimeStep(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(3);
  digitalWrite(stepPin, LOW);
}

static bool stepChimeSegment(
    const ChimeSegment& seg,
    long& netStepsX,
    long& netStepsY,
    long& netStepsZ,
    bool& cancelled
) {
  digitalWrite(PIN_DIR_1, seg.dirX >= 0 ? HIGH : LOW);
  digitalWrite(PIN_DIR_2, seg.dirY >= 0 ? HIGH : LOW);
  digitalWrite(PIN_DIR_3, seg.dirZ >= 0 ? HIGH : LOW);
  delayMicroseconds(5);

  unsigned long periodXUs = (seg.freqXHz > 1.0f) ? (unsigned long)lroundf(1000000.0f / seg.freqXHz) : 0UL;
  unsigned long periodYUs = (seg.freqYHz > 1.0f) ? (unsigned long)lroundf(1000000.0f / seg.freqYHz) : 0UL;
  unsigned long periodZUs = (seg.freqZHz > 1.0f) ? (unsigned long)lroundf(1000000.0f / seg.freqZHz) : 0UL;

  unsigned long startMs = millis();
  unsigned long nowUs = micros();
  unsigned long nextXUs = nowUs;
  unsigned long nextYUs = nowUs;
  unsigned long nextZUs = nowUs;
  unsigned long lastWdtYieldMs = startMs;

  while (millis() - startMs < seg.durationMs) {
    yieldWdt(lastWdtYieldMs);
    if (pollChimeCancelOrQueue()) {
      cancelled = true;
      return false;
    }

    nowUs = micros();
    if (periodXUs > 0 && (long)(nowUs - nextXUs) >= 0) {
      pulseChimeStep(PIN_STEP_1);
      netStepsX += (seg.dirX >= 0) ? 1L : -1L;
      nextXUs += periodXUs;
    }
    if (periodYUs > 0 && (long)(nowUs - nextYUs) >= 0) {
      pulseChimeStep(PIN_STEP_2);
      netStepsY += (seg.dirY >= 0) ? 1L : -1L;
      nextYUs += periodYUs;
    }
    if (periodZUs > 0 && (long)(nowUs - nextZUs) >= 0) {
      pulseChimeStep(PIN_STEP_3);
      netStepsZ += (seg.dirZ >= 0) ? 1L : -1L;
      nextZUs += periodZUs;
    }
  }
  return true;
}

static bool returnChimeOffsetSilently(long netStepsX, long netStepsY, long netStepsZ) {
  // Drop back to normal run currents while keeping chime microsteps so step counts match,
  // then unwind the musical displacement cleanly back to the exact starting microstep before
  // StealthChop + regular microstepping/interpolation are restored at standstill.
  for (int i = 0; i < NUM_AXES; i++) {
    axes[i].driver->rms_current(axes[i].currentMa, axisHoldMultiplier(axes[i]));
  }
  delay(10);

  long remX = labs(netStepsX);
  long remY = labs(netStepsY);
  long remZ = labs(netStepsZ);
  digitalWrite(PIN_DIR_1, (netStepsX > 0) ? LOW : HIGH);
  digitalWrite(PIN_DIR_2, (netStepsY > 0) ? LOW : HIGH);
  digitalWrite(PIN_DIR_3, (netStepsZ > 0) ? LOW : HIGH);
  delayMicroseconds(5);

  unsigned long lastWdtYieldMs = millis();
  while (remX > 0 || remY > 0 || remZ > 0) {
    yieldWdt(lastWdtYieldMs);
    if (checkHardwareEstop()) {
      return false;
    }
    if (remX > 0) {
      pulseChimeStep(PIN_STEP_1);
      remX--;
    }
    if (remY > 0) {
      pulseChimeStep(PIN_STEP_2);
      remY--;
    }
    if (remZ > 0) {
      pulseChimeStep(PIN_STEP_3);
      remZ--;
    }
    delayMicroseconds(2000);
  }
  return true;
}

void handleChimeGcode(String params, bool emitProtocolReply) {
  params.trim();
  params.toUpperCase();
  if (checkHardwareEstop()) {
    setRelayOutput(false, "hardware estop");
    if (emitProtocolReply) Serial.println("ERR:ESTOP");
    return;
  }
  if (!ensureDriversReady()) {
    if (emitProtocolReply) Serial.println("ERR:TMC_NOT_READY");
    return;
  }

  // Snapshot exact pre-chime hardware registers and software settings/limits on all 3 axes
  SavedAxisSnapshot snapshots[NUM_AXES];
  for (int i = 0; i < NUM_AXES; i++) {
    captureAxisSnapshot(axes[i], snapshots[i]);
    axes[i].driver->GSTAT(0x07);
  }

  float curXMm = (float)axes[0].stepper->currentPosition() / axes[0].stepsPerMm;
  float curYMm = (float)axes[1].stepper->currentPosition() / axes[1].stepsPerMm;
  float curZMm = (float)axes[2].stepper->currentPosition() / axes[2].stepsPerMm;

  // Safe directions pointing away from the nearest mechanical stop
  int awayX = (curXMm > TRAVEL_X_MM - 8.0f) ? -1 : 1;
  int awayY = (curYMm > TRAVEL_Y_MM - 8.0f) ? -1 : 1;
  int awayZ = (curZMm > TRAVEL_Z_MM - 8.0f) ? -1 : 1;

  estopLatched = false;
  for (int i = 0; i < NUM_AXES; i++) {
    setChimeDriverMode(axes[i], true);
  }
  delay(10);

  long netStepsX = 0;
  long netStepsY = 0;
  long netStepsZ = 0;
  bool cancelled = false;

  // Windows 3.1 "Tada!" (3-voice: X + Z upper dyad, Y octave/root bass)
  const ChimeSegment tada[] = {
    { 392.00f, 196.00f,   0.00f, 130UL, 0, 0, 0 }, // "Ta-" (G4 + G3)
    {   0.00f,   0.00f,   0.00f,  25UL, 0, 0, 0 }, // crisp rest
    { 523.25f, 261.63f, 659.25f, 560UL, 0, 0, 0 }, // "-DAAA!" (C5 + C4 + E5)
  };

  // Windows 95 Startup Theme ("The Microsoft Sound", Eb major / Abmaj9 -> Eb)
  const ChimeSegment win95[] = {
    { 622.25f, 311.13f, 466.16f, 200UL, 0, 0, 0 }, // Eb5 + Eb4 + Bb4
    {   0.00f,   0.00f,   0.00f,  20UL, 0, 0, 0 },
    { 466.16f, 311.13f, 392.00f, 175UL, 0, 0, 0 }, // Bb4 + Eb4 + G4
    {   0.00f,   0.00f,   0.00f,  20UL, 0, 0, 0 },
    { 415.30f, 207.65f, 622.25f, 175UL, 0, 0, 0 }, // Ab4 + Ab3 + Eb5
    {   0.00f,   0.00f,   0.00f,  20UL, 0, 0, 0 },
    { 622.25f, 207.65f, 523.25f, 460UL, 0, 0, 0 }, // Eb5 + Ab3 + C5 (Abmaj9)
    {   0.00f,   0.00f,   0.00f,  25UL, 0, 0, 0 },
    { 466.16f, 233.08f, 349.23f, 190UL, 0, 0, 0 }, // Bb4 + Bb3 + F4
    {   0.00f,   0.00f,   0.00f,  20UL, 0, 0, 0 },
    { 698.46f, 233.08f, 466.16f, 320UL, 0, 0, 0 }, // F5 + Bb3 + Bb4 (Bbsus)
    {   0.00f,   0.00f,   0.00f,  20UL, 0, 0, 0 },
    { 622.25f, 311.13f, 392.00f, 525UL, 0, 0, 0 }, // Eb5 + Eb4 + G4 (hold pt 1)
    { 622.25f, 311.13f, 392.00f, 525UL, 0, 0, 0 }, // Eb5 + Eb4 + G4 (hold pt 2)
  };

  const bool isWin95 = (params == "WIN95" || params == "95");
  const ChimeSegment* seq = isWin95 ? win95 : tada;
  const unsigned int count = isWin95
      ? (sizeof(win95) / sizeof(win95[0]))
      : (sizeof(tada) / sizeof(tada[0]));

  for (unsigned int i = 0; i < count; i++) {
    ChimeSegment seg = seq[i];
    seg.dirX = chooseChimeSegmentDir(awayX, netStepsX, seg.freqXHz, seg.durationMs);
    seg.dirY = chooseChimeSegmentDir(awayY, netStepsY, seg.freqYHz, seg.durationMs);
    seg.dirZ = chooseChimeSegmentDir(awayZ, netStepsZ, seg.freqZHz, seg.durationMs);
    if (!stepChimeSegment(seg, netStepsX, netStepsY, netStepsZ, cancelled)) {
      break;
    }
  }

  if (cancelled && estopLatched) {
    sleepAll();
    setRelayOutput(false);
    if (emitProtocolReply) Serial.println("ERR:ESTOP");
    return;
  }

  if (!returnChimeOffsetSilently(netStepsX, netStepsY, netStepsZ)) {
    sleepAll();
    setRelayOutput(false);
    if (emitProtocolReply) Serial.println("ERR:ESTOP");
    return;
  }

  for (int i = 0; i < NUM_AXES; i++) {
    restoreAxisSnapshot(axes[i], snapshots[i]);
  }
  // Allow StealthChop AT#1 to calibrate cleanly at standstill using exact restored settings
  delay(max(DRIVER_ENABLE_SETTLE_MS, 250UL));

  if (emitProtocolReply) {
    Serial.println(cancelled ? "ERR:CANCELLED" : "OK");
  }
}
