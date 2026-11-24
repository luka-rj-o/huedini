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
#include "soc/gpio_struct.h"
#include "esp_cpu.h"

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
const int SETUP_CURRENT_MA_XZ     = 350; // Calibrated 350mA for X and Z lead screws
const int SETUP_CURRENT_MA_Y_MOVE = 480; // Cool StealthChop2 8x DDS current (0.35x hold = ~168 mA standby)
const int SETUP_MICROSTEPS        = 2;   // Z uses 2x
const int SETUP_MICROSTEPS_X      = 8;   // X uses 8x
const int SETUP_MICROSTEPS_Y      = 8;   // Y uses 8x StealthChop2 + 256x intpol + IRAM S-curve DDS (40.0 steps/mm)
// Crash (stall) detection SGTHRS used during MOVE / manual jogs.
// Tuned per-axis at cruise speed + settle time + debounce (see isStalled()) so fast S-curve
// moves maintain strong StallGuard headroom while catching real collisions.
// Set to 0 with "s1 s0" (etc.) to disable crash detection on an axis.
const int SETUP_STALL_SENS_X  = 50;  // Triggers at SG_RESULT <= 100 (X cruise avg ~290)
const int SETUP_STALL_SENS_Y  = 0;   // Disabled during Y moves (prioritizes drift-free travel)
const int SETUP_STALL_SENS_Z  = 40;  // Triggers at SG_RESULT <= 80 (Z UP cruise min ~228 at 35 mm/s)

// Per-axis defaults (speeds in microsteps/s, peak S-curve acceleration in microsteps/s^2)
const float DEFAULT_MAX_SPEED_X   = 12000.0f; // 60.0 mm/s on 8mm-lead screw @ 8x (200 steps/mm)
const float DEFAULT_ACCEL_X       = 40000.0f; // 200.0 mm/s^2 peak S-curve accel on 8mm-lead screw @ 8x
const float DEFAULT_HOME_SPEED_X  = 3200.0f;  // 16.0 mm/s @ 8x (same physical speed & StallGuard4 back-EMF)

const float DEFAULT_MAX_SPEED_Z   = 1750.0f;  // 35.0 mm/s on 8mm-lead screw @ 2x (50 steps/mm)
const float DEFAULT_ACCEL_Z       = 6000.0f;  // 120.0 mm/s^2 peak S-curve accel on 8mm-lead screw @ 2x
const float DEFAULT_HOME_SPEED_XZ = 800.0f;   // 16.0 mm/s @ 2x (calibrated for healthy back-EMF)

const float DEFAULT_MAX_SPEED_Y   = 4000.0f;  // 100.0 mm/s on 40mm/rev GT2 20T belt @ 8x (40.0 steps/mm)
const float DEFAULT_ACCEL_Y       = 9600.0f;  // 240.0 mm/s^2 peak S-curve accel @ 8x (verified 0.00 mm drift)
const float DEFAULT_HOME_SPEED_Y  = 2000.0f;  // 50.0 mm/s (1.25 rev/s) @ 8x in native StealthChop2

const int   DEFAULT_HOME_SENS     = 60;      // Homing sensitivity threshold for X and Z lead screws (SGTHRS=60)
const int   DEFAULT_HOME_SENS_Y   = 35;      // Calibrated Y StallGuard4 threshold (stall ~56-60, cruise ~82-142, 0.00 px repeat)

// X: Lead screw 8.0 mm/rev @ 8x microsteps -> 200.0 microsteps/mm.
// Z: Lead screw 8.0 mm/rev @ 2x microsteps -> 50.0 microsteps/mm.
// Y: GT2 belt (2.0 mm pitch) x 20-tooth pulley = 40.0 mm/rev -> 40.0 microsteps/mm @ 8x move microsteps.
const float LEAD_SCREW_LEAD_MM     = 8.0f;
const float BELT_PULLEY_MM_REV_Y   = 40.0f; // 20 teeth * 2.0 mm GT2 pitch
const float MOTOR_FULL_STEPS_REV   = 200.0f;
const float DEFAULT_STEPS_PER_MM_X = (MOTOR_FULL_STEPS_REV * SETUP_MICROSTEPS_X) / LEAD_SCREW_LEAD_MM;   // 200.0f
const float DEFAULT_STEPS_PER_MM_Y = (MOTOR_FULL_STEPS_REV * SETUP_MICROSTEPS_Y) / BELT_PULLEY_MM_REV_Y; // 40.0f
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
const float HOME_BACKOFF_MM_Y  = 20.0f;
const unsigned long HOME_PRE_SETTLE_MS = 50;

// Per-axis cap applied to MOVE feedrates (microsteps/s) so each axis stays inside its verified
// S-curve + StallGuard operating envelope.
const float MAX_FEED_SPEED_X  = DEFAULT_MAX_SPEED_X;  // 12000 steps/s = 60.0 mm/s @ 8x
const float MAX_FEED_SPEED_Z  = DEFAULT_MAX_SPEED_Z;  // 1750 steps/s = 35.0 mm/s @ 2x
const float MAX_FEED_SPEED_Y  = DEFAULT_MAX_SPEED_Y;  // 4000 steps/s = 100.0 mm/s @ 8x

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
  int             stepPin;
  int             dirPin;
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

// Phase-locked hardware-timer DDS S-curve velocity profile state per axis
struct SCurveAxis {
  Axis*  axis = nullptr;
  long   curPos = 0;
  long   targetPos = 0;
  int    dir = 1;
  float  vPeak = 0.0f;
  float  vFloor = 0.0f;
  float  lastCmdSpeed = -1.0f;
  float  tRampSec = 0.0f;
  float  decelDist = 0.0f;
  bool   active = false;
  bool   managed = false;
};

struct IsrStepAxis {
  volatile bool     active;
  volatile int      stepPin;
  volatile int      dir;
  volatile long     curPos;
  volatile long     targetPos;
  volatile uint32_t ddsPhase;
  volatile uint32_t ddsInc;
};

static const uint32_t STEP_ISR_HZ = 50000UL; // 20 us hardware timer tick -> jitter-free stepping
static portMUX_TYPE stepMux = portMUX_INITIALIZER_UNLOCKED;
static volatile IsrStepAxis isrAxes[3] = {
  { false, -1, 1, 0, 0, 0, 0 },
  { false, -1, 1, 0, 0, 0, 0 },
  { false, -1, 1, 0, 0, 0, 0 },
};
static hw_timer_t* stepTimer = nullptr;

const int NUM_AXES = 3;
Axis axes[NUM_AXES] = {
  { "S1", 'X', PIN_STEP_1, PIN_DIR_1, PIN_DIAG_1, &stepper1, &driver1, false, DEFAULT_MAX_SPEED_X, DEFAULT_ACCEL_X, DEFAULT_HOME_SPEED_X,  SETUP_STALL_SENS_X, DEFAULT_HOME_SENS,   DEFAULT_STEPS_PER_MM_X, LEAD_SCREW_LEAD_MM,   DEFAULT_HOME_DIR_REVERSE_X, INVERT_DIR_X, SETUP_CURRENT_MA_XZ,     TRAVEL_X_MM, HOME_BACKOFF_MM_XZ, MAX_FEED_SPEED_X, STALL_SETTLE_MS_XZ, SETUP_MICROSTEPS_X },
  { "S2", 'Y', PIN_STEP_2, PIN_DIR_2, PIN_DIAG_2, &stepper2, &driver2, false, DEFAULT_MAX_SPEED_Y, DEFAULT_ACCEL_Y, DEFAULT_HOME_SPEED_Y,  SETUP_STALL_SENS_Y, DEFAULT_HOME_SENS_Y, DEFAULT_STEPS_PER_MM_Y, BELT_PULLEY_MM_REV_Y, DEFAULT_HOME_DIR_REVERSE_Y, INVERT_DIR_Y, SETUP_CURRENT_MA_Y_MOVE, TRAVEL_Y_MM, HOME_BACKOFF_MM_Y,  MAX_FEED_SPEED_Y, STALL_SETTLE_MS_Y,  SETUP_MICROSTEPS_Y },
  { "S3", 'Z', PIN_STEP_3, PIN_DIR_3, PIN_DIAG_3, &stepper3, &driver3, false, DEFAULT_MAX_SPEED_Z, DEFAULT_ACCEL_Z, DEFAULT_HOME_SPEED_XZ, SETUP_STALL_SENS_Z, DEFAULT_HOME_SENS,   DEFAULT_STEPS_PER_MM_Z, LEAD_SCREW_LEAD_MM,   DEFAULT_HOME_DIR_REVERSE_Z, INVERT_DIR_Z, SETUP_CURRENT_MA_XZ,     TRAVEL_Z_MM, HOME_BACKOFF_MM_XZ, MAX_FEED_SPEED_Z, STALL_SETTLE_MS_XZ, SETUP_MICROSTEPS },
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
void initSCurveLuts();
void initStepTimer();
void handleMoveGcode(String params);
void handlePressGcode(String params);
void handleHomeAllGcode();
void handleGripGcode(String params);
void handleTareGcode();
void handleWeighGcode();
bool sampleLoadCellFilteredRaw(long& outRaw, float& outSpreadG, int numSamples = 11, int flushCount = 2, bool* outEstop = nullptr);
void handleStatusGcode();
void handleRegsGcode();
void handleStepsGcode(String params);
void handleChimeGcode(String params, bool emitProtocolReply = true);
void handleTuneGcode(String params);
void applyYMoveMode(Axis& axis);
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
  initSCurveLuts();
  initStepTimer();

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

  // Initialize HX711 load cell amplifier with a fast single-sample boot zero
  scale.begin(PIN_HX711_DT, PIN_HX711_SCK);
  scale.set_scale(hx711ScaleFactor);
  if (scale.wait_ready_timeout(120)) {
    scale.set_offset(scale.read());
  }

  for (int i = 0; i < NUM_AXES; i++) {
    pinMode(axes[i].diagPin, INPUT_PULLDOWN);
    axes[i].stepper->setMaxSpeed(axes[i].maxSpeed);
    axes[i].stepper->setAcceleration(axes[i].acceleration);
    isrAxes[i].curPos = 0;
    isrAxes[i].targetPos = 0;
    isrAxes[i].active = false;
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
  if (upper.startsWith("PRESS ") || upper == "PRESS") {
    handlePressGcode(upper.length() > 5 ? upper.substring(6) : "");
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
  if (upper == "TUNE" || upper.startsWith("TUNE ")) {
    handleTuneGcode(upper.length() > 4 ? upper.substring(5) : "");
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
  else if (command == "tune" || command.startsWith("tune "))   { handleTuneGcode(command.length() > 4 ? command.substring(5) : ""); }
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

  // Guarantee every axis has its calibrated microsteps, interpolation, current, and chopper mode before homing
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

static bool pollMoveEstop() {
  if (checkHardwareEstop()) return true;
  if (Serial.available() > 0) {
    String inCmd = Serial.readStringUntil('\n');
    inCmd.trim();
    String up = inCmd;
    up.toUpperCase();
    if (up == "ESTOP" || up == "X" || up == "SLEEP") {
      return true;
    }
    if (inCmd.length() > 0 && pendingSerialCommand.length() == 0) {
      pendingSerialCommand = inCmd;
    }
  }
  return false;
}

static const int SCURVE_LUT_N = 64;
static float sCurveAccelLut[SCURVE_LUT_N + 1];
static float sCurveDecelLut[SCURVE_LUT_N + 1];
static bool sCurveLutsReady = false;

void initSCurveLuts() {
  if (sCurveLutsReady) return;
  for (int i = 0; i <= SCURVE_LUT_N; i++) {
    float x = (float)i / (float)SCURVE_LUT_N;
    sCurveAccelLut[i] = 0.5f * (1.0f - cosf(M_PI * x));
    if (i == 0) {
      sCurveDecelLut[i] = 0.0f;
    } else {
      float tau = constrain(cbrtf((6.0f * x) / (M_PI * M_PI)), 0.0f, 1.0f);
      for (int it = 0; it < 12; it++) {
        float f = tau - sinf(M_PI * tau) / M_PI - x;
        float df = max(1e-6f, 1.0f - cosf(M_PI * tau));
        tau = constrain(tau - f / df, 0.0f, 1.0f);
      }
      sCurveDecelLut[i] = 0.5f * (1.0f - cosf(M_PI * tau));
    }
  }
  sCurveLutsReady = true;
}

static inline float lookupSCurveLut(const float* lut, float x) {
  float clamped = constrain(x, 0.0f, 1.0f) * (float)SCURVE_LUT_N;
  int idx = (int)clamped;
  if (idx >= SCURVE_LUT_N) return lut[SCURVE_LUT_N];
  float frac = clamped - (float)idx;
  return lut[idx] + frac * (lut[idx + 1] - lut[idx]);
}

static inline uint32_t speedToDdsInc(float stepsPerSec) {
  float inc = max(1.0f, stepsPerSec) * (4294967296.0f / (float)STEP_ISR_HZ);
  if (inc >= 4294967295.0f) return 0xFFFFFFFFUL;
  return (uint32_t)(inc + 0.5f);
}

static void IRAM_ATTR onStepTimer() {
  uint32_t stepMask = 0;
  portENTER_CRITICAL_ISR(&stepMux);
  for (int i = 0; i < NUM_AXES; i++) {
    volatile IsrStepAxis& ax = isrAxes[i];
    if (!ax.active) continue;
    long cur = ax.curPos;
    long tgt = ax.targetPos;
    if (cur == tgt) {
      ax.active = false;
      continue;
    }
    uint32_t prev = ax.ddsPhase;
    uint32_t next = prev + ax.ddsInc;
    ax.ddsPhase = next;
    if (next < prev) {
      if (ax.stepPin >= 0 && ax.stepPin < 32) {
        stepMask |= (1UL << ax.stepPin);
      }
      cur += ax.dir;
      ax.curPos = cur;
      if (cur == tgt) {
        ax.active = false;
      }
    }
  }
  portEXIT_CRITICAL_ISR(&stepMux);

  if (stepMask != 0) {
    GPIO.out_w1ts = stepMask;
    uint32_t t0 = esp_cpu_get_cycle_count();
    while ((uint32_t)(esp_cpu_get_cycle_count() - t0) < 480U) {} // exact 2.00 us @ 240 MHz
    GPIO.out_w1tc = stepMask;
  }
}

void initStepTimer() {
  if (stepTimer != nullptr) return;
  stepTimer = timerBegin(1000000UL); // 1 MHz base clock (1 us/tick)
  if (stepTimer != nullptr) {
    timerAttachInterrupt(stepTimer, &onStepTimer);
    timerAlarm(stepTimer, 1000000UL / STEP_ISR_HZ, true, 0); // 20 us = 50 kHz auto-reload
    timerStop(stepTimer);
  }
}

// Keeps AccelStepper and the hardware-timer ISR position mirror identical across
// homing, sleep, zero-delta moves, and S-curve completion.
static void syncAxisPosition(Axis& axis, long pos) {
  int idx = (int)(&axis - axes);
  if (idx >= 0 && idx < NUM_AXES) {
    portENTER_CRITICAL(&stepMux);
    isrAxes[idx].curPos = pos;
    isrAxes[idx].targetPos = pos;
    isrAxes[idx].active = false;
    portEXIT_CRITICAL(&stepMux);
  }
  if (axis.stepper != nullptr) {
    axis.stepper->setCurrentPosition(pos);
  }
}

static void initSCurveAxis(SCurveAxis& sc, Axis& axis, long targetPos) {
  initSCurveLuts();
  long curPos = axis.stepper->currentPosition();
  long delta = targetPos - curPos;
  sc.axis = &axis;
  sc.curPos = curPos;
  sc.targetPos = targetPos;
  resetStallMonitor(axis);

  if (delta == 0) {
    syncAxisPosition(axis, curPos);
    sc.managed = false;
    sc.active = false;
    return;
  }

  long totalSteps = labs(delta);
  sc.dir = (delta > 0) ? 1 : -1;
  float vmax = max(10.0f, axis.maxSpeed);
  float apeak = max(10.0f, axis.acceleration);
  float dRampFull = (PI * vmax * vmax) / (2.0f * apeak);
  sc.vPeak = ((float)totalSteps >= dRampFull)
      ? vmax
      : sqrtf((2.0f * apeak * (float)totalSteps) / PI);
  sc.tRampSec = (PI * sc.vPeak) / (2.0f * apeak);
  sc.decelDist = 0.5f * sc.vPeak * sc.tRampSec;
  sc.vFloor = min(sc.vPeak, max(15.0f, 0.5f * axis.stepsPerMm));
  sc.lastCmdSpeed = sc.vFloor;
  sc.managed = true;
  sc.active = true;

  axis.stepper->setMaxSpeed(vmax);
  axis.stepper->setSpeed((float)sc.dir * sc.vFloor);
  digitalWrite(axis.dirPin, (sc.dir > 0) ? HIGH : LOW);
  delayMicroseconds(2);
}

static void startStepEngine(SCurveAxis* scList, int count) {
  initStepTimer();
  portENTER_CRITICAL(&stepMux);
  for (int i = 0; i < NUM_AXES; i++) {
    isrAxes[i].active = false;
  }
  for (int j = 0; j < count; j++) {
    if (scList[j].axis != nullptr) {
      int idx = (int)(scList[j].axis - axes);
      isrAxes[idx].curPos    = scList[j].curPos;
      isrAxes[idx].targetPos = scList[j].targetPos;
      if (scList[j].active) {
        isrAxes[idx].stepPin  = scList[j].axis->stepPin;
        isrAxes[idx].dir      = scList[j].dir;
        isrAxes[idx].ddsPhase = 0xFFFFFFFFUL; // Trigger step #1 on first tick
        isrAxes[idx].ddsInc   = speedToDdsInc(scList[j].vFloor);
        isrAxes[idx].active   = true;
      } else {
        isrAxes[idx].active   = false;
      }
    }
  }
  portEXIT_CRITICAL(&stepMux);
  if (stepTimer != nullptr) {
    timerRestart(stepTimer);
    timerStart(stepTimer);
  }
}

static void stopStepEngine(SCurveAxis* scList, int count) {
  if (stepTimer != nullptr) {
    timerStop(stepTimer);
  }
  portENTER_CRITICAL(&stepMux);
  for (int i = 0; i < NUM_AXES; i++) {
    isrAxes[i].active = false;
  }
  for (int j = 0; j < count; j++) {
    if (scList[j].axis != nullptr && scList[j].managed) {
      int idx = (int)(scList[j].axis - axes);
      scList[j].curPos = isrAxes[idx].curPos;
      scList[j].active = false;
    }
  }
  portEXIT_CRITICAL(&stepMux);
  for (int j = 0; j < count; j++) {
    if (scList[j].axis != nullptr && scList[j].managed) {
      scList[j].axis->stepper->setCurrentPosition(scList[j].curPos);
      scList[j].managed = false;
    }
  }
}

// Syncs ISR step counts and updates S-curve DDS frequencies; returns true while any axis moves.
static bool tickStepEngine(SCurveAxis* scList, int count, unsigned long moveStartUs) {
  unsigned long nowUs = micros();
  float tSec = (float)((unsigned long)(nowUs - moveStartUs)) * 1e-6f;
  bool anyActive = false;

  for (int j = 0; j < count; j++) {
    SCurveAxis& sc = scList[j];
    if (!sc.active || sc.axis == nullptr) continue;
    int idx = (int)(sc.axis - axes);

    portENTER_CRITICAL(&stepMux);
    sc.curPos = isrAxes[idx].curPos;
    sc.active = isrAxes[idx].active;
    portEXIT_CRITICAL(&stepMux);

    if (!sc.active) {
      syncAxisPosition(*sc.axis, sc.curPos);
      sc.managed = false;
      continue;
    }
    anyActive = true;

    long rem = (sc.dir > 0) ? (sc.targetPos - sc.curPos) : (sc.curPos - sc.targetPos);
    if (rem <= 0) continue;

    float va = (tSec < sc.tRampSec && sc.tRampSec > 1e-6f)
        ? (sc.vFloor + (sc.vPeak - sc.vFloor) * lookupSCurveLut(sCurveAccelLut, tSec / sc.tRampSec))
        : sc.vPeak;

    float vd = sc.vPeak;
    if ((float)rem < sc.decelDist && sc.decelDist > 1e-6f) {
      vd = max(sc.vFloor, sc.vPeak * lookupSCurveLut(sCurveDecelLut, (float)rem / sc.decelDist));
    }

    float vCmd = max(sc.vFloor, min(va, vd));
    if (fabsf(vCmd - sc.lastCmdSpeed) >= 1.0f) {
      sc.lastCmdSpeed = vCmd;
      sc.axis->stepper->setSpeed((float)sc.dir * vCmd);
      uint32_t newInc = speedToDdsInc(vCmd);
      portENTER_CRITICAL(&stepMux);
      isrAxes[idx].ddsInc = newInc;
      portEXIT_CRITICAL(&stepMux);
    }
  }
  return anyActive;
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
      // Per-axis cap keeps X/Y/Z within empirically verified torque and StallGuard headroom.
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

  // Unified phase-locked S-curve DDS motion engine in the -O2 IRAM timer ISR for X, Y, and Z.
  SCurveAxis scAxes[NUM_AXES];
  for (int i = 0; i < NUM_AXES; i++) {
    float targetMm = 0.0f;
    long targetSteps = axes[i].stepper->currentPosition();
    if (parseCoordToken(params, axes[i].coordCode, targetMm)) {
      targetMm = constrain(targetMm, 0.0f, axes[i].maxTravelMm);
      targetSteps = lroundf(targetMm * axes[i].stepsPerMm);
    }
    initSCurveAxis(scAxes[i], axes[i], targetSteps);
  }

  bool anyActive = (scAxes[0].active || scAxes[1].active || scAxes[2].active);
  if (anyActive) {
    unsigned long lastWdtYieldMs = millis();
    unsigned long moveStartUs = micros();
    startStepEngine(scAxes, NUM_AXES);
    while (tickStepEngine(scAxes, NUM_AXES, moveStartUs)) {
      yieldWdt(lastWdtYieldMs);
      if (hasPendingAngle) {
        float curYmm = (float)scAxes[1].curPos / axes[1].stepsPerMm;
        if (curYmm >= SERVO_SAFE_Y_MM) {
          writeServoLogicalAngle(targetAngle);
          hasPendingAngle = false;
        }
      }
      if (pollMoveEstop()) {
        stopStepEngine(scAxes, NUM_AXES);
        sleepAll();
        setRelayOutput(false);
        Serial.println("ERR:ESTOP");
        return;
      }
      for (int i = 0; i < NUM_AXES; i++) {
        if (scAxes[i].active && isStalled(axes[i])) {
          stopStepEngine(scAxes, NUM_AXES);
          // Immediately cut power to steppers so motor does not sustain power against the stop!
          sleepAll();
          Serial.print("ERR:STALL_");
          Serial.println(axes[i].coordCode);
          return;
        }
      }
      delayMicroseconds(250);
    }
    stopStepEngine(scAxes, NUM_AXES);
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

// Flushes stale/in-motion HX711 samples, polls N fresh samples with E-stop & WDT checks,
// and computes a trimmed mean (outlier rejection) + spread in grams.
bool sampleLoadCellFilteredRaw(long& outRaw, float& outSpreadG, int numSamples, int flushCount, bool* outEstop) {
  if (outEstop) *outEstop = false;
  numSamples = constrain(numSamples, 1, 15);
  flushCount = constrain(flushCount, 0, 4);
  unsigned long lastWdtYieldMs = millis();

  // Flush stale/in-flight ADC conversions taken during deceleration or servo ringing
  for (int f = 0; f < flushCount; f++) {
    unsigned long waitStart = millis();
    while (!scale.is_ready()) {
      yieldWdt(lastWdtYieldMs);
      if (pollMoveEstop()) {
        if (outEstop) *outEstop = true;
        return false;
      }
      if (millis() - waitStart > 250UL) {
        return false;
      }
      delay(2);
    }
    (void)scale.read();
  }

  long vals[15];
  for (int i = 0; i < numSamples; i++) {
    unsigned long waitStart = millis();
    while (!scale.is_ready()) {
      yieldWdt(lastWdtYieldMs);
      if (pollMoveEstop()) {
        if (outEstop) *outEstop = true;
        return false;
      }
      if (millis() - waitStart > 250UL) {
        return false;
      }
      delay(2);
    }
    vals[i] = scale.read();
  }

  // Insertion sort for trimmed-mean outlier rejection
  for (int i = 1; i < numSamples; i++) {
    long key = vals[i];
    int j = i - 1;
    while (j >= 0 && vals[j] > key) {
      vals[j + 1] = vals[j];
      j--;
    }
    vals[j + 1] = key;
  }

  int trim = (numSamples >= 5) ? (numSamples / 4) : 0;
  long long sum = 0;
  int count = 0;
  for (int i = trim; i < numSamples - trim; i++) {
    sum += (long long)vals[i];
    count++;
  }
  outRaw = (count > 0) ? (long)(sum / count) : vals[numSamples / 2];

  if (fabsf(hx711ScaleFactor) > 1e-3f) {
    long lo = vals[trim];
    long hi = vals[numSamples - 1 - trim];
    outSpreadG = fabsf((float)(hi - lo) / hx711ScaleFactor);
  } else {
    outSpreadG = 0.0f;
  }
  return true;
}

// Bounded downward floor/surface press using HX711 compressive force feedback capped at Z<=maxZ:
//   PRESS Z<max_z_mm> D<seat_force_g> F<mm_s> [G<guard_force_g> S<guard_limit_z_mm>]
void handlePressGcode(String params) {
  if (checkHardwareEstop()) {
    setRelayOutput(false, "hardware estop");
    Serial.println("ERR:ESTOP");
    return;
  }
  if (!ensureDriversReady()) {
    Serial.println("ERR:TMC_NOT_READY");
    return;
  }

  Axis& zAx = axes[2];
  float curZmm = (float)zAx.stepper->currentPosition() / zAx.stepsPerMm;
  float maxZmm = curZmm;
  parseCoordToken(params, 'Z', maxZmm);
  maxZmm = constrain(maxZmm, curZmm, zAx.maxTravelMm);

  float targetDeltaG = 0.0f;
  parseCoordToken(params, 'D', targetDeltaG);
  targetDeltaG = max(0.0f, targetDeltaG);

  float pressSpeedMmS = 8.0f;
  parseCoordToken(params, 'F', pressSpeedMmS);
  pressSpeedMmS = constrain(pressSpeedMmS, 1.0f, 35.0f);

  float guardDeltaG = targetDeltaG;
  float guardLimitZmm = curZmm;
  bool hasGuard = parseCoordToken(params, 'G', guardDeltaG) && parseCoordToken(params, 'S', guardLimitZmm);
  if (hasGuard) {
    guardDeltaG = max(0.0f, guardDeltaG);
    guardLimitZmm = constrain(guardLimitZmm, curZmm, maxZmm);
  }

  unsigned long lastWdtYieldMs = millis();

  // Brief settle after prior move, flush stale ADC reading, and sample fresh HX711 baseline
  long baseRaw = 0;
  float baseSpread = 0.0f;
  bool hasBaseline = false;
  float lastDeltaG = 0.0f;
  if ((targetDeltaG > 0.5f || (hasGuard && guardDeltaG > 0.5f)) && fabsf(hx711ScaleFactor) > 1e-3f) {
    for (int w = 0; w < 4; w++) {
      yieldWdt(lastWdtYieldMs);
      if (pollMoveEstop()) {
        sleepAll();
        setRelayOutput(false);
        Serial.println("ERR:ESTOP");
        return;
      }
      delay(10);
    }
    bool estopAborted = false;
    if (sampleLoadCellFilteredRaw(baseRaw, baseSpread, 1, 1, &estopAborted)) {
      hasBaseline = true;
    } else if (estopAborted) {
      sleepAll();
      setRelayOutput(false);
      Serial.println("ERR:ESTOP");
      return;
    }
  }

  float savedMaxSpeed = zAx.maxSpeed;
  zAx.maxSpeed = constrain(pressSpeedMmS * zAx.stepsPerMm, 10.0f, zAx.maxFeedSpeed);
  SCurveAxis scZ;
  initSCurveAxis(scZ, zAx, lroundf(maxZmm * zAx.stepsPerMm));

  unsigned long moveStartUs = micros();
  bool forceReached = false;
  bool guardTripped = false;
  bool slowedForContact = false;
  startStepEngine(&scZ, 1);
  while (tickStepEngine(&scZ, 1, moveStartUs)) {
    yieldWdt(lastWdtYieldMs);
    if (pollMoveEstop()) {
      stopStepEngine(&scZ, 1);
      zAx.maxSpeed = savedMaxSpeed;
      zAx.stepper->setMaxSpeed(savedMaxSpeed);
      sleepAll();
      setRelayOutput(false);
      Serial.println("ERR:ESTOP");
      return;
    }

    // Two-stage guarded descent: cruise fast in open air, then decelerate to gentle 8.0 mm/s
    // for the final ~5 mm contact window so 10 Hz HX711 latency never slams the load cell beam.
    if (!forceReached && !slowedForContact && hasGuard && pressSpeedMmS > 9.0f) {
      float curLiveZmm = (float)scZ.curPos / zAx.stepsPerMm;
      float slowStartZmm = max(curZmm, guardLimitZmm - 1.5f);
      if (curLiveZmm >= slowStartZmm) {
        slowedForContact = true;
        scZ.vPeak = min(scZ.vPeak, max(40.0f, 8.0f * zAx.stepsPerMm));
        scZ.decelDist = min(scZ.decelDist, ((float)PI * scZ.vPeak * scZ.vPeak) / (4.0f * max(10.0f, zAx.acceleration)));
      }
    }

    if (hasBaseline && !forceReached && (unsigned long)(micros() - moveStartUs) >= 40000UL && scale.is_ready()) {
      long raw = scale.read();
      long liveCur = 0;
      portENTER_CRITICAL(&stepMux);
      liveCur = isrAxes[2].curPos;
      portEXIT_CRITICAL(&stepMux);
      float liveZmm = (float)liveCur / zAx.stepsPerMm;

      float guardZoneStartZmm = hasGuard ? max(curZmm, guardLimitZmm - 22.0f) : curZmm;
      float rawDeltaG = (float)(raw - baseRaw) / hx711ScaleFactor;

      // In high open air well above the guard band, track steady downward Z lead-screw friction baseline
      if (hasGuard && liveZmm < (guardZoneStartZmm - 2.0f) && fabsf(rawDeltaG) <= 65.0f) {
        baseRaw = raw;
        lastDeltaG = 0.0f;
      } else {
        // Hanging load reads +grams; upward contact/floor press reads negative deltaG
        lastDeltaG = rawDeltaG;

        bool inGuardZone = hasGuard && (liveZmm >= guardZoneStartZmm) && (liveZmm < guardLimitZmm);
        bool aboveGuardZone = hasGuard && (liveZmm < guardZoneStartZmm);
        float activeThreshG = aboveGuardZone
            ? max(targetDeltaG, 180.0f)
            : (inGuardZone ? guardDeltaG : targetDeltaG);

        if (activeThreshG > 0.5f && lastDeltaG <= -activeThreshG) {
          forceReached = true;
          guardTripped = hasGuard && (liveZmm < guardLimitZmm);
          portENTER_CRITICAL(&stepMux);
          liveCur = isrAxes[2].curPos;
          long rem = scZ.targetPos - liveCur;
          long stopSteps = (rem > 0) ? min(rem, 3L) : 0L;
          long newTarget = liveCur + stopSteps;
          isrAxes[2].targetPos = newTarget;
          if (stopSteps == 0L) isrAxes[2].active = false;
          scZ.curPos = liveCur;
          scZ.targetPos = newTarget;
          scZ.decelDist = max(1.0f, (float)stopSteps);
          scZ.active = (stopSteps > 0L);
          portEXIT_CRITICAL(&stepMux);
        }
      }
    }

    if (scZ.active && isStalled(zAx)) {
      stopStepEngine(&scZ, 1);
      zAx.maxSpeed = savedMaxSpeed;
      zAx.stepper->setMaxSpeed(savedMaxSpeed);
      sleepAll();
      Serial.println("ERR:STALL_Z");
      return;
    }
    delayMicroseconds(250);
  }
  stopStepEngine(&scZ, 1);

  zAx.maxSpeed = savedMaxSpeed;
  zAx.stepper->setMaxSpeed(savedMaxSpeed);

  float actualZmm = (float)zAx.stepper->currentPosition() / zAx.stepsPerMm;
  char replyBuf[80];
  snprintf(replyBuf, sizeof(replyBuf), "PRESSED Z:%.2f DF:%.2f HIT:%d GUARD:%d",
           actualZmm, lastDeltaG, forceReached ? 1 : 0, guardTripped ? 1 : 0);
  Serial.println(replyBuf);
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
  if (checkHardwareEstop()) {
    sleepAll();
    setRelayOutput(false, "hardware estop");
    Serial.println("ERR:ESTOP");
    return;
  }
  long baseRaw = 0;
  float spreadG = 0.0f;
  bool estopAborted = false;
  if (!sampleLoadCellFilteredRaw(baseRaw, spreadG, 5, 1, &estopAborted)) {
    if (estopAborted) {
      sleepAll();
      setRelayOutput(false);
      Serial.println("ERR:ESTOP");
    } else {
      Serial.println("ERR:HX711_NOT_READY");
    }
    return;
  }

  // If spread exceeds 1.5 g due to transient vibration, settle 150 ms and re-sample once
  if (spreadG > 1.5f) {
    unsigned long lastWdtYieldMs = millis();
    for (int w = 0; w < 15; w++) {
      yieldWdt(lastWdtYieldMs);
      if (pollMoveEstop()) {
        sleepAll();
        setRelayOutput(false);
        Serial.println("ERR:ESTOP");
        return;
      }
      delay(10);
    }
    if (!sampleLoadCellFilteredRaw(baseRaw, spreadG, 5, 1, &estopAborted)) {
      if (estopAborted) {
        sleepAll();
        setRelayOutput(false);
        Serial.println("ERR:ESTOP");
      } else {
        Serial.println("ERR:HX711_NOT_READY");
      }
      return;
    }
  }

  scale.set_offset(baseRaw);
  float checkG = 0.0f;

  Serial.print("TARED OFFSET:");
  Serial.print(baseRaw);
  Serial.print(" CHECK:");
  Serial.print(checkG, 2);
  Serial.print(" SPREAD:");
  Serial.println(spreadG, 2);
  Serial.println("OK");
}

void handleWeighGcode() {
  if (checkHardwareEstop()) {
    sleepAll();
    setRelayOutput(false, "hardware estop");
    Serial.println("ERR:ESTOP");
    return;
  }
  long raw = 0;
  float spreadG = 0.0f;
  bool estopAborted = false;
  if (!sampleLoadCellFilteredRaw(raw, spreadG, 5, 1, &estopAborted)) {
    if (estopAborted) {
      sleepAll();
      setRelayOutput(false);
      Serial.println("ERR:ESTOP");
    } else {
      Serial.println("ERR:HX711_NOT_READY");
    }
    return;
  }
  float massGrams = (fabsf(hx711ScaleFactor) > 1e-3f)
      ? (float)(raw - scale.get_offset()) / hx711ScaleFactor
      : 0.0f;
  Serial.print("WEIGH_RAW:");
  Serial.print(raw);
  Serial.print(" OFFSET:");
  Serial.print(scale.get_offset());
  Serial.print(" SPREAD:");
  Serial.println(spreadG, 2);
  Serial.print("WEIGHT:");
  Serial.println(massGrams, 2);
  Serial.println("OK");
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
    uint32_t drv = ax.driver->DRV_STATUS();
    Serial.print("REGS "); Serial.print(ax.name);
    Serial.print(" GCONF=0x"); Serial.print(ax.driver->GCONF(), HEX);
    Serial.print(" GSTAT=0x"); Serial.print(ax.driver->GSTAT(), HEX);
    Serial.print(" CHOP=0x"); Serial.print(ax.driver->CHOPCONF(), HEX);
    Serial.print(" PWM=0x"); Serial.print(ax.driver->PWMCONF(), HEX);
    Serial.print(" DRV=0x"); Serial.print(drv, HEX);
    Serial.print(" OTPW="); Serial.print(drv & 0x01);
    Serial.print(" OT="); Serial.print((drv >> 1) & 0x01);
    Serial.print(" FLG=0x"); Serial.print((uint32_t)(drv & 0x0FFF), HEX);
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
  Serial.println("  SETUP | HOME | MOVE X<mm> Y<mm> Z<mm> A<deg> F<mm_s> | PRESS Z<mm> D<g> F<mm_s>");
  Serial.println("  GRIP <0|1> | TARE | WEIGH | STATUS | STEPS X<f> Y<f> Z<f> | CHIME [TADA|WIN95] | TUNE <X|Y|Z|ALL> <Hz> [ms] [ms_div] [mA] | ESTOP");
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
  // Y runs StealthChop2 at 480 mA (0.35x = ~168 mA standby, IHOLD=5) matching X and Z standby heat.
  return (axis.coordCode == 'Y') ? 0.35f : 0.50f;
}

void applyYMoveMode(Axis& axis) {
  axis.microsteps      = SETUP_MICROSTEPS_Y;
  axis.stepsPerMm      = DEFAULT_STEPS_PER_MM_Y;
  axis.currentMa       = SETUP_CURRENT_MA_Y_MOVE;
  axis.moveSensitivity = SETUP_STALL_SENS_Y;
  axis.maxSpeed        = DEFAULT_MAX_SPEED_Y;
  axis.maxFeedSpeed    = MAX_FEED_SPEED_Y;
  axis.acceleration    = DEFAULT_ACCEL_Y;
  axis.stepper->setMinPulseWidth(2);
  axis.stepper->setMaxSpeed(axis.maxSpeed);
  axis.stepper->setAcceleration(axis.acceleration);

  axis.driver->en_spreadCycle(false);
  axis.driver->toff(5);
  axis.driver->hstrt(5);
  axis.driver->hend(0);
  axis.driver->tbl(0);
  axis.driver->intpol(true);
  axis.driver->multistep_filt(true);
  axis.driver->microsteps(SETUP_MICROSTEPS_Y);
  axis.driver->rms_current(axis.currentMa, axisHoldMultiplier(axis));
  axis.driver->TPOWERDOWN(15); // ~327 ms standstill delay matching X and Z
  axis.driver->iholddelay(4);
  axis.driver->pwm_ofs(36);
  axis.driver->pwm_grad(14);
  if (axis.driver->pwm_grad_auto() == 0) {
    axis.driver->pwm_autograd(false);
    delay(2);
  }
  axis.driver->pwm_autoscale(true);
  axis.driver->pwm_autograd(true);
  axis.driver->TCOOLTHRS(0xFFFFF);
  axis.driver->TPWMTHRS(0);
  axis.driver->SGTHRS(axis.moveSensitivity);
}

void applyAxisDriverSettings(Axis& axis) {
  // Required for UART control + StallGuard
  axis.driver->pdn_disable(true);
  axis.driver->mstep_reg_select(true);
  axis.driver->I_scale_analog(false);
  axis.driver->shaft(axis.invertDir);

  if (axis.coordCode == 'Y') {
    applyYMoveMode(axis);
    return;
  }

  axis.driver->multistep_filt(true);
  axis.driver->toff(5);
  axis.driver->en_spreadCycle(false);  // StealthChop on (needed for StallGuard)
  axis.driver->pwm_ofs(36);
  axis.driver->pwm_grad(14);
  if (axis.driver->pwm_grad_auto() == 0) {
    axis.driver->pwm_autograd(false);
    delay(2);
  }
  axis.driver->pwm_autoscale(true);
  axis.driver->pwm_autograd(true);
  axis.driver->intpol(true);           // Enable TMC2209 internal 256x microstep interpolation for silent, smooth motion
  axis.driver->TCOOLTHRS(0xFFFFF);     // Enable StallGuard/DIAG at all speeds above ~0
  axis.driver->TPWMTHRS(0);

  // Restore default maxSpeed and acceleration so prior MOVE feedrate caps never taint SETUP/HOME
  axis.maxSpeed     = (axis.coordCode == 'X') ? DEFAULT_MAX_SPEED_X : DEFAULT_MAX_SPEED_Z;
  axis.acceleration = (axis.coordCode == 'X') ? DEFAULT_ACCEL_X : DEFAULT_ACCEL_Z;
  axis.stepper->setMinPulseWidth(2);
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
  if (stepTimer != nullptr) {
    timerStop(stepTimer);
  }
  portENTER_CRITICAL(&stepMux);
  for (int i = 0; i < NUM_AXES; i++) {
    isrAxes[i].active = false;
  }
  portEXIT_CRITICAL(&stepMux);
  for (int i = 0; i < NUM_AXES; i++) {
    AccelStepper* s = axes[i].stepper;
    syncAxisPosition(axes[i], s->currentPosition());
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
  SCurveAxis sc;
  initSCurveAxis(sc, axis, stepper->currentPosition() + steps);

  unsigned long lastWdtYieldMs = millis();
  unsigned long moveStartUs = micros();
  startStepEngine(&sc, 1);
  while (tickStepEngine(&sc, 1, moveStartUs)) {
    yieldWdt(lastWdtYieldMs);
    if (checkHardwareEstop()) {
      stopStepEngine(&sc, 1);
      sleepAll();
      setRelayOutput(false);
      Serial.println("[ALERT] Hardware E-STOP triggered. Steppers disabled.");
      return false;
    }
    if (sc.active && isStalled(axis)) {
      stopStepEngine(&sc, 1);
      sleepAll(); // Cut power immediately!
      Serial.print("[ALERT] "); Serial.print(axis.name);
      Serial.print(" stall detected (SG_RESULT="); Serial.print(axis.driver->SG_RESULT());
      Serial.println("). Steppers disabled (asleep). Position may be inaccurate.");
      return false;
    }
    delayMicroseconds(250);
  }
  stopStepEngine(&sc, 1);

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

  if (digitalRead(axis.diagPin) != HIGH) {
    if (axis.coordCode == 'Y') axis.diagFirstHighMs = 0;
    return false;
  }

  // Y belt pulley emits a periodic ~5.3 ms DIAG square wave during normal fast travel; require
  // continuous HIGH >= 10 ms on Y (matching homeAxis) while X/Z keep 15 ms dual-sample confirm.
  if (axis.coordCode == 'Y') {
    if (axis.diagFirstHighMs == 0) {
      axis.diagFirstHighMs = (nowMs == 0) ? 1UL : nowMs;
      return false;
    }
    return (nowMs - axis.diagFirstHighMs) >= 10UL;
  }

  if (axis.diagFirstHighMs == 0 || nowMs - axis.diagFirstHighMs > STALL_CONFIRM_WINDOW_MS) {
    axis.diagFirstHighMs = (nowMs == 0) ? 1UL : nowMs;
    return false;
  }
  return (nowMs - axis.diagFirstHighMs) >= 1UL;
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
  unsigned long minMs = knownDistance ? 3000UL : ((axis.coordCode == 'Y') ? 8000UL : HOMING_TIMEOUT_MS);
  if (!knownDistance && axis.coordCode == 'Y' && ms > 8000UL) {
    ms = 8000UL;
  }
  return ms > minMs ? ms : minMs;
}

// Single-axis move used for the pre-home pull-away. Does NOT sleep the drivers on a stall.
MotionResult runAxisTo(Axis& axis, long target) {
  SCurveAxis sc;
  initSCurveAxis(sc, axis, target);
  unsigned long lastWdtYieldMs = millis();
  unsigned long moveStartUs = micros();
  startStepEngine(&sc, 1);
  while (tickStepEngine(&sc, 1, moveStartUs)) {
    yieldWdt(lastWdtYieldMs);
    if (sc.active && isStalled(axis)) {
      stopStepEngine(&sc, 1);
      return MOTION_STALLED;
    }
    if (pollCancelInput()) {
      stopStepEngine(&sc, 1);
      return MOTION_CANCELLED;
    }
    delayMicroseconds(250);
  }
  stopStepEngine(&sc, 1);
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
  bool diagHighActive = false;
  unsigned long diagHighStartUs = 0;
  const unsigned long diagDebounceUs = (axis.coordCode == 'Y') ? 600UL : 150UL;

  // Non-blocking UART SG_RESULT telemetry sampler for Y while cruising post-blanking (no step jitter)
  const unsigned long Y_SG_POLL_MS = 5;
  const unsigned long Y_SG_WARMUP_MS = HOMING_RAMP_MS + 20;
  unsigned long lastYSgPollMs = 0;
  bool ySgReqPending = false;
  uint16_t ySgMin = 1023;
  uint16_t ySgMax = 0;
  uint32_t ySgSum = 0;
  int ySgSamples = 0;
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

    // Collect live SG_RESULT telemetry for Y during post-blanking cruise
    if (axis.coordCode == 'Y' && axis.homeSensitivity > 0 && elapsed >= Y_SG_WARMUP_MS && (nowMs - lastYSgPollMs >= Y_SG_POLL_MS)) {
      lastYSgPollMs = nowMs;
      if (ySgReqPending && TMCSerial.available() >= 12) {
        uint8_t buf[16];
        int n = 0;
        while (TMCSerial.available() && n < 16) buf[n++] = (uint8_t)TMCSerial.read();
        if (n >= 12 && buf[4] == 0x05 && buf[5] == 0xFF && buf[6] == 0x41 && buf[11] == tmc2209Crc(&buf[4], 7)) {
          uint16_t sg = ((uint16_t)(buf[9] & 0x03) << 8) | buf[10];
          if (elapsed > HOMING_BLANKING_MS) {
            if (sg < ySgMin) ySgMin = sg;
            if (sg > ySgMax) ySgMax = sg;
            ySgSum += sg;
            ySgSamples++;
          }
        }
      }
      if (outcome == RUNNING) {
        while (TMCSerial.available()) TMCSerial.read();
        TMCSerial.write(ySgReq, 4);
        ySgReqPending = true;
      }
    }

    // Check DIAG after blanking window (350 ms) with clean continuous-HIGH debounce (2 ms on Y, 150 us on X/Z)
    if (outcome == RUNNING && elapsed > HOMING_BLANKING_MS) {
      if (digitalRead(axis.diagPin) == HIGH) {
        unsigned long nowUs = micros();
        if (!diagHighActive) {
          diagHighActive = true;
          diagHighStartUs = nowUs;
        } else if ((nowUs - diagHighStartUs) >= diagDebounceUs) {
          outcome = HOMED;
        }
      } else {
        diagHighActive = false;
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
      long homePos = 0;
      if (axis.coordCode == 'Y') {
        float homePosMm = towardOrigin ? 0.0f : (dirSign * axis.maxTravelMm);
        if (!axis.homeReverse) homePosMm = -homePosMm;
        if (towardOrigin) {
          const float postHomePulloffMm = 1.0f;
          syncAxisPosition(axis, -lroundf(postHomePulloffMm * axis.stepsPerMm));
          delay(10);
          runAxisTo(axis, 0);
        }
        homePos = lroundf(homePosMm * axis.stepsPerMm);
      } else {
        homePos = towardOrigin ? 0 : (long)(dirSign * lroundf(axis.maxTravelMm * axis.stepsPerMm));
        if (!axis.homeReverse) homePos = -homePos;
      }
      syncAxisPosition(axis, homePos);
      axis.homed = true;
      Serial.print("-> "); Serial.print(axis.name);
      Serial.print(" homed (SG_RESULT="); Serial.print(stallSg);
      Serial.print(", TSTEP="); Serial.print(stallTstep);
      if (axis.coordCode == 'Y' && ySgSamples > 0) {
        Serial.print(", SG_CRUISE="); Serial.print(ySgMin);
        Serial.print("/"); Serial.print(ySgSum / (uint32_t)ySgSamples);
        Serial.print("/"); Serial.print(ySgMax);
      }
      Serial.print("). Position set to "); Serial.print(homePos);
      Serial.println(towardOrigin ? "." : " (far end).");
      return true;
    }
    case TIMED_OUT:
      syncAxisPosition(axis, stepper->currentPosition());
      setRelayOutput(false, "homing timeout");
      Serial.print("[ERROR] "); Serial.print(axis.name);
      Serial.println(" homing timed out with no stall detected.");
      return false;
    default:
      syncAxisPosition(axis, stepper->currentPosition());
      if (estopLatched) { sleepAll(); setRelayOutput(false); }
      else { setRelayOutput(false, "homing cancelled"); }
      Serial.print("-> "); Serial.print(axis.name);
      Serial.println(" homing cancelled.");
      return false;
  }
}

// ============================================================
//  STEPPER CHIMES (TADA & WIN95 via verified SpreadCycle + DDS)
// ============================================================
static const int CHIME_MICROSTEPS_X  = 4;   // Verified 1:1 fundamental pitch (A4=440, Eb5=622.25)
static const int CHIME_MICROSTEPS_Y  = 8;   // Unused in 2-voice chimes (belt damps tone)
static const int CHIME_MICROSTEPS_Z  = 1;   // Verified 1:1 clean bass/harmony (C4=261.63, A4=440)
static const int CHIME_CURRENT_MA_XZ = 500; // Moderate chime boost (avoids rail droop / UVLO)
static const int CHIME_CURRENT_MA_Y  = 500; // Matches normal 480 mA Y motor rating in vsense=1

static int chimeMicrostepsFor(const Axis& axis) {
  if (axis.coordCode == 'X') return CHIME_MICROSTEPS_X;
  if (axis.coordCode == 'Y') return CHIME_MICROSTEPS_Y;
  return CHIME_MICROSTEPS_Z;
}

static int chimeCurrentMaFor(const Axis& axis) {
  return (axis.coordCode == 'Y') ? CHIME_CURRENT_MA_Y : CHIME_CURRENT_MA_XZ;
}

static bool isValidMicrostepDiv(int ms) {
  return ms >= 1 && ms <= 256 && (ms & (ms - 1)) == 0;
}

static bool enterChimeDriverMode(Axis& axis, int microstepsOverride = -1, int currentMaOverride = -1) {
  int ms = isValidMicrostepDiv(microstepsOverride) ? microstepsOverride : chimeMicrostepsFor(axis);
  // TMCStepper::microsteps() maps 0 -> mres(8) (1x full step); microsteps(1) is a no-op!
  uint16_t tmcMs = (ms <= 1) ? 0 : (uint16_t)ms;
  int currMa = (currentMaOverride > 0) ? constrain(currentMaOverride, 100, 950) : chimeCurrentMaFor(axis);

  for (int attempt = 0; attempt < 4; attempt++) {
    while (TMCSerial.available()) TMCSerial.read();
    uint8_t gstat = axis.driver->GSTAT();
    if (!axis.driver->CRCerror && (gstat & 0x01)) {
      axis.driver->GSTAT(0x07);
      axis.driver->begin();
    }

    uint8_t ifcntBefore = axis.driver->IFCNT();
    axis.driver->SGTHRS(0);
    axis.driver->en_spreadCycle(true);   // Crisp hysteresis current chopping for audible tones
    axis.driver->intpol(false);          // Disable 256x interpolation so step edges ring cleanly
    axis.driver->microsteps(tmcMs);
    axis.driver->rms_current(currMa, 1.0f); // Boost current + keep full hold across rests
    delay(3);

    uint8_t ifcntAfter = axis.driver->IFCNT();
    bool spreadOk = axis.driver->en_spreadCycle() && !axis.driver->CRCerror;
    bool intpolOk = !axis.driver->intpol() && !axis.driver->CRCerror;
    bool msOk     = (axis.driver->microsteps() == tmcMs) && !axis.driver->CRCerror;
    if (ifcntAfter != ifcntBefore && spreadOk && intpolOk && msOk) {
      return true;
    }
    delay(5);
  }
  return false;
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

static float chimeSegFreqForAxis(const ChimeSegment& seg, int axisIdx) {
  if (axisIdx == 0) return seg.freqXHz;
  if (axisIdx == 1) return seg.freqYHz;
  return seg.freqZHz;
}

static long chimePhraseSteps(
    const ChimeSegment* seq,
    unsigned int count,
    unsigned int startIdx,
    int axisIdx
) {
  float prevF = chimeSegFreqForAxis(seq[startIdx], axisIdx);
  if (prevF <= 1.0f) return 0;
  unsigned long prevMs = seq[startIdx].durationMs;
  long total = lroundf(prevF * ((float)prevMs / 1000.0f));
  for (unsigned int j = startIdx + 1; j < count; j++) {
    float fj = chimeSegFreqForAxis(seq[j], axisIdx);
    unsigned long mj = seq[j].durationMs;
    if (fj <= 1.0f) break;
    bool isScoopFollow = (prevMs <= 30UL && prevF > 1.0f);
    bool isSameFreq = (fabsf(fj - prevF) < 0.5f);
    if (!isScoopFollow && !isSameFreq) break;
    total += lroundf(fj * ((float)mj / 1000.0f));
    prevF = fj;
    prevMs = mj;
  }
  return total;
}

static int resolveChimeAxisDir(
    const ChimeSegment* seq,
    unsigned int count,
    unsigned int idx,
    int axisIdx,
    int awayDir,
    long netSteps,
    int lastDir,
    float prevFreq,
    unsigned long prevDurMs,
    long maxSafeOffsetSteps
) {
  float f = chimeSegFreqForAxis(seq[idx], axisIdx);
  if (f <= 1.0f) return awayDir;
  bool isScoopFollow = (prevDurMs <= 30UL && prevFreq > 1.0f);
  if (isScoopFollow) return lastDir;

  // Include grouped continuations when checking headroom so netSteps never crosses 0.
  long groupSteps = chimePhraseSteps(seq, count, idx, axisIdx);
  long awayProgress = netSteps * (long)awayDir;
  bool isSameFreq = (fabsf(f - prevFreq) < 0.5f);
  if (isSameFreq) {
    if (labs(netSteps + (long)lastDir * groupSteps) > maxSafeOffsetSteps && awayProgress >= groupSteps) {
      return -awayDir;
    }
    return lastDir;
  }
  return (awayProgress >= groupSteps) ? -awayDir : awayDir;
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
  initStepTimer();
  if (stepTimer != nullptr) {
    timerStop(stepTimer);
  }

  const float freqs[NUM_AXES] = { seg.freqXHz, seg.freqYHz, seg.freqZHz };
  const int   dirs[NUM_AXES]  = { seg.dirX,    seg.dirY,    seg.dirZ };

  bool anyActive = false;
  for (int i = 0; i < NUM_AXES; i++) {
    digitalWrite(axes[i].dirPin, (dirs[i] >= 0) ? HIGH : LOW);
    if (freqs[i] > 1.0f) anyActive = true;
  }
  delayMicroseconds(5);

  portENTER_CRITICAL(&stepMux);
  for (int i = 0; i < NUM_AXES; i++) {
    isrAxes[i].curPos    = 0;
    isrAxes[i].dir       = (dirs[i] >= 0) ? 1 : -1;
    isrAxes[i].targetPos = (long)isrAxes[i].dir * 2000000L;
    isrAxes[i].stepPin   = axes[i].stepPin;
    isrAxes[i].ddsPhase  = 0xFFFFFFFFUL;
    isrAxes[i].ddsInc    = (freqs[i] > 1.0f) ? speedToDdsInc(freqs[i]) : 0UL;
    isrAxes[i].active    = (freqs[i] > 1.0f);
  }
  portEXIT_CRITICAL(&stepMux);

  unsigned long startUs = micros();
  unsigned long durUs = seg.durationMs * 1000UL;
  unsigned long lastWdtYieldMs = millis();

  if (anyActive && stepTimer != nullptr) {
    timerRestart(stepTimer);
    timerStart(stepTimer);
  }

  while ((unsigned long)(micros() - startUs) < durUs) {
    yieldWdt(lastWdtYieldMs);
    if (pollChimeCancelOrQueue()) {
      cancelled = true;
      break;
    }
    delayMicroseconds(250);
  }

  if (stepTimer != nullptr) {
    timerStop(stepTimer);
  }
  portENTER_CRITICAL(&stepMux);
  for (int i = 0; i < NUM_AXES; i++) {
    isrAxes[i].active = false;
  }
  long dx = isrAxes[0].curPos;
  long dy = isrAxes[1].curPos;
  long dz = isrAxes[2].curPos;
  isrAxes[0].curPos = 0;
  isrAxes[1].curPos = 0;
  isrAxes[2].curPos = 0;
  portEXIT_CRITICAL(&stepMux);

  netStepsX += dx;
  netStepsY += dy;
  netStepsZ += dz;
  return !cancelled;
}

static bool enterSilentUnwindMode(Axis& axis, const SavedAxisSnapshot& snap, int chimeMs) {
  uint16_t tmcMs = (chimeMs <= 1) ? 0 : (uint16_t)chimeMs;
  for (int attempt = 0; attempt < 4; attempt++) {
    while (TMCSerial.available()) TMCSerial.read();
    uint8_t gstat = axis.driver->GSTAT();
    if (!axis.driver->CRCerror && (gstat & 0x01)) {
      axis.driver->GSTAT(0x07);
      axis.driver->begin();
    }

    uint8_t ifcntBefore = axis.driver->IFCNT();
    if (snap.hasRegs) {
      axis.driver->GCONF(snap.gconf);      // Restores StealthChop (en_spreadCycle=0), shaft, etc.
      axis.driver->PWMCONF(snap.pwmconf);  // Restores StealthChop autotuned PWM settings
    } else {
      axis.driver->en_spreadCycle(false);
    }
    axis.driver->intpol(true);             // 256x MicroPlyer smoothing during silent unwind
    axis.driver->microsteps(tmcMs);        // Keep chime step size so netSteps unwinds 1:1
    axis.driver->rms_current(axis.currentMa, axisHoldMultiplier(axis));
    delay(3);

    uint8_t ifcntAfter = axis.driver->IFCNT();
    bool stealthOk = !axis.driver->en_spreadCycle() && !axis.driver->CRCerror;
    bool intpolOk  = axis.driver->intpol() && !axis.driver->CRCerror;
    bool msOk      = (axis.driver->microsteps() == tmcMs) && !axis.driver->CRCerror;
    if (ifcntAfter != ifcntBefore && stealthOk && intpolOk && msOk) {
      return true;
    }
    delay(5);
  }
  return false;
}

static bool returnChimeOffsetSilently(
    long netStepsX,
    long netStepsY,
    long netStepsZ,
    const SavedAxisSnapshot snapshots[NUM_AXES],
    const int chimeMs[NUM_AXES]
) {
  long remX = labs(netStepsX);
  long remY = labs(netStepsY);
  long remZ = labs(netStepsZ);
  if (remX == 0 && remY == 0 && remZ == 0) {
    return true;
  }

  // Revert to quiet StealthChop + 256x interpolation and normal run current BEFORE jogging back,
  // while keeping chime microstep resolution so netSteps unwinds to the exact starting microstep.
  if (remX > 0) enterSilentUnwindMode(axes[0], snapshots[0], chimeMs[0]);
  if (remY > 0) enterSilentUnwindMode(axes[1], snapshots[1], chimeMs[1]);
  if (remZ > 0) enterSilentUnwindMode(axes[2], snapshots[2], chimeMs[2]);
  delay(15);

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
    delayMicroseconds(3000);
  }
  return true;
}

void handleTuneGcode(String params) {
  params.trim();
  params.toUpperCase();
  if (checkHardwareEstop()) {
    setRelayOutput(false, "hardware estop");
    Serial.println("ERR:ESTOP");
    return;
  }
  if (!ensureDriversReady()) {
    Serial.println("ERR:TMC_NOT_READY");
    return;
  }

  // Parse positional whitespace tokens: TUNE <X|Y|Z|ALL> <Hz> [ms] [ms_div] [current_mA]
  String toks[5];
  int tokCount = 0;
  int pos = 0;
  while (pos < (int)params.length() && tokCount < 5) {
    while (pos < (int)params.length() && isSpace(params.charAt(pos))) pos++;
    if (pos >= (int)params.length()) break;
    int end = pos;
    while (end < (int)params.length() && !isSpace(params.charAt(end))) end++;
    toks[tokCount++] = params.substring(pos, end);
    pos = end;
  }

  if (tokCount < 2) {
    Serial.println("ERR:USAGE_TUNE_<X|Y|Z|ALL>_<HZ>_[MS]_[MS_DIV]_[MA]");
    return;
  }

  bool activeMask[NUM_AXES] = { false, false, false };
  if (toks[0] == "X" || toks[0] == "S1")      activeMask[0] = true;
  else if (toks[0] == "Y" || toks[0] == "S2") activeMask[1] = true;
  else if (toks[0] == "Z" || toks[0] == "S3") activeMask[2] = true;
  else if (toks[0] == "ALL") {
    activeMask[0] = activeMask[1] = activeMask[2] = true;
  } else {
    Serial.println("ERR:BAD_AXIS");
    return;
  }

  float freqHz = constrain(toks[1].toFloat(), 50.0f, 3000.0f);
  unsigned long durationMs = (tokCount >= 3) ? (unsigned long)constrain(toks[2].toInt(), 100L, 8000L) : 2000UL;
  int reqMsDiv = (tokCount >= 4) ? toks[3].toInt() : -1;
  if (reqMsDiv > 0 && !isValidMicrostepDiv(reqMsDiv)) {
    Serial.println("ERR:BAD_MICROSTEPS");
    return;
  }
  int reqCurrMa = (tokCount >= 5) ? toks[4].toInt() : -1;

  SavedAxisSnapshot snapshots[NUM_AXES];
  for (int i = 0; i < NUM_AXES; i++) {
    captureAxisSnapshot(axes[i], snapshots[i]);
    axes[i].driver->GSTAT(0x07);
  }

  estopLatched = false;
  for (int i = 0; i < NUM_AXES; i++) {
    if (activeMask[i]) {
      enterChimeDriverMode(axes[i], reqMsDiv, reqCurrMa);
    }
  }
  delay(10);

  long netSteps[NUM_AXES] = { 0, 0, 0 };
  int tuneMs[NUM_AXES] = { chimeMicrostepsFor(axes[0]), chimeMicrostepsFor(axes[1]), chimeMicrostepsFor(axes[2]) };
  int awayDir[NUM_AXES] = { 1, 1, 1 };
  int curDir[NUM_AXES] = { 1, 1, 1 };
  long maxExcursionSteps[NUM_AXES] = { 0, 0, 0 };
  for (int i = 0; i < NUM_AXES; i++) {
    if (!activeMask[i]) continue;
    int effMs = (reqMsDiv > 0) ? reqMsDiv : chimeMicrostepsFor(axes[i]);
    tuneMs[i] = effMs;
    float tuneStepsPerMm = (MOTOR_FULL_STEPS_REV * (float)effMs) / axes[i].mmPerRev;
    float curMm = (float)snapshots[i].stepperPos / axes[i].stepsPerMm;
    float safeExcursionMm = (i == 1) ? 16.0f : 10.0f;
    if (axes[i].homed) {
      float headPlus  = max(0.0f, axes[i].maxTravelMm - curMm - 5.0f);
      float headMinus = max(0.0f, curMm - 5.0f);
      if (headPlus >= headMinus) {
        awayDir[i] = 1;
        safeExcursionMm = min(30.0f, max(4.0f, headPlus));
      } else {
        awayDir[i] = -1;
        safeExcursionMm = min(30.0f, max(4.0f, headMinus));
      }
    } else {
      awayDir[i] = (curMm > axes[i].maxTravelMm - 8.0f) ? -1 : 1;
    }
    curDir[i] = awayDir[i];
    maxExcursionSteps[i] = max(20L, lroundf(safeExcursionMm * tuneStepsPerMm));
    digitalWrite(axes[i].dirPin, (curDir[i] >= 0) ? HIGH : LOW);
  }
  delayMicroseconds(5);

  initStepTimer();
  if (stepTimer != nullptr) timerStop(stepTimer);
  uint32_t ddsInc = speedToDdsInc(freqHz);
  portENTER_CRITICAL(&stepMux);
  for (int i = 0; i < NUM_AXES; i++) {
    isrAxes[i].curPos    = 0;
    isrAxes[i].dir       = curDir[i];
    isrAxes[i].targetPos = (long)curDir[i] * maxExcursionSteps[i];
    isrAxes[i].stepPin   = axes[i].stepPin;
    isrAxes[i].ddsPhase  = 0xFFFFFFFFUL;
    isrAxes[i].ddsInc    = ddsInc;
    isrAxes[i].active    = activeMask[i];
  }
  portEXIT_CRITICAL(&stepMux);

  bool cancelled = false;
  unsigned long startUs = micros();
  unsigned long durUs = durationMs * 1000UL;
  unsigned long lastWdtYieldMs = millis();

  if (stepTimer != nullptr) {
    timerRestart(stepTimer);
    timerStart(stepTimer);
  }

  while ((unsigned long)(micros() - startUs) < durUs) {
    yieldWdt(lastWdtYieldMs);
    if (pollChimeCancelOrQueue()) {
      cancelled = true;
      break;
    }

    // Safely bounce direction only if single-sweep excursion limit is reached
    for (int i = 0; i < NUM_AXES; i++) {
      if (!activeMask[i]) continue;
      bool hitLimit = false;
      long stepDelta = 0;
      portENTER_CRITICAL(&stepMux);
      if (!isrAxes[i].active && isrAxes[i].curPos == isrAxes[i].targetPos) {
        stepDelta = isrAxes[i].curPos;
        isrAxes[i].curPos = 0;
        hitLimit = true;
      }
      portEXIT_CRITICAL(&stepMux);

      if (hitLimit) {
        netSteps[i] += stepDelta;
        curDir[i] = -curDir[i];
        digitalWrite(axes[i].dirPin, (curDir[i] >= 0) ? HIGH : LOW);
        delayMicroseconds(5);
        long nextTarget = (curDir[i] == awayDir[i])
            ? ((long)awayDir[i] * maxExcursionSteps[i] - netSteps[i])
            : -netSteps[i];
        if (nextTarget == 0) nextTarget = (long)curDir[i] * maxExcursionSteps[i];
        portENTER_CRITICAL(&stepMux);
        isrAxes[i].dir       = curDir[i];
        isrAxes[i].targetPos = nextTarget;
        isrAxes[i].active    = true;
        portEXIT_CRITICAL(&stepMux);
      }
    }
    delayMicroseconds(250);
  }

  if (stepTimer != nullptr) timerStop(stepTimer);
  portENTER_CRITICAL(&stepMux);
  for (int i = 0; i < NUM_AXES; i++) {
    isrAxes[i].active = false;
    netSteps[i] += isrAxes[i].curPos;
    isrAxes[i].curPos = 0;
  }
  portEXIT_CRITICAL(&stepMux);

  if (cancelled && estopLatched) {
    sleepAll();
    setRelayOutput(false);
    Serial.println("ERR:ESTOP");
    return;
  }

  for (int i = 0; i < NUM_AXES; i++) {
    if (!activeMask[i]) continue;
    uint16_t msRead = axes[i].driver->microsteps();
    Serial.print("TUNE "); Serial.print(axes[i].coordCode);
    Serial.print(" HZ:"); Serial.print(freqHz, 2);
    Serial.print(" MS:"); Serial.print(msRead == 0 ? 1 : (int)msRead);
    Serial.print(" SPREAD:"); Serial.print(axes[i].driver->en_spreadCycle());
    Serial.print(" INTPOL:"); Serial.print(axes[i].driver->intpol());
    Serial.print(" IRUN:"); Serial.print(axes[i].driver->irun());
    Serial.print(" VSENSE:"); Serial.print(axes[i].driver->vsense());
    Serial.print(" NET:"); Serial.println(netSteps[i]);
  }

  if (!returnChimeOffsetSilently(netSteps[0], netSteps[1], netSteps[2], snapshots, tuneMs)) {
    sleepAll();
    setRelayOutput(false);
    Serial.println("ERR:ESTOP");
    return;
  }

  for (int i = 0; i < NUM_AXES; i++) {
    if (!activeMask[i]) continue;
    syncAxisPosition(axes[i], snapshots[i].stepperPos);
    restoreAxisSnapshot(axes[i], snapshots[i]);
  }
  delay(max(DRIVER_ENABLE_SETTLE_MS, 250UL));
  Serial.println(cancelled ? "ERR:CANCELLED" : "OK");
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

  // Safe directions pointing away from the closer mechanical stop
  int awayX = (curXMm > TRAVEL_X_MM * 0.5f) ? -1 : 1;
  int awayY = (curYMm > TRAVEL_Y_MM * 0.5f) ? -1 : 1;
  int awayZ = (curZMm > TRAVEL_Z_MM * 0.5f) ? -1 : 1;

  // Calibrated 2-voice arrangements (X @ 4x = 1:1 treble/melody, Z @ 1x = 1:1 bass/harmony, Y muted):
  // Windows 3.1 "Tada!" (same C6+E4 brass chord repeated twice with 22ms lip-up attack, second sustains longer)
  const ChimeSegment tada[] = {
    {  995.00f, 0.00f, 314.00f,   22UL, 0, 0, 0 }, // brass attack scoop into C6+E4
    { 1046.50f, 0.00f, 329.63f,   73UL, 0, 0, 0 }, // "Ta-" (C6 + E4 short burst)
    {    0.00f, 0.00f,   0.00f,   30UL, 0, 0, 0 }, // crisp articulation breath
    {  995.00f, 0.00f, 314.00f,   22UL, 0, 0, 0 }, // matching brass attack scoop into C6+E4
    { 1046.50f, 0.00f, 329.63f, 1020UL, 0, 0, 0 }, // "-DAAA!" (C6 + E4 sustained burst)
  };

  // Windows 95 Startup Theme ("The Microsoft Sound", spectral transcription at original A4=428.5Hz pitch)
  const ChimeSegment win95[] = {
    {  764.40f, 0.00f, 169.50f,  880UL, 0, 0, 0 }, // G5 bell + F3 bass drone (0.00-0.88s)
    { 1145.00f, 0.00f, 254.50f,  100UL, 0, 0, 0 }, // D6 bell leap + C4 chord swell
    { 1145.00f, 0.00f, 301.50f,  100UL, 0, 0, 0 }, // D6 (cont) + Eb4 chord bloom
    { 1145.00f, 0.00f, 340.30f,  910UL, 0, 0, 0 }, // D6 (cont) + F4 warm hold
    { 1018.50f, 0.00f, 340.30f,  330UL, 0, 0, 0 }, // C6 echo ping 1 + F4 sustain
    {    0.00f, 0.00f, 254.50f,  510UL, 0, 0, 0 }, // C4 pad sustain between pings
    { 1018.50f, 0.00f, 340.30f,  330UL, 0, 0, 0 }, // C6 echo ping 2 + F4 sustain
    {    0.00f, 0.00f, 254.50f,  510UL, 0, 0, 0 }, // C4 pad sustain between pings
    { 1018.50f, 0.00f, 340.30f,  330UL, 0, 0, 0 }, // C6 echo ping 3 + F4 sustain
    {    0.00f, 0.00f, 254.50f,  510UL, 0, 0, 0 }, // C4 pad sustain between pings
    { 1018.50f, 0.00f, 340.30f,  850UL, 0, 0, 0 }, // C6 echo ping 4 + F4 (longer final ring)
  };

  const bool isWin95 = (params == "WIN95" || params == "95");
  const ChimeSegment* seq = isWin95 ? win95 : tada;
  const unsigned int count = isWin95
      ? (sizeof(win95) / sizeof(win95[0]))
      : (sizeof(tada) / sizeof(tada[0]));

  bool usedMask[NUM_AXES] = { false, false, false };
  for (unsigned int i = 0; i < count; i++) {
    if (seq[i].freqXHz > 1.0f) usedMask[0] = true;
    if (seq[i].freqYHz > 1.0f) usedMask[1] = true;
    if (seq[i].freqZHz > 1.0f) usedMask[2] = true;
  }

  estopLatched = false;
  for (int i = 0; i < NUM_AXES; i++) {
    if (usedMask[i]) {
      enterChimeDriverMode(axes[i]);
    }
  }
  delay(10);

  long netStepsX = 0;
  long netStepsY = 0;
  long netStepsZ = 0;
  bool cancelled = false;

  int lastDirX = awayX, lastDirY = awayY, lastDirZ = awayZ;
  float prevFreqX = -1.0f, prevFreqY = -1.0f, prevFreqZ = -1.0f;
  unsigned long prevDurMs = 0;
  const long MAX_CHIME_OFFSET_X = 2500L; // 25.0 mm @ 4x
  const long MAX_CHIME_OFFSET_Y = 800L;  // 20.0 mm @ 8x
  const long MAX_CHIME_OFFSET_Z = 500L;  // 20.0 mm @ 1x
  for (unsigned int i = 0; i < count; i++) {
    ChimeSegment seg = seq[i];
    seg.dirX = resolveChimeAxisDir(seq, count, i, 0, awayX, netStepsX, lastDirX, prevFreqX, prevDurMs, MAX_CHIME_OFFSET_X);
    seg.dirY = resolveChimeAxisDir(seq, count, i, 1, awayY, netStepsY, lastDirY, prevFreqY, prevDurMs, MAX_CHIME_OFFSET_Y);
    seg.dirZ = resolveChimeAxisDir(seq, count, i, 2, awayZ, netStepsZ, lastDirZ, prevFreqZ, prevDurMs, MAX_CHIME_OFFSET_Z);
    lastDirX = seg.dirX; lastDirY = seg.dirY; lastDirZ = seg.dirZ;
    prevFreqX = seg.freqXHz; prevFreqY = seg.freqYHz; prevFreqZ = seg.freqZHz;
    prevDurMs = seg.durationMs;
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

  int chimeMs[NUM_AXES] = {
    chimeMicrostepsFor(axes[0]),
    chimeMicrostepsFor(axes[1]),
    chimeMicrostepsFor(axes[2])
  };
  if (!returnChimeOffsetSilently(netStepsX, netStepsY, netStepsZ, snapshots, chimeMs)) {
    sleepAll();
    setRelayOutput(false);
    if (emitProtocolReply) Serial.println("ERR:ESTOP");
    return;
  }

  for (int i = 0; i < NUM_AXES; i++) {
    if (!usedMask[i]) continue;
    syncAxisPosition(axes[i], snapshots[i].stepperPos);
    restoreAxisSnapshot(axes[i], snapshots[i]);
  }
  // Allow StealthChop AT#1 to calibrate cleanly at standstill using exact restored settings
  delay(max(DRIVER_ENABLE_SETTLE_MS, 250UL));

  if (emitProtocolReply) {
    Serial.println(cancelled ? "ERR:CANCELLED" : "OK");
  }
}
