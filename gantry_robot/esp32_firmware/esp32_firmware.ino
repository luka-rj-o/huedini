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
         SLEEP
         ESTOP
    2. Interactive manual testing commands from HUE-DINI-NOV3.txt:
         setup, sleep, s1 <steps>, s1 c/m/s/v/a/r/hv/hs, home s1 [-],
         servo <deg>, relay <sec>, l, help
*/

#include <Arduino.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <TMCStepper.h>
#include <HX711.h>

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

// Optional hardware start button (set >= 0 if wired)
const int PIN_START_BTN = -1;

// ============================================================
//  SETTINGS & KINEMATIC DEFAULTS
// ============================================================
const float R_SENSE = 0.11f;

// Values applied to steppers by the "setup" command
const int SETUP_CURRENT_MA_XZ = 300;
const int SETUP_CURRENT_MA_Y  = 700; // Optimal 700mA for Y-axis carrying X+Z gantry bridge on belt
const int SETUP_MICROSTEPS    = 2;
const int SETUP_STALL_SENS    = 0;   // 0 = disable false-stall aborts during normal moves (StallGuard active during homing)

// Per-axis defaults (speeds in microsteps/s, acceleration in microsteps/s^2)
const float DEFAULT_MAX_SPEED_XZ  = 1000.0f; // 5.0 mm/s on 2mm lead screw
const float DEFAULT_ACCEL_XZ      = 2000.0f; // 10.0 mm/s^2 on 2mm lead screw
const float DEFAULT_HOME_SPEED_XZ = 500.0f;  // 2.5 mm/s on 2mm lead screw

const float DEFAULT_MAX_SPEED_Y   = 700.0f;  // 70.0 mm/s on 40mm/rev GT2 20T belt
const float DEFAULT_ACCEL_Y       = 1000.0f; // 100.0 mm/s^2 on 40mm/rev GT2 20T belt
const float DEFAULT_HOME_SPEED_Y  = 700.0f;  // 70.0 mm/s (1.75 rev/s for healthy back-EMF)

const int   DEFAULT_HOME_SENS     = 20;
const int   DEFAULT_HOME_SENS_Y   = 3;       // Homing sensitivity threshold for Y belt pulley (SGTHRS=3)

// X & Z: Lead screw pitch = 2.0 mm/rev, NEMA 11 = 200 full steps/rev -> 200.0 microsteps/mm @ 2x microsteps.
// Y: GT2 belt (2.0 mm pitch) x 20-tooth pulley = 40.0 mm/rev -> 10.0 microsteps/mm @ 2x microsteps.
const float LEAD_SCREW_PITCH_MM    = 2.0f;
const float BELT_PULLEY_MM_REV_Y   = 40.0f; // 20 teeth * 2.0 mm GT2 pitch
const float MOTOR_FULL_STEPS_REV   = 200.0f;
const float DEFAULT_STEPS_PER_MM_X = (MOTOR_FULL_STEPS_REV * SETUP_MICROSTEPS) / LEAD_SCREW_PITCH_MM;  // 200.0f
const float DEFAULT_STEPS_PER_MM_Y = (MOTOR_FULL_STEPS_REV * SETUP_MICROSTEPS) / BELT_PULLEY_MM_REV_Y; // 10.0f
const float DEFAULT_STEPS_PER_MM_Z = (MOTOR_FULL_STEPS_REV * SETUP_MICROSTEPS) / LEAD_SCREW_PITCH_MM;  // 200.0f

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

// Load cell default calibration scale factor (raw counts per gram)
float hx711ScaleFactor = 420.0f;

// Solenoid gripper relay logic:
// Because the solenoid is wired Normally Closed (NC):
//   - GRIP 1 (closed/gripping) -> relay LOW (de-energized, NC holds grip without heating solenoid)
//   - GRIP 0 (open/released)   -> relay HIGH (energized, opens gripper jaws)
// If your mechanical linkage works in the opposite sense, toggle `gripOpenRelayHigh`.
bool gripOpenRelayHigh = true;

const unsigned long HOMING_RAMP_MS     = 350;    // smooth linear acceleration ramp during homing
const unsigned long HOMING_BLANKING_MS = 800;    // ignore DIAG during ramp & initial belt settling (800ms)
const unsigned long HOMING_TIMEOUT_MS  = 30000;  // give up if no stall within 30 s

// Stall detection during moves only runs near full speed
const float STALL_CHECK_SPEED_RATIO = 0.9f;

// Wrist servo mechanical zero trim (degrees added to logical 0..180 commands)
float servoZeroOffsetDeg = 12.0f;

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
};

const int NUM_AXES = 3;
Axis axes[NUM_AXES] = {
  { "S1", 'X', PIN_DIAG_1, &stepper1, &driver1, false, DEFAULT_MAX_SPEED_XZ, DEFAULT_ACCEL_XZ, DEFAULT_HOME_SPEED_XZ, SETUP_STALL_SENS, DEFAULT_HOME_SENS,   DEFAULT_STEPS_PER_MM_X, LEAD_SCREW_PITCH_MM,  DEFAULT_HOME_DIR_REVERSE_X, INVERT_DIR_X, SETUP_CURRENT_MA_XZ },
  { "S2", 'Y', PIN_DIAG_2, &stepper2, &driver2, false, DEFAULT_MAX_SPEED_Y,  DEFAULT_ACCEL_Y,  DEFAULT_HOME_SPEED_Y,  SETUP_STALL_SENS, DEFAULT_HOME_SENS_Y, DEFAULT_STEPS_PER_MM_Y, BELT_PULLEY_MM_REV_Y, DEFAULT_HOME_DIR_REVERSE_Y, INVERT_DIR_Y, SETUP_CURRENT_MA_Y },
  { "S3", 'Z', PIN_DIAG_3, &stepper3, &driver3, false, DEFAULT_MAX_SPEED_XZ, DEFAULT_ACCEL_XZ, DEFAULT_HOME_SPEED_XZ, SETUP_STALL_SENS, DEFAULT_HOME_SENS,   DEFAULT_STEPS_PER_MM_Z, LEAD_SCREW_PITCH_MM,  DEFAULT_HOME_DIR_REVERSE_Z, INVERT_DIR_Z, SETUP_CURRENT_MA_XZ },
};

float currentAngleDeg = 0.0f;
int   currentGripState = 1; // 1 = gripping (NC default), 0 = open

void writeServoLogicalAngle(float logicalDeg) {
  logicalDeg = constrain(logicalDeg, 0.0f, 180.0f);
  currentAngleDeg = logicalDeg;
  int physDeg = (int)roundf(constrain(logicalDeg + servoZeroOffsetDeg, 0.0f, 180.0f));
  servo.write(physDeg);
}

// Forward declarations
void printHelp();
void handleCommand(String command);
bool setupAll(bool quietProtocol = false);
bool setupAxis(Axis& axis);
void sleepAll();
bool homeAxis(Axis& axis, bool reverse);
bool moveAxis(Axis& axis, long steps);
bool isStalled(Axis& axis);
void handleMoveGcode(String params);
void handleHomeAllGcode();
void handleGripGcode(String params);
void handleTareGcode();
void handleWeighGcode();
void handleStatusGcode();
void handleStepsGcode(String params);
void handleStepperCommand(Axis& axis, String param);
void handleHomeSettings(Axis& axis, String param);
void handleHomeCommand(String param);
void handleServoCommand(String param);
void handleRelayCommand(String param);
void handleLoadCellCommand();
void setCurrent(Axis& axis, int mA);
void setMicrosteps(Axis& axis, int microsteps);
void setSensitivity(Axis& axis, int sensitivity);
void setMaxSpeed(Axis& axis, float speed);
void setAcceleration(Axis& axis, float accel);
void readStallGuard(Axis& axis);

// ============================================================
//  ARDUINO ENTRY POINTS
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, HIGH);         // Drivers disabled until "setup" or first motion command

  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);       // Relay off (Normally Closed solenoid in default state)

  if (PIN_START_BTN >= 0) {
    pinMode(PIN_START_BTN, INPUT_PULLUP);
  }

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
  if (PIN_START_BTN >= 0 && digitalRead(PIN_START_BTN) == LOW) {
    delay(50);
    if (digitalRead(PIN_START_BTN) == LOW) {
      Serial.println("EVENT:START_BTN");
      while (digitalRead(PIN_START_BTN) == LOW) {
        delay(10);
      }
    }
  }

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() > 0) {
      handleCommand(command);
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
  String upper = rawCommand;
  upper.trim();
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
  if (upper.startsWith("STEPS ")) {
    handleStepsGcode(upper.substring(6));
    return;
  }
  if (upper == "ESTOP") {
    sleepAll();
    digitalWrite(PIN_RELAY, LOW);
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
      Serial.println("ERR:TMC_SETUP_FAILED");
    }
  }
  else if (command == "sleep") {
    sleepAll();
    Serial.println("OK");
  }
  else if (command == "help")            { printHelp(); Serial.println("OK"); }
  else if (command == "l")               { handleLoadCellCommand(); Serial.println("OK"); }
  else if (command.startsWith("home "))  { handleHomeCommand(command.substring(5)); Serial.println("OK"); }
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
  if (!ensureDriversReady()) {
    Serial.println("ERR:TMC_NOT_READY");
    return;
  }

  // Home Z (S3) first so the end-effector is clear of obstacles, then X (S1) and Y (S2)
  int homeOrder[3] = { 2, 0, 1 };
  for (int k = 0; k < 3; k++) {
    Axis& ax = axes[homeOrder[k]];
    if (!homeAxis(ax, ax.homeReverse)) {
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
  if (!ensureDriversReady()) {
    Serial.println("ERR:TMC_NOT_READY");
    return;
  }

  float feedrateMmPerSec = 0.0f;
  if (parseCoordToken(params, 'F', feedrateMmPerSec) && feedrateMmPerSec > 0.0f) {
    for (int i = 0; i < NUM_AXES; i++) {
      float spdSteps = constrain(feedrateMmPerSec * axes[i].stepsPerMm, 10.0f, 20000.0f);
      axes[i].maxSpeed = spdSteps;
      axes[i].stepper->setMaxSpeed(spdSteps);
    }
  }

  // Update servo angle A if provided
  float targetAngle = currentAngleDeg;
  if (parseCoordToken(params, 'A', targetAngle)) {
    writeServoLogicalAngle(targetAngle);
  }

  // Set target positions for X (S1), Y (S2), Z (S3)
  for (int i = 0; i < NUM_AXES; i++) {
    float targetMm = 0.0f;
    if (parseCoordToken(params, axes[i].coordCode, targetMm)) {
      long targetSteps = lroundf(targetMm * axes[i].stepsPerMm);
      axes[i].stepper->moveTo(targetSteps);
    } else {
      axes[i].stepper->moveTo(axes[i].stepper->currentPosition());
    }
  }

  // Coordinated multi-axis step loop with StallGuard monitoring
  while (axes[0].stepper->distanceToGo() != 0 ||
         axes[1].stepper->distanceToGo() != 0 ||
         axes[2].stepper->distanceToGo() != 0) {
    for (int i = 0; i < NUM_AXES; i++) {
      if (axes[i].stepper->distanceToGo() != 0) {
        axes[i].stepper->run();
        if (isStalled(axes[i])) {
          for (int j = 0; j < NUM_AXES; j++) {
            axes[j].stepper->setCurrentPosition(axes[j].stepper->currentPosition());
          }
          Serial.print("ERR:STALL_");
          Serial.println(axes[i].coordCode);
          return;
        }
      }
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
  digitalWrite(PIN_RELAY, relayHigh ? HIGH : LOW);
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
  Serial.println(currentGripState);
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
  Serial.println("  GRIP <0|1> | TARE | WEIGH | STATUS | STEPS X<f> Y<f> Z<f> | ESTOP");
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
  Serial.println("  help        -> Show this list");
  Serial.println("---------------------------------\n");
}

void handleStepperCommand(Axis& axis, String param) {
  param.trim();
  if (param.length() == 0) return;

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
  if (param.length() == 0) return;

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
  } else {
    Serial.println("[ERROR] Use hv<speed> or hs<sensitivity>.");
  }
}

void handleHomeCommand(String param) {
  param.trim();
  if (param.length() < 2 || param.charAt(0) != 's') {
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
    digitalWrite(PIN_RELAY, HIGH);
    Serial.println("-> Relay ON");
    return;
  }
  if (param == "off") {
    digitalWrite(PIN_RELAY, LOW);
    Serial.println("-> Relay OFF");
    return;
  }

  int seconds = param.toInt();
  if (seconds <= 0) seconds = 2;

  Serial.print("-> Relay ON for ");
  Serial.print(seconds);
  Serial.println(" seconds...");

  digitalWrite(PIN_RELAY, HIGH);
  delay(seconds * 1000);
  digitalWrite(PIN_RELAY, LOW);

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

  if (!quietProtocol) {
    Serial.print("-> Setup done: ");
    Serial.print(readyCount);
    Serial.print(" of ");
    Serial.print(NUM_AXES);
    Serial.println(" steppers ready.");
  }
  return (readyCount == NUM_AXES);
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

  // Required for UART control + StallGuard
  axis.driver->pdn_disable(true);
  axis.driver->I_scale_analog(false);
  axis.driver->shaft(axis.invertDir);
  axis.driver->toff(5);
  axis.driver->en_spreadCycle(false);  // StealthChop on (needed for StallGuard)
  axis.driver->pwm_autoscale(true);
  axis.driver->TCOOLTHRS(0xFFFFF);     // Enable StallGuard/DIAG at all speeds above ~0

  // User-facing defaults
  axis.driver->rms_current(axis.currentMa);
  axis.driver->microsteps(SETUP_MICROSTEPS);
  axis.moveSensitivity = SETUP_STALL_SENS;
  axis.driver->SGTHRS(axis.moveSensitivity);

  axis.ready = true;
  Serial.print("-> "); Serial.print(axis.name);
  Serial.print(" ready (");
  Serial.print(axis.currentMa); Serial.print(" mA, microsteps ");
  Serial.print(SETUP_MICROSTEPS); Serial.print(", steps/mm ");
  Serial.print(axis.stepsPerMm, 1); Serial.println(")");
  return true;
}

void sleepAll() {
  for (int i = 0; i < NUM_AXES; i++) {
    AccelStepper* s = axes[i].stepper;
    s->setCurrentPosition(s->currentPosition());
    axes[i].ready = false;
  }
  digitalWrite(PIN_EN, HIGH);
  Serial.println("-> All steppers asleep (disabled). Run 'setup' to wake them.");
}

// ============================================================
//  PER-AXIS SETTINGS
// ============================================================
void setCurrent(Axis& axis, int mA) {
  mA = constrain(mA, 50, 2000);
  axis.currentMa = mA;
  axis.driver->rms_current(mA);
  Serial.print("-> "); Serial.print(axis.name);
  Serial.print(" current limit: "); Serial.print(mA); Serial.println(" mA");
}

void setMicrosteps(Axis& axis, int microsteps) {
  axis.driver->microsteps(microsteps);
  if (microsteps > 0) {
    axis.stepsPerMm = (MOTOR_FULL_STEPS_REV * (float)microsteps) / axis.mmPerRev;
  }
  Serial.print("-> "); Serial.print(axis.name);
  Serial.print(" microstepping: "); Serial.print(microsteps);
  Serial.print(" (steps/mm: "); Serial.print(axis.stepsPerMm, 1); Serial.println(")");
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

  while (stepper->distanceToGo() != 0) {
    stepper->run();

    if (isStalled(axis)) {
      stepper->setCurrentPosition(stepper->currentPosition());
      Serial.print("[ALERT] "); Serial.print(axis.name);
      Serial.print(" stall detected (SG_RESULT="); Serial.print(axis.driver->SG_RESULT());
      Serial.println("). Move aborted. Position may be inaccurate.");
      return false;
    }
  }

  Serial.print("-> "); Serial.print(axis.name); Serial.println(" move complete.");
  return true;
}

bool isStalled(Axis& axis) {
  if (axis.moveSensitivity <= 0) return false;
  bool atCruiseSpeed = fabs(axis.stepper->speed()) >= (axis.maxSpeed * STALL_CHECK_SPEED_RATIO);
  return atCruiseSpeed && digitalRead(axis.diagPin) == HIGH;
}

bool homeAxis(Axis& axis, bool reverse) {
  if (!axis.ready) {
    Serial.print("[ERROR] "); Serial.print(axis.name);
    Serial.println(" not ready. Run 'setup' first.");
    return false;
  }

  Serial.print("-> Homing "); Serial.print(axis.name);
  Serial.print(reverse ? " backward" : " forward");
  Serial.print(" at "); Serial.print(axis.homeSpeed);
  Serial.print(" steps/s, sensitivity "); Serial.println(axis.homeSensitivity);

  AccelStepper* stepper = axis.stepper;

  axis.driver->SGTHRS(axis.homeSensitivity);
  stepper->setMaxSpeed(axis.homeSpeed);
  float startSpeed = min(80.0f, axis.homeSpeed);
  float dirSign = reverse ? -1.0f : 1.0f;
  stepper->setSpeed(dirSign * startSpeed);

  enum { RUNNING, HOMED, TIMED_OUT, CANCELLED } outcome = RUNNING;
  unsigned long startMs = millis();
  unsigned long lastRampMs = startMs;

  while (outcome == RUNNING) {
    stepper->runSpeed();

    unsigned long nowMs = millis();
    unsigned long elapsed = nowMs - startMs;

    // Smoothly ramp velocity during HOMING_RAMP_MS so belt axes don't skip steps on startup
    if (elapsed <= HOMING_RAMP_MS && (nowMs - lastRampMs) >= 10) {
      lastRampMs = nowMs;
      float frac = (float)elapsed / (float)HOMING_RAMP_MS;
      float curSpd = startSpeed + frac * (axis.homeSpeed - startSpeed);
      stepper->setSpeed(dirSign * curSpd);
    } else if (elapsed > HOMING_RAMP_MS && fabs(stepper->speed()) != axis.homeSpeed) {
      stepper->setSpeed(dirSign * axis.homeSpeed);
    }

    if (elapsed > HOMING_BLANKING_MS && digitalRead(axis.diagPin) == HIGH) {
      outcome = HOMED;
    } else if (elapsed > HOMING_TIMEOUT_MS) {
      outcome = TIMED_OUT;
    } else if (Serial.available() > 0) {
      while (Serial.available() > 0) Serial.read();
      outcome = CANCELLED;
    }
  }

  uint16_t stallSg = axis.driver->SG_RESULT();
  uint32_t stallTstep = axis.driver->TSTEP();

  axis.driver->SGTHRS(axis.moveSensitivity);
  stepper->setMaxSpeed(axis.maxSpeed);

  switch (outcome) {
    case HOMED:
      stepper->setCurrentPosition(0);
      Serial.print("-> "); Serial.print(axis.name);
      Serial.print(" homed (SG_RESULT="); Serial.print(stallSg);
      Serial.print(", TSTEP="); Serial.print(stallTstep);
      Serial.println("). Position set to 0.");
      return true;
    case TIMED_OUT:
      stepper->setCurrentPosition(stepper->currentPosition());
      Serial.print("[ERROR] "); Serial.print(axis.name);
      Serial.println(" homing timed out with no stall detected.");
      return false;
    default:
      stepper->setCurrentPosition(stepper->currentPosition());
      Serial.print("-> "); Serial.print(axis.name);
      Serial.println(" homing cancelled.");
      return false;
  }
}
