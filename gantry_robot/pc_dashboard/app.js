// ============================================================================
//  HUE-DINI Gantry Supervisor — Next Gen 3D CAD Digital Twin Dashboard
//  Compiled Onshape V3 Gantry WebGL Engine + 2D Tactical Map + Pi WebSocket
// ============================================================================

const $ = (id) => document.getElementById(id);

const wsUrlInput = $("wsUrlInput");
const connectBtn = $("connectBtn");
const courseCanvas = $("courseCanvas");
const canvasWrap = $("canvasWrap");
const ctx = courseCanvas.getContext("2d");

if (window.location.host) {
  wsUrlInput.value = `ws://${window.location.host}`;
}

// ============================================================================
//  EXACT CAD GEOMETRY & DATUM CONSTANTS (mm)
// ============================================================================
const PLATE_W_X = 284.0;
const PLATE_H_Y = 167.4;
const COURSE_ORIGIN_X = 15.0;
const COURSE_ORIGIN_Y = 0.0;
const COURSE_LEN_X = 254.0;
const COURSE_WID_Y = 152.4;
const FIDUCIAL_SIZE = 15.0;
const BLOCK_SIZE = 20.0;
const COURSE_FLOOR_Z = 6.35;

// Motor travel strokes from homed (0, 0, 0)
const STROKE_X = 227.0;
const STROKE_Y = 128.0;
const STROKE_Z_MAX = 76.7;
const STROKE_Z_BOTTOM = 70.36;

// Homed datum (tower origin)
const DATUM_X = 28.5;
const DATUM_Y = -43.5;
const DATUM_Z = 127.5;

// Nominal homed end-effector & camera (plate coordinates at motor (0, 0, 0))
const EE_HOMED_X = 31.0;
const EE_HOMED_Y = -8.5;
const EE_HOMED_Z = 97.6;
const CAM_HOMED_X = 32.5;
const CAM_HOMED_Y = 33.1;
const CAM_HOMED_Z = 135.4;
const EE_OFFSET_X = -1.5;
const EE_OFFSET_Y = -41.6;

// Aluminum drop-off pedestal
const DROPOFF_X = 224.6;
const DROPOFF_Y = 44.5;
const DROPOFF_Z_UP = 15.9;
const ALUM_PLATFORM_SIZE = 76.2;

// Camera sensor
const FRAME_W_PX = 640;
const FRAME_H_PX = 480;
const CAL_PLANE_Z = COURSE_FLOOR_Z + BLOCK_SIZE;

const CORNER_FIDUCIALS = [
  { id: 0, x: 7.5, y: 7.5 },
  { id: 1, x: 7.5, y: 159.9 },
  { id: 2, x: 276.5, y: 159.9 },
  { id: 3, x: 276.5, y: 7.5 },
];

const COLOR_HEX = {
  white: "#f1f5f9",
  red: "#ef4444",
  green: "#22c55e",
  blue: "#3b82f6",
  yellow: "#eab308",
  aluminum: "#94a3b8",
};
const LIGHT_BLOCKS = new Set(["white", "yellow", "aluminum"]);

const STACK_ZONES = [
  { label: "RED", color: "red", x: 250.2, y: 115.0, z: COURSE_FLOOR_Z },
  { label: "GREEN", color: "green", x: 210.2, y: 115.0, z: COURSE_FLOOR_Z },
  { label: "BLUE", color: "blue", x: 170.2, y: 115.0, z: COURSE_FLOOR_Z },
  { label: "ALUM", color: "aluminum", x: DROPOFF_X, y: DROPOFF_Y, z: DROPOFF_Z_UP },
];

const THEME = {
  grid: "rgba(255,255,255,0.035)",
  gridMajor: "rgba(255,255,255,0.07)",
  tick: "#5b6676",
  plateFill: "#141a23",
  plateStroke: "#2f3846",
  courseFill: "#18202b",
  courseStroke: "#3d4859",
  label: "#7d899a",
  labelStrong: "#a7b1bf",
  text: "#e6ebf2",
  origin: "#f0506e",
  ee: "#3ccf91",
  cam: "#4f9cf9",
  carriage: "#f5a524",
  fidLight: "#d9dfe8",
  fidDark: "#0b0e13",
  panel: "#11161e",
  panelStroke: "#232a35",
  mono: '"JetBrains Mono", "Cascadia Mono", Consolas, monospace',
  ui: '"Inter", "Segoe UI", system-ui, sans-serif',
};

// ============================================================================
//  STATE
// ============================================================================
let motorX = 0.0;
let motorY = 0.0;
let motorZDown = 0.0;
let servoAngle = 0.0;
let gripClosed = true;

let drawX = 0.0;
let drawY = 0.0;
let drawZ = 0.0;
let drawAngle = 0.0;
let drawGripFactor = 1.0; // 1 = closed, 0 = open

let sandboxPreviewActive = false;
let lastHwPoseSig = "";

let socket = null;
let latestState = {
  state: "IDLE",
  gantry_pose: { x_mm: 0.0, y_mm: 0.0, z_mm: 0.0, angle_deg: 0.0, grip_closed: true },
  current_mass_g: null,
  mass_class: null,
  calibration: { calibrated: false, kx_mm_per_px: 0.25, ky_mm_per_px: 0.25 },
  steps_per_mm: { x: 200.0, y: 10.0, z: 50.0 },
  blocks: [],
  logs: [],
  vision_frame_b64: null,
};

let currentJogStepMm = 10.0;
let mouseMm = null;
let twinMode = "split"; // split | 3d | 2d
let cameraPreset = "iso"; // iso | top | front | side | eyeinhand | follow

const layers = {
  fov: true,
  trail: true,
  reach: true,
  wireframe: false,
};

// Replay state
let replayActive = false;
let replayPlaying = false;
let replayFrames = [];
let replayIdx = 0;
let replayTimer = null;

function isOpen() {
  return socket && socket.readyState === WebSocket.OPEN;
}

function sendCmd(command, extra = {}) {
  const clearsPreview = ["START_SCAN", "RESET", "HOME_ALL", "JOG_AXIS", "FREE_MOTORS", "WAKE_MOTORS", "EMERGENCY_STOP"];
  if (clearsPreview.includes(command)) {
    exitReplayMode();
    sandboxPreviewActive = false;
    setSyncButtonVisible(false);
    if (command === "START_SCAN" || command === "RESET") {
      clearTrail();
    }
  }
  if (isOpen()) {
    socket.send(JSON.stringify({ command, ...extra }));
  }
}

// ============================================================================
//  RUN CONTROLS & WEBSOCKET
// ============================================================================
function togglePauseResume() {
  sendCmd(latestState.state === "PAUSED" ? "RESUME" : "PAUSE");
}

function syncPauseResumeButton(state) {
  const btn = $("pauseResumeBtn");
  if (!btn) return;
  const paused = state === "PAUSED";
  btn.querySelector(".ico").innerHTML = paused ? "&#9654;" : "&#10074;&#10074;";
  btn.querySelector(".lbl").textContent = paused ? "Resume" : "Pause";
  btn.classList.toggle("is-warning", paused);
}

let reconnectTimer = null;
function setConnStatus(up, text) {
  const dot = $("connDot");
  const label = $("connBadge");
  if (dot) {
    dot.dataset.status = up ? "up" : "down";
    dot.title = text;
  }
  if (label) label.textContent = text;
}

function connectWebSocket() {
  if (reconnectTimer) {
    clearTimeout(reconnectTimer);
    reconnectTimer = null;
  }
  if (socket) {
    socket.onclose = null;
    socket.onerror = null;
    try { socket.close(); } catch (_) {}
  }

  const url = wsUrlInput.value.trim();
  try {
    socket = new WebSocket(url);
  } catch (_) {
    setConnStatus(false, "Invalid URL");
    return;
  }
  setConnStatus(false, "Connecting…");

  socket.onopen = () => setConnStatus(true, "Connected");

  socket.onclose = () => {
    setConnStatus(false, "Reconnecting…");
    if (!reconnectTimer) {
      reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        connectWebSocket();
      }, 2500);
    }
  };

  socket.onerror = () => { /* onclose handles reconnection */ };

  socket.onmessage = (event) => {
    if (replayActive) return;
    try {
      const payload = JSON.parse(event.data);
      const prevState = latestState ? latestState.state : "IDLE";
      latestState = payload;
      updateUI(payload, prevState);
    } catch (err) {
      console.error("Invalid telemetry JSON:", err);
    }
  };
}

connectBtn.addEventListener("click", connectWebSocket);
wsUrlInput.addEventListener("keydown", (e) => {
  if (e.key === "Enter") connectWebSocket();
});

// ============================================================================
//  MANUAL JOG & LOCAL 4-DOF SANDBOX PREVIEW
// ============================================================================
function setJogStep(stepMm) {
  currentJogStepMm = stepMm;
  document.querySelectorAll(".jog-step-btn").forEach((b) => b.classList.remove("active"));
  const activeBtn = $(`jogStep${Math.round(stepMm)}`);
  if (activeBtn) activeBtn.classList.add("active");
}

function jogAxis(axis, dir) {
  exitReplayMode();
  if (sandboxPreviewActive) syncPoseToHardware();
  const uiDelta = dir * currentJogStepMm;
  const motorDelta = axis === "Z" ? -uiDelta : uiDelta;
  if (!isOpen()) return;

  if (axis === "X") motorX = clamp(motorX + motorDelta, 0, STROKE_X);
  else if (axis === "Y") motorY = clamp(motorY + motorDelta, 0, STROKE_Y);
  else if (axis === "Z") motorZDown = clamp(motorZDown + motorDelta, 0, STROKE_Z_BOTTOM);
  syncSlidersFromState();
  updateReadoutsFromMotor();

  socket.send(JSON.stringify({ command: "JOG_AXIS", axis, dist_mm: motorDelta }));
}

function setSyncButtonVisible(show) {
  const btn = $("syncHwPoseBtn");
  if (btn) btn.classList.toggle("hidden", !show);
}

function onSliderPose() {
  exitReplayMode();
  sandboxPreviewActive = true;
  setSyncButtonVisible(true);
  motorX = Number($("sliderX").value);
  motorY = Number($("sliderY").value);
  motorZDown = Number($("sliderZ").value);
  servoAngle = Number($("sliderA").value);
  syncSlidersFromState();
  updateReadoutsFromMotor();
}

function toggleJawManual() {
  sandboxPreviewActive = true;
  setSyncButtonVisible(true);
  gripClosed = !gripClosed;
  updateJawButton();
  updateReadoutsFromMotor();
}

function syncPoseToHardware() {
  sandboxPreviewActive = false;
  setSyncButtonVisible(false);
  const pose = latestState.gantry_pose || {};
  if (pose.x_mm !== undefined && pose.y_mm !== undefined) {
    motorX = clamp(Number(pose.x_mm) || 0.0, 0, STROKE_X);
    motorY = clamp(Number(pose.y_mm) || 0.0, 0, STROKE_Y);
    motorZDown = clamp(Number(pose.z_mm) || 0.0, 0, STROKE_Z_MAX);
    servoAngle = clamp(Number(pose.angle_deg) || 0.0, 0, 180);
    gripClosed = pose.grip_closed !== undefined ? !!pose.grip_closed : true;
  }
  syncSlidersFromState();
  updateReadoutsFromMotor();
}

function updateJawButton() {
  const btn = $("jawToggleBtn");
  if (btn) btn.textContent = gripClosed ? "Jaw: Closed" : "Jaw: Open";
}

function syncSlidersFromState() {
  if (!$("sliderX")) return;
  $("sliderX").value = motorX.toFixed(1);
  $("sliderY").value = motorY.toFixed(1);
  $("sliderZ").value = motorZDown.toFixed(1);
  $("sliderA").value = servoAngle.toFixed(0);

  setText("sliderXVal", motorX.toFixed(1));
  setText("sliderYVal", motorY.toFixed(1));
  setText("sliderZVal", (motorZDown > 0.01 ? -motorZDown : 0.0).toFixed(1));
  setText("sliderAVal", `${servoAngle.toFixed(0)}°`);
  updateJawButton();
}

// ============================================================================
//  LEAD SCREW CALIBRATION, CAMERA, LOAD CELL, CONSOLE, TABS
// ============================================================================
function setSpmTag(xz, y) {
  const tag = $("currentSpmTag");
  if (tag) tag.textContent = `X/Z ${Number(xz).toFixed(1)} · Y ${Number(y).toFixed(1)}`;
}

function applyLeadPreset(leadMm, spm) {
  document.querySelectorAll(".preset-btn").forEach((b) => b.classList.remove("active"));
  const btn = $(`presetT8x${Math.round(leadMm)}`);
  if (btn) btn.classList.add("active");
  setSpmTag(spm, 10.0);
  sendCmd("SET_STEPS_CALIBRATION", { steps_per_mm_x: spm, steps_per_mm_y: 10.0, steps_per_mm_z: spm });
}

function promptCustomSpm() {
  const curX = latestState.steps_per_mm?.x || 200.0;
  const input = prompt("Steps/mm for X and Z lead screws (50 = T8×8, 100 = T8×4, 200 = T8×2):", curX.toFixed(1));
  if (input === null) return;
  const val = parseFloat(input);
  if (!isNaN(val) && val > 0) applyLeadPreset(0, val);
}

let liveStreamActive = true;
function toggleLiveStream() {
  liveStreamActive = !liveStreamActive;
  sendCmd("TOGGLE_LIVE_STREAM");
  const btn = $("toggleLiveBtn");
  const pill = $("liveCamPill");
  if (btn) btn.textContent = liveStreamActive ? "Pause" : "Resume";
  if (pill) {
    pill.dataset.live = liveStreamActive ? "1" : "0";
    pill.lastChild.textContent = liveStreamActive ? "LIVE" : "PAUSED";
  }
}

let liveCvEnabled = false;
function setCvButton() {
  const btn = $("toggleLiveCvBtn");
  if (!btn) return;
  btn.textContent = liveCvEnabled ? "CV: Full" : "CV: Eco";
  btn.classList.toggle("is-on", liveCvEnabled);
}
function toggleLiveCv() {
  liveCvEnabled = !liveCvEnabled;
  sendCmd("TOGGLE_LIVE_CV");
  setCvButton();
}
function toggleCamExpand(force) {
  const on = typeof force === "boolean" ? force : !document.body.classList.contains("cam-expanded");
  document.body.classList.toggle("cam-expanded", on);
}

const MASS_CLASS_LABEL = {
  low_infill: "Low infill",
  med_infill: "Med infill",
  high_infill: "High infill",
  aluminum: "Aluminum",
};

function setMassPill(text, cls) {
  const pill = $("massClassPill");
  if (!pill) return;
  pill.textContent = text;
  pill.className = `pill ${cls}`;
}

function highlightMassBin(cls) {
  document.querySelectorAll("#massBins span").forEach((s) => {
    s.classList.toggle("active", s.dataset.cls === cls);
  });
}

function sendTareCmd() {
  setText("liveMassReadout", "0.00");
  setMassPill("Tared", "pill-dim");
  highlightMassBin(null);
  sendCmd("TARE_SCALE");
}

function sendWeighCmd() {
  setText("liveMassReadout", "…");
  setMassPill("Sampling", "pill-accent");
  sendCmd("WEIGH_BLOCK");
}

function sendRawSerial() {
  const input = $("rawSerialInput");
  const line = input.value.trim();
  if (!line) return;
  sendCmd("RAW_SERIAL", { line });
  input.value = "";
}

function clearLogs() {
  const logEl = $("logConsole");
  if (logEl) logEl.textContent = "";
  lastLogSignature = "";
}

$("rawSerialInput").addEventListener("keydown", (e) => {
  if (e.key === "Enter") sendRawSerial();
});

function selectTab(name) {
  document.querySelectorAll(".tab").forEach((t) => t.classList.toggle("active", t.dataset.tab === name));
  document.querySelectorAll(".tab-pane").forEach((p) => p.classList.toggle("active", p.dataset.pane === name));
  document.querySelectorAll(".tab-actions").forEach((a) => a.classList.toggle("hidden", a.dataset.for !== name));
  if (name === "reports") fetchRecentReports();
}

// ============================================================================
//  UTILITY & TELEMETRY UI UPDATE
// ============================================================================
function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

function setText(id, text) {
  const el = $(id);
  if (el && el.textContent !== text) el.textContent = text;
}

function fmt3(x, y, z) {
  return `(${x.toFixed(1)}, ${y.toFixed(1)}, ${z.toFixed(1)})`;
}

function getCamHomedOrigin() {
  const cal = latestState.calibration || {};
  const ox = cal.cam_origin_x_mm !== undefined && cal.cam_origin_x_mm !== null ? Number(cal.cam_origin_x_mm) : CAM_HOMED_X;
  const oy = cal.cam_origin_y_mm !== undefined && cal.cam_origin_y_mm !== null ? Number(cal.cam_origin_y_mm) : CAM_HOMED_Y;
  return { x: ox, y: oy, calibrated: !!cal.origin_calibrated };
}

function motorCamToPlate(cx_m, cy_m) {
  const pt = latestState.calibration?.plate_transform;
  if (pt && pt.a00 !== undefined) {
    return {
      x: pt.a00 * cx_m + pt.a01 * cy_m + pt.tx,
      y: pt.a10 * cx_m + pt.a11 * cy_m + pt.ty,
    };
  }
  const o = getCamHomedOrigin();
  return { x: o.x + cx_m, y: o.y + cy_m };
}

function updateReadoutsFromMotor() {
  const o = getCamHomedOrigin();
  const camP = motorCamToPlate(motorX, motorY);
  const eeP = motorCamToPlate(motorX + EE_OFFSET_X, motorY + EE_OFFSET_Y);
  const eeH = motorCamToPlate(EE_OFFSET_X, EE_OFFSET_Y);

  setText("camPlateReadout", fmt3(camP.x, camP.y, CAM_HOMED_Z - motorZDown));
  setText("eePlateReadout", fmt3(eeP.x, eeP.y, EE_HOMED_Z - motorZDown));
  setText("camHomedOriginReadout", `${fmt3(o.x, o.y, CAM_HOMED_Z)} mm`);
  setText("eeHomedOriginReadout", `${fmt3(eeH.x, eeH.y, EE_HOMED_Z)} mm`);

  setText("motorXReadout", motorX.toFixed(1));
  setText("motorYReadout", motorY.toFixed(1));
  setText("motorZReadout", (motorZDown > 0.005 ? -motorZDown : 0.0).toFixed(2));

  $("motorXBar").style.width = `${clamp((motorX / STROKE_X) * 100, 0, 100)}%`;
  $("motorYBar").style.width = `${clamp((motorY / STROKE_Y) * 100, 0, 100)}%`;
  $("motorZBar").style.width = `${clamp((motorZDown / STROKE_Z_BOTTOM) * 100, 0, 100)}%`;
  setText("poseA", servoAngle.toFixed(1));
  setText("poseGrip", gripClosed ? "Closed" : "Open");
}

let lastLogSignature = "";
let lastBlocksSignature = "";

function updateUI(data, prevState) {
  const state = data.state || "IDLE";
  if (prevState && prevState !== state && (state === "COMPLETE" || state === "ERROR") && !replayActive) {
    setTimeout(fetchRecentReports, 800);
  }

  setText("stateReadout", state);
  const chip = $("stateChip");
  if (chip) chip.dataset.state = state;
  syncPauseResumeButton(state);

  const pose = data.gantry_pose || {};
  if (pose.x_mm !== undefined && pose.y_mm !== undefined) {
    const px = clamp(Number(pose.x_mm) || 0.0, 0, STROKE_X);
    const py = clamp(Number(pose.y_mm) || 0.0, 0, STROKE_Y);
    const pz = clamp(Number(pose.z_mm) || 0.0, 0, STROKE_Z_MAX);
    const pa = clamp(Number(pose.angle_deg) || 0.0, 0, 180);
    const pg = pose.grip_closed !== undefined ? !!pose.grip_closed : true;
    const hwSig = `${px.toFixed(2)}|${py.toFixed(2)}|${pz.toFixed(2)}|${pa.toFixed(1)}|${pg ? 1 : 0}`;
    const hwMoved = hwSig !== lastHwPoseSig;
    lastHwPoseSig = hwSig;

    // Only apply hardware telemetry if not actively previewing a local sandbox pose,
    // or immediately release sandbox preview whenever the physical hardware moves or runs a cycle.
    if (!sandboxPreviewActive || hwMoved || state !== "IDLE") {
      if (sandboxPreviewActive && (hwMoved || state !== "IDLE")) {
        sandboxPreviewActive = false;
        setSyncButtonVisible(false);
      }
      motorX = px;
      motorY = py;
      motorZDown = pz;
      servoAngle = pa;
      gripClosed = pg;
    }
  }
  syncSlidersFromState();
  updateReadoutsFromMotor();

  if (data.current_mass_g !== undefined && data.current_mass_g !== null) {
    setText("liveMassReadout", Number(data.current_mass_g).toFixed(2));
    const cls = (data.mass_class || "").toLowerCase();
    if (MASS_CLASS_LABEL[cls]) {
      setMassPill(MASS_CLASS_LABEL[cls], `pill-${cls}`);
      highlightMassBin(cls);
    } else {
      setMassPill("Measured", "pill-dim");
      highlightMassBin(null);
    }
  } else if (replayActive) {
    setText("liveMassReadout", "--.--");
    setMassPill("Standby", "pill-dim");
    highlightMassBin(null);
  }

  const cal = data.calibration || {};
  setText("calStatus", cal.calibrated ? "Calibrated" : "Default");
  const originEl = $("calOriginStatus");
  if (originEl) {
    const markers = Array.isArray(cal.markers_detected) ? cal.markers_detected : [];
    let labelText = "CAD nominal";
    if (cal.origin_calibrated) {
      if (markers.length >= 3) labelText = `${markers.length}-ArUco Ground Truth`;
      else if (markers.length === 2) labelText = "2-ArUco Similarity";
      else labelText = "1-ArUco Fallback";
    }
    originEl.textContent = labelText;
    originEl.className = `pill ${cal.origin_calibrated ? "pill-success" : "pill-dim"}`;
  }
  setText("calKx", (cal.kx_mm_per_px ?? 0.25).toFixed(4));
  setText("calKy", (cal.ky_mm_per_px ?? 0.25).toFixed(4));
  const skew = Number(cal.cam_skew_deg) || 0.0;
  setText("calCamSkew", `${skew >= 0 ? "+" : ""}${skew.toFixed(1)}°`);
  const skewEl = $("calSkewStatus");
  if (skewEl) {
    const comp = Math.abs(skew) > 0.05;
    skewEl.textContent = comp ? "Compensated" : "Nominal";
    skewEl.className = `pill ${comp ? "pill-success" : "pill-dim"}`;
  }
  const plateYaw = Number(cal.plate_yaw_deg) || 0.0;
  const rmsStr = cal.residual_rms_mm != null ? `${Number(cal.residual_rms_mm).toFixed(3)} mm` : "— mm";
  setText("calPlateFitReadout", `${plateYaw >= 0 ? "+" : ""}${plateYaw.toFixed(2)}° · ${rmsStr}`);
  if (data.steps_per_mm) setSpmTag(data.steps_per_mm.x, data.steps_per_mm.y);

  if (data.live_cv_enabled !== undefined) {
    liveCvEnabled = !!data.live_cv_enabled;
    setCvButton();
  }

  // Camera view: live JPEG from Pi when available, synthetic preview fallback when absent
  const imgEl = $("visionPreviewImg");
  const synthCanvas = $("synthCamCanvas");
  if (data.vision_frame_b64) {
    imgEl.src = `data:image/jpeg;base64,${data.vision_frame_b64}`;
    imgEl.style.display = "block";
    synthCanvas.style.display = "none";
  } else {
    imgEl.style.display = "none";
    synthCanvas.style.display = "block";
    renderSyntheticCamera(synthCanvas, data);
  }

  const fids = data.live_fiducials || [];
  const liveBlocks = data.live_blocks || [];
  const hudEl = $("liveDetectionText");
  if (hudEl) {
    if (fids.length === 0 && liveBlocks.length === 0) {
      setText("liveDetectionText", liveCvEnabled ? "No fiducials or blocks in view" : "Eco mode — enable CV: Full for detections");
      hudEl.title = "";
    } else {
      const parts = [];
      for (const f of fids) parts.push(`ArUco #${f.id} (${f.u_px}, ${f.v_px})${f.angle_deg !== undefined ? " " + f.angle_deg + "°" : ""}`);
      for (const b of liveBlocks) parts.push(`${String(b.color).toUpperCase()} (${b.u_px}, ${b.v_px}) ${b.angle_deg}°`);
      const txt = parts.join("  ·  ");
      setText("liveDetectionText", txt);
      hudEl.title = parts.join("\n");
    }
  }

  const blocks = data.blocks || [];
  const blocksSig = JSON.stringify(blocks);
  if (blocksSig !== lastBlocksSignature) {
    lastBlocksSignature = blocksSig;
    renderBlocksTable(blocks);
  }

  if (Array.isArray(data.logs) && data.logs.length > 0) {
    const sig = data.logs.length + "|" + data.logs[data.logs.length - 1];
    if (sig !== lastLogSignature) {
      lastLogSignature = sig;
      const logEl = $("logConsole");
      const pinned = logEl.scrollHeight - logEl.scrollTop - logEl.clientHeight < 24;
      logEl.textContent = data.logs.join("\n");
      if (pinned) logEl.scrollTop = logEl.scrollHeight;
    }
  }
}

function renderBlocksTable(blocks) {
  setText("blockCountBadge", String(blocks.length));
  const tbody = $("blocksTableBody");
  if (!tbody) return;
  if (blocks.length === 0) {
    tbody.innerHTML = `<tr><td colspan="6" class="empty-cell">No blocks cataloged yet.</td></tr>`;
    return;
  }
  tbody.innerHTML = blocks
    .map((b) => {
      const stacked = b.status === "STACKED";
      const mass = b.mass_g > 0 ? `${b.mass_g.toFixed(1)}g` : "—";
      return `
      <tr title="${b.mass_class || ""}">
        <td>#${b.id}</td>
        <td><span class="swatch" style="background:${COLOR_HEX[b.color] || "#94a3b8"}"></span>${b.color}</td>
        <td>${Number(b.x_mm).toFixed(0)}, ${Number(b.y_mm).toFixed(0)}</td>
        <td>${Number(b.angle_deg).toFixed(0)}°</td>
        <td>${mass}</td>
        <td><span class="pill ${stacked ? "pill-success" : "pill-dim"}">${b.status}</span></td>
      </tr>`;
    })
    .join("");
}

// ============================================================================
//  SYNTHETIC 640x480 EYE-IN-HAND CAMERA FALLBACK PREVIEW
// ============================================================================
function renderSyntheticCamera(canvas, data) {
  const c = canvas.getContext("2d");
  const W = canvas.width;
  const H = canvas.height;
  c.fillStyle = "#0a0d12";
  c.fillRect(0, 0, W, H);

  if (!liveStreamActive) {
    c.fillStyle = "#6c7787";
    c.font = "600 18px Inter, sans-serif";
    c.textAlign = "center";
    c.fillText("CAMERA STREAM PAUSED", W / 2, H / 2);
    return;
  }

  const camX = CAM_HOMED_X + drawX;
  const camY = CAM_HOMED_Y + drawY;
  const camZ = CAM_HOMED_Z - drawZ;
  const fovScale = Math.max(0.12, (camZ - CAL_PLANE_Z) / (CAM_HOMED_Z - CAL_PLANE_Z));
  const kx = 0.25 * fovScale;
  const ky = 0.25 * fovScale;

  const worldToPx = (wx, wy) => ({
    u: W / 2 - (wx - camX) / kx,
    v: H / 2 + (wy - camY) / ky,
  });

  // Outer plate
  const p0 = worldToPx(0, 0);
  const p1 = worldToPx(PLATE_W_X, PLATE_H_Y);
  c.fillStyle = "#1c2430";
  c.fillRect(Math.min(p0.u, p1.u), Math.min(p0.v, p1.v), Math.abs(p1.u - p0.u), Math.abs(p1.v - p0.v));

  // Inner course
  const c0 = worldToPx(COURSE_ORIGIN_X, COURSE_ORIGIN_Y);
  const c1 = worldToPx(COURSE_ORIGIN_X + COURSE_LEN_X, COURSE_ORIGIN_Y + COURSE_WID_Y);
  c.fillStyle = "#64748b";
  c.fillRect(Math.min(c0.u, c1.u), Math.min(c0.v, c1.v), Math.abs(c1.u - c0.u), Math.abs(c1.v - c0.v));

  // Aluminum pedestal
  const a0 = worldToPx(DROPOFF_X - ALUM_PLATFORM_SIZE / 2, DROPOFF_Y - ALUM_PLATFORM_SIZE / 2);
  const a1 = worldToPx(DROPOFF_X + ALUM_PLATFORM_SIZE / 2, DROPOFF_Y + ALUM_PLATFORM_SIZE / 2);
  c.fillStyle = "#94a3b8";
  c.fillRect(Math.min(a0.u, a1.u), Math.min(a0.v, a1.v), Math.abs(a1.u - a0.u), Math.abs(a1.v - a0.v));

  // Corner fiducials
  for (const cf of CORNER_FIDUCIALS) {
    const fp = worldToPx(cf.x, cf.y);
    const fs = FIDUCIAL_SIZE / kx;
    c.fillStyle = "#f1f5f9";
    c.fillRect(fp.u - fs / 2, fp.v - fs / 2, fs, fs);
    c.fillStyle = "#0b0e13";
    c.fillRect(fp.u - fs * 0.36, fp.v - fs * 0.36, fs * 0.72, fs * 0.72);
    if (liveCvEnabled) {
      c.strokeStyle = "#22c55e";
      c.lineWidth = 2;
      c.strokeRect(fp.u - fs / 2 - 3, fp.v - fs / 2 - 3, fs + 6, fs + 6);
    }
  }

  // Known/Replay blocks
  const blocks = (data.world_blocks && data.world_blocks.length) ? data.world_blocks : (data.blocks || []);
  for (const b of blocks) {
    if (b.id === data.held_block_id) continue;
    const bp = worldToPx(b.x_mm, b.y_mm);
    const bs = BLOCK_SIZE / kx;
    c.save();
    c.translate(bp.u, bp.v);
    c.rotate(((Number(b.angle_deg) || 0) * Math.PI) / 180);
    c.fillStyle = COLOR_HEX[b.color] || "#94a3b8";
    c.fillRect(-bs / 2, -bs / 2, bs, bs);
    if (liveCvEnabled) {
      c.strokeStyle = "#3ccf91";
      c.lineWidth = 2;
      c.strokeRect(-bs / 2 - 3, -bs / 2 - 3, bs + 6, bs + 6);
    }
    c.restore();
  }

  c.strokeStyle = "rgba(79, 156, 249, 0.75)";
  c.lineWidth = 1.5;
  c.beginPath();
  c.moveTo(W / 2 - 18, H / 2); c.lineTo(W / 2 + 18, H / 2);
  c.moveTo(W / 2, H / 2 - 18); c.lineTo(W / 2, H / 2 + 18);
  c.stroke();

  c.fillStyle = "rgba(11, 14, 19, 0.72)";
  c.fillRect(8, 8, 224, 24);
  c.fillStyle = "#e6ebf2";
  c.font = "600 12px Consolas, monospace";
  c.textAlign = "left";
  c.fillText(`SYNTH CAM (${camX.toFixed(1)}, ${camY.toFixed(1)}, ${camZ.toFixed(1)})`, 14, 24);
}

// ============================================================================
//  THREE.JS 3D CAD DIGITAL TWIN ENGINE
// ============================================================================
const viewport3dWrap = $("viewport3dWrap");
const twin3dCanvas = $("twin3dCanvas");

let renderer3d = null;
let scene3d = null;
let orbitCam = null;
let eyeCam = null;
let activeCam3d = null;
let controls3d = null;

// Kinematic stage groups
let worldGroup = null;
let xStageGroup = null;
let zStageGroup = null;
let yStageGroup = null;
let eeStageGroup = null;

// Sub-parts for animation & inspection
let fovConeGroup = null;
let reachBoundsGroup = null;
let trailLine = null;
let trailPoints = [];
let blockMeshPool = new Map();
let customCadMeshes = [];
let bulkPartAssignments = [];

// Compiled full Onshape V3 Gantry CAD state
let compiledCadLoaded = false;
let compiledCadActive = false;
let compiledCadMeta = {
  cad_motor_x_mm: 14.4,
  cad_motor_y_mm: 70.0,
  ee_pivot_three: [-96.6, 106.5, -1.5],
  cam_local_three: [-95.6, 120.5, -58.5],
};
let compiledStageGroups = {};
let compiledPlungerGroup = null;

const stageGroupsMap = {};

// Plate (X, Y, Z) -> Three.js World (X, Y_up, Z_back)
function plateToThree(px, py, pz) {
  return new THREE.Vector3(px - 142.0, pz, -(py - 60.0));
}

function createBlockTopTexture(id, colorName) {
  const c = document.createElement("canvas");
  c.width = 128;
  c.height = 128;
  const g = c.getContext("2d");
  g.fillStyle = COLOR_HEX[colorName] || "#94a3b8";
  g.fillRect(0, 0, 128, 128);
  g.strokeStyle = "rgba(0,0,0,0.18)";
  g.lineWidth = 6;
  g.strokeRect(3, 3, 122, 122);
  g.fillStyle = LIGHT_BLOCKS.has(colorName) ? "#0b0e13" : "#ffffff";
  g.font = "700 44px Consolas, monospace";
  g.textAlign = "center";
  g.textBaseline = "middle";
  g.fillText(`#${id}`, 64, 66);
  return new THREE.CanvasTexture(c);
}

function init3DScene() {
  if (typeof THREE === "undefined") {
    setText("meshStatusText", "Three.js unavailable");
    return;
  }

  scene3d = new THREE.Scene();
  scene3d.background = new THREE.Color("#0b0f16");
  scene3d.fog = new THREE.FogExp2("#0b0f16", 0.00075);

  const w = Math.max(300, viewport3dWrap.clientWidth);
  const h = Math.max(240, viewport3dWrap.clientHeight);

  orbitCam = new THREE.PerspectiveCamera(40, w / h, 1, 2600);
  eyeCam = new THREE.PerspectiveCamera(53, w / h, 2, 1200);
  activeCam3d = orbitCam;

  renderer3d = new THREE.WebGLRenderer({ canvas: twin3dCanvas, antialias: true, powerPreference: "high-performance" });
  renderer3d.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer3d.setSize(w, h, false);
  renderer3d.shadowMap.enabled = true;
  renderer3d.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer3d.outputEncoding = THREE.sRGBEncoding;
  renderer3d.toneMapping = THREE.ACESFilmicToneMapping;
  renderer3d.toneMappingExposure = 0.92;

  controls3d = new THREE.OrbitControls(orbitCam, twin3dCanvas);
  controls3d.enableDamping = true;
  controls3d.dampingFactor = 0.08;
  controls3d.minDistance = 90;
  controls3d.maxDistance = 900;

  // Studio Lights
  const hemi = new THREE.HemisphereLight("#dbeafe", "#1e293b", 0.62);
  scene3d.add(hemi);

  const keyLight = new THREE.DirectionalLight("#ffffff", 0.88);
  keyLight.position.set(220, 380, -240);
  keyLight.castShadow = true;
  keyLight.shadow.mapSize.set(2048, 2048);
  keyLight.shadow.camera.near = 50;
  keyLight.shadow.camera.far = 950;
  keyLight.shadow.camera.left = -260;
  keyLight.shadow.camera.right = 260;
  keyLight.shadow.camera.top = 260;
  keyLight.shadow.camera.bottom = -260;
  keyLight.shadow.bias = -0.0005;
  scene3d.add(keyLight);

  const fillLight = new THREE.DirectionalLight("#93c5fd", 0.45);
  fillLight.position.set(-240, 190, 220);
  scene3d.add(fillLight);

  const grid = new THREE.GridHelper(900, 45, "#263244", "#151c27");
  grid.position.set(0, -4.0, 0);
  scene3d.add(grid);

  // ==========================================================================
  //  STAGE 0: WORLD / COURSE / STATIC BASE
  // ==========================================================================
  worldGroup = new THREE.Group();
  scene3d.add(worldGroup);
  stageGroupsMap.world = worldGroup;

  // Reach Envelopes (EE + Cam 3D wireframes)
  reachBoundsGroup = new THREE.Group();
  const eeReachGeo = new THREE.EdgesGeometry(new THREE.BoxGeometry(STROKE_X, STROKE_Z_BOTTOM, STROKE_Y));
  const eeReachBox = new THREE.LineSegments(eeReachGeo, new THREE.LineBasicMaterial({ color: "#3ccf91", transparent: true, opacity: 0.35 }));
  eeReachBox.position.copy(plateToThree(EE_HOMED_X + STROKE_X / 2, EE_HOMED_Y + STROKE_Y / 2, COURSE_FLOOR_Z + STROKE_Z_BOTTOM / 2));
  reachBoundsGroup.add(eeReachBox);

  const camReachGeo = new THREE.EdgesGeometry(new THREE.BoxGeometry(STROKE_X, 2, STROKE_Y));
  const camReachBox = new THREE.LineSegments(camReachGeo, new THREE.LineBasicMaterial({ color: "#4f9cf9", transparent: true, opacity: 0.35 }));
  camReachBox.position.copy(plateToThree(CAM_HOMED_X + STROKE_X / 2, CAM_HOMED_Y + STROKE_Y / 2, COURSE_FLOOR_Z + 1));
  reachBoundsGroup.add(camReachBox);
  worldGroup.add(reachBoundsGroup);

  // Toolpath 3D trail line
  const trailGeo = new THREE.BufferGeometry();
  trailLine = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: "#3ccf91", transparent: true, opacity: 0.8 }));
  worldGroup.add(trailLine);

  // ==========================================================================
  //  STAGES 1-4: X-SLIDER/Z-TOWER -> Z-SADDLE/Y-BOOM -> Y-CARRIAGE -> EE_V6
  // ==========================================================================
  xStageGroup = new THREE.Group();
  xStageGroup.position.set(-compiledCadMeta.cad_motor_x_mm, 0, 0);
  scene3d.add(xStageGroup);
  stageGroupsMap.xStage = xStageGroup;

  zStageGroup = new THREE.Group();
  xStageGroup.add(zStageGroup);
  stageGroupsMap.zStage = zStageGroup;

  yStageGroup = new THREE.Group();
  yStageGroup.position.set(0, 0, compiledCadMeta.cad_motor_y_mm);
  zStageGroup.add(yStageGroup);
  stageGroupsMap.yStage = yStageGroup;

  // Eye-in-hand virtual camera & 3D FOV frustum anchored in physical Load Cell Bracket nose pocket
  const camPos = compiledCadMeta.cam_local_three || [-95.6, 120.5, -58.5];
  eyeCam.position.set(camPos[0], camPos[1] - 0.5, camPos[2]);
  eyeCam.up.set(0, 0, -1);
  eyeCam.lookAt(camPos[0], 0, camPos[2]);
  yStageGroup.add(eyeCam);

  fovConeGroup = new THREE.Group();
  fovConeGroup.position.set(camPos[0], camPos[1], camPos[2]);
  yStageGroup.add(fovConeGroup);
  rebuildFovCone(camPos[1]);

  // Stage 4: Rotating EE_V6 Solenoid Gripper Assembly
  eeStageGroup = new THREE.Group();
  const piv = compiledCadMeta.ee_pivot_three || [-96.6, 106.5, -1.5];
  eeStageGroup.position.set(piv[0], piv[1], piv[2]);
  yStageGroup.add(eeStageGroup);
  stageGroupsMap.eeStage = eeStageGroup;

  // Green EE center crosshair indicator on top
  const eeMarker = new THREE.Mesh(
    new THREE.RingGeometry(2.2, 3.4, 24),
    new THREE.MeshBasicMaterial({ color: "#3ccf91", side: THREE.DoubleSide })
  );
  eeMarker.rotation.x = -Math.PI / 2;
  eeMarker.position.set(0, 0.2, 0);
  eeStageGroup.add(eeMarker);

  // Load pre-compiled 53-draw-call full Onshape V3 Gantry assembly
  loadCompiledGantryCad();

  setCameraPreset("iso");
}

function applyCadModeTransforms() {
  if (!yStageGroup || !eeStageGroup || !eyeCam || !fovConeGroup) return;
  for (const g of Object.values(compiledStageGroups)) g.visible = compiledCadActive;
  const camPos = compiledCadMeta.cam_local_three || [-95.6, 120.5, -58.5];
  eyeCam.position.set(camPos[0], camPos[1] - 0.5, camPos[2]);
  eyeCam.up.set(0, 0, -1);
  eyeCam.lookAt(camPos[0], 0, camPos[2]);
  fovConeGroup.position.set(camPos[0], camPos[1], camPos[2]);
  rebuildFovCone(camPos[1] - drawZ);
  const piv = compiledCadMeta.ee_pivot_three || [-96.6, 106.5, -1.5];
  eeStageGroup.position.set(piv[0], piv[1], piv[2]);
}

async function loadCompiledGantryCad() {
  try {
    const [metaRes, binRes] = await Promise.all([
      fetch("assets/v3_gantry_compiled.json?v=2"),
      fetch("assets/v3_gantry_compiled.bin?v=2"),
    ]);
    if (!metaRes.ok || !binRes.ok) throw new Error("Compiled CAD not found");
    const manifest = await metaRes.json();
    const binBuf = await binRes.arrayBuffer();
    compiledCadMeta = manifest;

    // Build shared PBR materials with crisp roughness so saturated cyan & pink PLA colors pop cleanly
    const mats = manifest.materials.map((m) => {
      const lum = (m.color[0] + m.color[1] + m.color[2]) / 3;
      const isSteel = Math.abs(m.color[0] - m.color[2]) < 0.04 && lum > 0.35 && lum < 0.78;
      return new THREE.MeshStandardMaterial({
        color: new THREE.Color(m.color[0], m.color[1], m.color[2]),
        roughness: isSteel ? 0.32 : 0.38,
        metalness: isSteel ? 0.65 : 0.06,
      });
    });

    compiledStageGroups = {
      world: new THREE.Group(),
      xStage: new THREE.Group(),
      zStage: new THREE.Group(),
      yStage: new THREE.Group(),
      eeStage: new THREE.Group(),
      plunger: new THREE.Group(),
    };
    worldGroup.add(compiledStageGroups.world);
    xStageGroup.add(compiledStageGroups.xStage);
    zStageGroup.add(compiledStageGroups.zStage);
    yStageGroup.add(compiledStageGroups.yStage);
    eeStageGroup.add(compiledStageGroups.eeStage);
    compiledStageGroups.eeStage.add(compiledStageGroups.plunger);
    compiledPlungerGroup = compiledStageGroups.plunger;

    for (const dc of manifest.draw_calls) {
      const posArr = new Float32Array(binBuf, dc.posOffset, dc.vertexCount * 3);
      const nrmArr = new Float32Array(binBuf, dc.nrmOffset, dc.vertexCount * 3);
      const idxArr = new Uint32Array(binBuf, dc.idxOffset, dc.idxCount);

      const geo = new THREE.BufferGeometry();
      geo.setAttribute("position", new THREE.BufferAttribute(posArr, 3));
      geo.setAttribute("normal", new THREE.BufferAttribute(nrmArr, 3));
      geo.setIndex(new THREE.BufferAttribute(idxArr, 1));

      const mesh = new THREE.Mesh(geo, mats[dc.material] || mats[0]);
      mesh.castShadow = dc.triangleCount >= 120;
      mesh.receiveShadow = dc.stage === "world" || dc.triangleCount >= 120;
      (compiledStageGroups[dc.stage] || compiledStageGroups.world).add(mesh);
    }

    compiledCadLoaded = true;
    compiledCadActive = true;
    applyCadModeTransforms();

    const st = manifest.stats;
    setText("meshStatusText", `Onshape CAD: ${st.compiled_draw_calls} draws · ${(st.compiled_triangles / 1000).toFixed(1)}k tris`);
  } catch (err) {
    compiledCadActive = false;
    setText("meshStatusText", `CAD load error: ${err.message}`);
    console.warn("Compiled CAD load skipped:", err);
  }
}

function rebuildFovCone(camZ) {
  if (!fovConeGroup) return;
  while (fovConeGroup.children.length > 0) {
    const ch = fovConeGroup.children[0];
    fovConeGroup.remove(ch);
    if (ch.geometry) ch.geometry.dispose();
  }
  const camHomeY = compiledCadMeta?.cam_local_three?.[1] ?? 120.5;
  const dropH = Math.max(12, camZ - (COURSE_FLOOR_Z + 0.5));
  const calHomeDrop = Math.max(12, camHomeY - CAL_PLANE_Z);
  const halfW = ((FRAME_W_PX * 0.25) / 2) * (dropH / calHomeDrop);
  const halfH = ((FRAME_H_PX * 0.25) / 2) * (dropH / calHomeDrop);

  const pts = [
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(-halfW, -dropH, -halfH),
    new THREE.Vector3(halfW, -dropH, -halfH),
    new THREE.Vector3(halfW, -dropH, halfH),
    new THREE.Vector3(-halfW, -dropH, halfH),
  ];
  const indices = [
    0, 1, 2,
    0, 2, 3,
    0, 3, 4,
    0, 4, 1,
  ];
  const geo = new THREE.BufferGeometry();
  const flat = [];
  for (const p of pts) flat.push(p.x, p.y, p.z);
  geo.setAttribute("position", new THREE.Float32BufferAttribute(flat, 3));
  geo.setIndex(indices);
  geo.computeVertexNormals();

  const coneMesh = new THREE.Mesh(
    geo,
    new THREE.MeshBasicMaterial({ color: "#4f9cf9", transparent: true, opacity: 0.08, side: THREE.DoubleSide, depthWrite: false })
  );
  fovConeGroup.add(coneMesh);

  const wirePts = [
    pts[0], pts[1], pts[0], pts[2], pts[0], pts[3], pts[0], pts[4],
    pts[1], pts[2], pts[2], pts[3], pts[3], pts[4], pts[4], pts[1],
    pts[0], new THREE.Vector3(0, -dropH, 0),
    new THREE.Vector3(-7, -dropH, 0), new THREE.Vector3(7, -dropH, 0),
    new THREE.Vector3(0, -dropH, -7), new THREE.Vector3(0, -dropH, 7),
  ];
  const lineGeo = new THREE.BufferGeometry().setFromPoints(wirePts);
  fovConeGroup.add(new THREE.LineSegments(lineGeo, new THREE.LineBasicMaterial({ color: "#4f9cf9", transparent: true, opacity: 0.45 })));
}

// ============================================================================
//  WORLD BLOCKS SYNCHRONIZATION IN 3D
// ============================================================================
function sync3DBlocks(data) {
  if (!worldGroup || !eeStageGroup) return;
  const blocks = (data.world_blocks && data.world_blocks.length) ? data.world_blocks : (data.blocks || []);
  const activeIds = new Set();

  for (const b of blocks) {
    activeIds.add(b.id);
    let mesh = blockMeshPool.get(b.id);
    if (!mesh) {
      const topTex = createBlockTopTexture(b.id, b.color);
      const sideMat = new THREE.MeshStandardMaterial({
        color: COLOR_HEX[b.color] || "#94a3b8",
        roughness: b.color === "aluminum" ? 0.25 : 0.45,
        metalness: b.color === "aluminum" ? 0.8 : 0.05,
      });
      const topMat = new THREE.MeshStandardMaterial({
        map: topTex,
        roughness: b.color === "aluminum" ? 0.25 : 0.45,
        metalness: b.color === "aluminum" ? 0.8 : 0.05,
      });
      mesh = new THREE.Mesh(
        new THREE.BoxGeometry(BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE),
        [sideMat, sideMat, topMat, sideMat, sideMat, sideMat]
      );
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      blockMeshPool.set(b.id, mesh);
      worldGroup.add(mesh);
    }

    if (b.id === data.held_block_id || b.status === "HELD") {
      if (mesh.parent !== eeStageGroup) {
        eeStageGroup.add(mesh);
      }
      mesh.position.set(0, -19.8, 0);
      mesh.rotation.set(0, 0, 0);
    } else {
      if (mesh.parent !== worldGroup) {
        worldGroup.add(mesh);
      }
      const zFloor = b.z_mm !== undefined ? Number(b.z_mm) : (b.status === "STACKED" && b.mass_class === "aluminum" ? DROPOFF_Z_UP : COURSE_FLOOR_Z);
      mesh.position.copy(plateToThree(Number(b.x_mm), Number(b.y_mm), zFloor + BLOCK_SIZE / 2));
      mesh.rotation.set(0, ((Number(b.angle_deg) || 0) * Math.PI) / 180, 0);
    }
  }

  for (const [id, mesh] of blockMeshPool.entries()) {
    if (!activeIds.has(id)) {
      if (mesh.parent) mesh.parent.remove(mesh);
      blockMeshPool.delete(id);
    }
  }
}

// ============================================================================
//  CAMERA PRESETS, LAYERS, HOT-SWAP CAD IMPORT
// ============================================================================
function setTwinMode(mode) {
  twinMode = mode;
  $("twinStage").dataset.mode = mode;
  document.querySelectorAll(".mode-btn").forEach((b) => b.classList.toggle("active", b.dataset.mode === mode));
  $("camPresetsBar").style.visibility = mode === "2d" ? "hidden" : "visible";
  resizeAll();
}

function setCameraPreset(preset) {
  cameraPreset = preset;
  document.querySelectorAll(".preset-chip").forEach((b) => b.classList.toggle("active", b.dataset.cam === preset));

  const reticle = $("eyeInHandReticle");
  if (reticle) reticle.classList.toggle("hidden", preset !== "eyeinhand");

  if (!orbitCam || !controls3d) return;

  if (preset === "eyeinhand") {
    activeCam3d = eyeCam;
    setText("hudCamLabel", "Eye-in-Hand Optical POV · V3 Bracket Lens");
    return;
  }

  activeCam3d = orbitCam;
  orbitCam.up.set(0, 1, 0);

  if (preset === "iso") {
    orbitCam.position.set(205, 225, -245);
    controls3d.target.set(0, 48, 0);
    setText("hudCamLabel", "Orbit · Isometric (V3 Gantry)");
  } else if (preset === "top") {
    orbitCam.up.set(0, 0, -1);
    orbitCam.position.set(-4, 520, 22);
    controls3d.target.set(-4, 0, 22);
    setText("hudCamLabel", "Top-Down Orthographic Alignment");
  } else if (preset === "front") {
    orbitCam.position.set(-4, 105, -395);
    controls3d.target.set(-4, 72, 10);
    setText("hudCamLabel", "Front Elevation");
  } else if (preset === "side") {
    orbitCam.position.set(430, 95, 32);
    controls3d.target.set(0, 82, 32);
    setText("hudCamLabel", "Right Side Profile");
  } else if (preset === "follow") {
    setText("hudCamLabel", "Follow End-Effector (EE_V6)");
  }
  controls3d.update();
}

function toggleLayer(name) {
  layers[name] = !layers[name];
  const btnMap = { fov: "layerFovBtn", trail: "layerTrailBtn", reach: "layerReachBtn", wireframe: "layerWireBtn" };
  const btn = $(btnMap[name]);
  if (btn) btn.classList.toggle("active", layers[name]);

  if (fovConeGroup) fovConeGroup.visible = layers.fov;
  if (reachBoundsGroup) reachBoundsGroup.visible = layers.reach;
  if (trailLine) trailLine.visible = layers.trail;

  if (name === "wireframe" && scene3d) {
    scene3d.traverse((obj) => {
      if (obj.isMesh && obj.material && !Array.isArray(obj.material)) {
        obj.material.wireframe = layers.wireframe;
      }
    });
  }
}

function clearTrail() {
  trailPoints = [];
  if (trailLine) {
    trailLine.geometry.setFromPoints([]);
  }
}

function updateTrail() {
  if (!trailLine || !layers.trail) return;
  const eeX = EE_HOMED_X + drawX;
  const eeY = EE_HOMED_Y + drawY;
  const eeZ = EE_HOMED_Z - drawZ;
  const p = plateToThree(eeX, eeY, eeZ);
  const last = trailPoints[trailPoints.length - 1];
  if (!last || last.distanceTo(p) > 2.2) {
    trailPoints.push(p);
    if (trailPoints.length > 400) trailPoints.shift();
    trailLine.geometry.setFromPoints(trailPoints);
  }
}

// Hot-Swap CAD & Bulk Onshape Assembly Auto-Mating Modal
function openCadImportModal() {
  $("cadImportModal").style.display = "flex";
}
function closeCadImportModal() {
  $("cadImportModal").style.display = "none";
}

function clearCustomCadMeshes() {
  for (const m of customCadMeshes) {
    if (m.parent) m.parent.remove(m);
  }
  customCadMeshes = [];
  bulkPartAssignments = [];
  compiledCadActive = compiledCadLoaded;
  applyCadModeTransforms();
  const wrap = $("bulkPartsWrap");
  if (wrap) wrap.classList.add("hidden");
  const tbody = $("bulkPartsBody");
  if (tbody) tbody.innerHTML = "";
  setText("cadImportStatus", "Reset to calibrated Onshape CAD assembly.");
}

function classifyBulkPartStage(name, plateCx, plateCy, plateCz) {
  const n = String(name || "").toLowerCase();
  if (/endeffector|coil_|spring_|circlip|plunger|microservo horn|mg90servo_shaft|solenoid|jf-0530/.test(n)) return "eeStage";
  if (/load cell bracket|5 kg load cell|belt tensioner|mg90servo_body|servo/.test(n)) return "yStage";
  if (/y-axis connector|y-axis arm|pulley cap|belt pulley|boom|saddle|idler/.test(n)) return "zStage";
  if (/x-axis slider|z-axis pillar|z-motor|logo -/.test(n)) return "xStage";
  if (/baseplate|default course|platform|fiducial|x-axis cover|x-axis foot|8 mm linear shaft|pin/.test(n)) return "world";

  if (/12 mm linear rail/.test(n)) return plateCy < -15 ? "xStage" : "zStage";
  if (plateCz >= 65 && plateCz <= 112 && plateCy >= -15 && plateCx >= 10) return "eeStage";
  if (plateCz >= 104 && plateCz <= 148 && plateCy >= -15 && plateCx >= 0) return "yStage";
  if (plateCz >= 95 && plateCz <= 145 && (plateCx < 5 || plateCy < -15)) return "zStage";
  if (plateCz > 25 && plateCy < -15 && plateCx >= 5) return "xStage";
  return "world";
}

function mergeOccurrenceMeshesByMaterial(rootNode, worldTransform) {
  const byMat = new Map();
  rootNode.traverse((child) => {
    if (!child.isMesh || !child.geometry) return;
    const geo = child.geometry;
    const posAttr = geo.attributes.position;
    if (!posAttr) return;
    child.updateMatrixWorld(true);
    const mat4 = new THREE.Matrix4().multiplyMatrices(worldTransform, child.matrixWorld);
    const nrmMat = new THREE.Matrix3().getNormalMatrix(mat4);
    const matKey = child.material?.uuid || "default";
    if (!byMat.has(matKey)) {
      byMat.set(matKey, { material: child.material, pos: [], nrm: [], idx: [], vOff: 0 });
    }
    const bucket = byMat.get(matKey);
    const vCount = posAttr.count;
    const nrmAttr = geo.attributes.normal;
    const tmpV = new THREE.Vector3();
    const tmpN = new THREE.Vector3();
    for (let i = 0; i < vCount; i++) {
      tmpV.fromBufferAttribute(posAttr, i).applyMatrix4(mat4);
      bucket.pos.push(tmpV.x, tmpV.y, tmpV.z);
      if (nrmAttr) {
        tmpN.fromBufferAttribute(nrmAttr, i).applyMatrix3(nrmMat).normalize();
        bucket.nrm.push(tmpN.x, tmpN.y, tmpN.z);
      } else {
        bucket.nrm.push(0, 1, 0);
      }
    }
    if (geo.index) {
      const idxArr = geo.index.array;
      for (let i = 0; i < idxArr.length; i++) bucket.idx.push(idxArr[i] + bucket.vOff);
    } else {
      for (let i = 0; i < vCount; i++) bucket.idx.push(i + bucket.vOff);
    }
    bucket.vOff += vCount;
  });

  const mergedGroup = new THREE.Group();
  mergedGroup.name = rootNode.name;
  for (const b of byMat.values()) {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.Float32BufferAttribute(b.pos, 3));
    g.setAttribute("normal", new THREE.Float32BufferAttribute(b.nrm, 3));
    g.setIndex(b.idx);
    const m = new THREE.Mesh(g, b.material);
    m.castShadow = b.idx.length >= 360;
    m.receiveShadow = true;
    mergedGroup.add(m);
  }
  return mergedGroup;
}

function reassignBulkPartStage(idx, newStage) {
  const rec = bulkPartAssignments[idx];
  if (!rec) return;
  const target = stageGroupsMap[newStage] || worldGroup;
  rec.mesh.updateMatrixWorld(true);
  target.attach(rec.mesh);
  rec.stage = newStage;
}

function renderBulkPartsTable() {
  const wrap = $("bulkPartsWrap");
  const tbody = $("bulkPartsBody");
  if (!wrap || !tbody) return;
  if (bulkPartAssignments.length === 0) {
    wrap.classList.add("hidden");
    return;
  }
  wrap.classList.remove("hidden");
  const stageOpts = [
    ["world", "Stage 0 · Base/Course"],
    ["xStage", "Stage 1 · X-Tower"],
    ["zStage", "Stage 2 · Z+Y Boom"],
    ["yStage", "Stage 3 · Y-Carriage"],
    ["eeStage", "Stage 4 · EE_V6"],
    ["hidden", "Hidden"],
  ];
  tbody.innerHTML = bulkPartAssignments.map((rec, idx) => `
    <tr>
      <td>${rec.name}</td>
      <td>(${rec.cx.toFixed(0)}, ${rec.cy.toFixed(0)}, ${rec.cz.toFixed(0)})</td>
      <td>
        <select class="bulk-stage-sel" onchange="onBulkStageSelect(${idx}, this.value)">
          ${stageOpts.map(([k, lbl]) => `<option value="${k}" ${rec.stage === k ? "selected" : ""}>${lbl}</option>`).join("")}
        </select>
      </td>
    </tr>`).join("");
}

function onBulkStageSelect(idx, val) {
  const rec = bulkPartAssignments[idx];
  if (!rec) return;
  if (val === "hidden") {
    rec.mesh.visible = false;
    rec.stage = "hidden";
    return;
  }
  rec.mesh.visible = true;
  reassignBulkPartStage(idx, val);
}

async function handleCadFileSelect(e) {
  const file = e.target.files?.[0];
  if (!file) return;
  const stageMode = $("cadStageSelect").value;
  const unitVal = $("cadUnitScale").value || "1000:zup";
  const [scaleStr, upAxis] = unitVal.split(":");
  const unitScale = Number(scaleStr) || 1;
  const ext = file.name.split(".").pop().toLowerCase();

  setText("cadImportStatus", `Parsing ${file.name}…`);
  try {
    const buf = await file.arrayBuffer();
    let obj3d = null;
    if (ext === "stl" && typeof THREE.STLLoader !== "undefined") {
      const geo = new THREE.STLLoader().parse(buf);
      geo.computeVertexNormals();
      obj3d = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: "#93c5fd", roughness: 0.4, metalness: 0.2 }));
    } else if (ext === "3mf" && typeof THREE.ThreeMFLoader !== "undefined") {
      obj3d = new THREE.ThreeMFLoader().parse(buf);
    } else if ((ext === "glb" || ext === "gltf") && typeof THREE.GLTFLoader !== "undefined") {
      const gltf = await new Promise((resolve, reject) => new THREE.GLTFLoader().parse(buf, "", resolve, reject));
      obj3d = gltf.scene;
    } else {
      throw new Error(`Unsupported or missing loader for .${ext}`);
    }

    const wrapper = new THREE.Group();
    wrapper.add(obj3d);
    obj3d.scale.setScalar(unitScale);

    if (ext === "glb" || ext === "gltf") {
      obj3d.rotation.y = Math.PI;
      obj3d.position.set(0, 0, -16.2);
    } else if (upAxis === "zup") {
      obj3d.rotation.order = "YXZ";
      obj3d.rotation.set(-Math.PI / 2, Math.PI, 0);
      obj3d.position.set(0, 0, -16.2);
    }
    wrapper.updateMatrixWorld(true);

    if (stageMode === "auto") {
      compiledCadActive = false;
      for (const g of Object.values(compiledStageGroups)) g.visible = false;

      const invWorld = new THREE.Matrix4();
      const occUnits = [];
      const rootNode = (obj3d.children.length === 1 && obj3d.children[0].children.length > 3) ? obj3d.children[0] : obj3d;
      for (const ch of rootNode.children) {
        if (/endeffector/i.test(ch.name)) {
          for (const sub of ch.children) occUnits.push(sub);
        } else {
          occUnits.push(ch);
        }
      }
      if (occUnits.length === 0) occUnits.push(rootNode);

      bulkPartAssignments = [];
      for (let i = 0; i < occUnits.length; i++) {
        const occ = occUnits[i];
        const merged = mergeOccurrenceMeshesByMaterial(occ, invWorld);
        if (merged.children.length === 0) continue;

        const mb = new THREE.Box3().setFromObject(merged);
        const mc = new THREE.Vector3();
        mb.getCenter(mc);
        const plateCx = mc.x + 142.0;
        const plateCy = 60.0 - mc.z;
        const plateCz = mc.y;
        const partName = occ.name || `Part ${i + 1}`;
        const assigned = classifyBulkPartStage(partName, plateCx, plateCy, plateCz);
        const target = stageGroupsMap[assigned] || worldGroup;

        if (assigned === "eeStage") {
          const piv = compiledCadMeta.ee_pivot_three;
          merged.position.set(-piv[0], -piv[1], -piv[2]);
        }
        target.add(merged);
        customCadMeshes.push(merged);
        bulkPartAssignments.push({ name: partName, cx: plateCx, cy: plateCy, cz: plateCz, stage: assigned, mesh: merged });
      }

      renderBulkPartsTable();
      setText("cadImportStatus", `Auto-mated & merged ${bulkPartAssignments.length} occurrences from ${file.name}!`);
    } else {
      const targetStage = stageGroupsMap[stageMode] || yStageGroup;
      targetStage.add(wrapper);
      customCadMeshes.push(wrapper);
      setText("cadImportStatus", `Attached ${file.name} (${unitScale}×) to ${stageMode}.`);
    }
  } catch (err) {
    setText("cadImportStatus", `Import error: ${err.message}`);
  }
  e.target.value = "";
}

// ============================================================================
//  2D TACTICAL MAP RENDERER (Full production drawXYView + drawZView)
// ============================================================================
const VIEW_MIN_X = -14.0;
const VIEW_MAX_X = 298.0;
const VIEW_MIN_Y = -56.0;
const VIEW_MAX_Y = 196.0;
const Z_VIEW_MAX = 145.0;
const L = { w: 0, h: 0, dpr: 1, scale: 1, x0: 0, y0: 0, zx: 0, zy: 0, zw: 0, zh: 0 };

function computeLayout() {
  const rect = canvasWrap.getBoundingClientRect();
  if (rect.width < 20 || rect.height < 20) return;
  const dpr = window.devicePixelRatio || 1;
  L.w = Math.max(160, Math.floor(rect.width));
  L.h = Math.max(160, Math.floor(rect.height));
  L.dpr = dpr;
  courseCanvas.width = Math.round(L.w * dpr);
  courseCanvas.height = Math.round(L.h * dpr);

  const pad = L.w < 500 ? 12 : 16;
  const axisPad = 20;
  L.zw = clamp(Math.round(L.w * 0.22), 86, 175);
  const gap = L.w < 500 ? 10 : 16;
  const xyW = Math.max(60, L.w - pad * 2 - L.zw - gap - axisPad);
  const xyH = Math.max(60, L.h - pad * 2 - axisPad);

  const rangeX = VIEW_MAX_X - VIEW_MIN_X;
  const rangeY = VIEW_MAX_Y - VIEW_MIN_Y;
  L.scale = Math.max(0.1, Math.min(xyW / rangeX, xyH / rangeY));
  const plotW = rangeX * L.scale;
  const plotH = rangeY * L.scale;

  L.x0 = pad + axisPad + (xyW - plotW) / 2;
  L.y0 = pad + (xyH + plotH) / 2;
  L.plotW = plotW;
  L.plotH = plotH;

  L.zx = L.w - pad - L.zw;
  L.zy = pad;
  L.zh = L.h - pad * 2;
}

function mmToCanvas(x_mm, y_mm) {
  return { cx: L.x0 + (x_mm - VIEW_MIN_X) * L.scale, cy: L.y0 - (y_mm - VIEW_MIN_Y) * L.scale, scale: L.scale };
}

function canvasToMm(cx, cy) {
  return { x: VIEW_MIN_X + (cx - L.x0) / L.scale, y: VIEW_MIN_Y + (L.y0 - cy) / L.scale };
}

function pixelToPlate(u, v, motorXMm, motorYMm, fovScale) {
  const cal = latestState.calibration || {};
  const kx = (Number(cal.kx_mm_per_px) || 0.25) * fovScale;
  const ky = (Number(cal.ky_mm_per_px) || 0.25) * fovScale;
  const up = -(u - FRAME_W_PX / 2) * kx;
  const vp = (v - FRAME_H_PX / 2) * ky;
  const pt = cal.plate_transform;
  const skewDeg = pt ? (Number(cal.cam_motor_skew_deg ?? cal.cam_skew_deg) || 0) : (Number(cal.cam_skew_deg) || 0);
  const rad = (skewDeg * Math.PI) / 180;
  const c = Math.cos(rad);
  const s = Math.sin(rad);
  const dx = up * c - vp * s;
  const dy = up * s + vp * c;
  return motorCamToPlate(motorXMm + dx, motorYMm + dy);
}

function rectMm(x0, y0, x1, y1) {
  const a = mmToCanvas(x0, y1);
  const b = mmToCanvas(x1, y0);
  return [a.cx, a.cy, b.cx - a.cx, b.cy - a.cy];
}

function roundRect(x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

function label2d(text, x, y, color, opts = {}) {
  ctx.fillStyle = color;
  ctx.font = `${opts.weight || 500} ${opts.size || 9.5}px ${opts.mono ? THEME.mono : THEME.ui}`;
  ctx.textAlign = opts.align || "left";
  ctx.textBaseline = opts.baseline || "alphabetic";
  ctx.fillText(text, x, y);
  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";
}

function renderCourseCanvas() {
  if (twinMode === "3d" || L.w < 20) return;
  ctx.setTransform(L.dpr, 0, 0, L.dpr, 0, 0);
  ctx.clearRect(0, 0, L.w, L.h);
  ctx.lineCap = "butt";
  ctx.lineJoin = "miter";

  drawXYView();
  drawZView();
}

function drawXYView() {
  const mX = drawX;
  const mY = drawY;

  const gx0 = Math.ceil(VIEW_MIN_X / 10) * 10;
  const gy0 = Math.ceil(VIEW_MIN_Y / 10) * 10;
  const top = mmToCanvas(0, VIEW_MAX_Y).cy;
  const bot = mmToCanvas(0, VIEW_MIN_Y).cy;
  const left = mmToCanvas(VIEW_MIN_X, 0).cx;
  const right = mmToCanvas(VIEW_MAX_X, 0).cx;

  ctx.lineWidth = 1;
  for (let x = gx0; x <= VIEW_MAX_X; x += 10) {
    const cx = Math.round(mmToCanvas(x, 0).cx) + 0.5;
    ctx.strokeStyle = x % 50 === 0 ? THEME.gridMajor : THEME.grid;
    ctx.beginPath(); ctx.moveTo(cx, top); ctx.lineTo(cx, bot); ctx.stroke();
    if (x % 50 === 0) label2d(String(x), cx, bot + 13, THEME.tick, { size: 8.5, mono: true, align: "center" });
  }
  for (let y = gy0; y <= VIEW_MAX_Y; y += 10) {
    const cy = Math.round(mmToCanvas(0, y).cy) + 0.5;
    ctx.strokeStyle = y % 50 === 0 ? THEME.gridMajor : THEME.grid;
    ctx.beginPath(); ctx.moveTo(left, cy); ctx.lineTo(right, cy); ctx.stroke();
    if (y % 50 === 0) label2d(String(y), left - 5, cy + 3, THEME.tick, { size: 8.5, mono: true, align: "right" });
  }

  // Fiducial plate & course
  let r = rectMm(0, 0, PLATE_W_X, PLATE_H_Y);
  ctx.fillStyle = THEME.plateFill;
  ctx.fillRect(...r);
  ctx.strokeStyle = THEME.plateStroke;
  ctx.lineWidth = 1.5;
  ctx.strokeRect(...r);
  if (L.w >= 460) label2d("FIDUCIAL PLATE 284×167.4", r[0] + 2, r[1] - 6, THEME.label, { size: 8.5, weight: 600 });

  r = rectMm(COURSE_ORIGIN_X, COURSE_ORIGIN_Y, COURSE_ORIGIN_X + COURSE_LEN_X, COURSE_ORIGIN_Y + COURSE_WID_Y);
  ctx.fillStyle = THEME.courseFill;
  ctx.fillRect(...r);
  ctx.strokeStyle = THEME.courseStroke;
  ctx.lineWidth = 1;
  ctx.strokeRect(...r);
  if (L.w >= 460) label2d("COURSE 254×152.4", r[0] + 6, r[1] + 13, THEME.label, { size: 8.5, weight: 600 });

  // Aluminum drop-off pedestal
  const pedR = rectMm(DROPOFF_X - ALUM_PLATFORM_SIZE / 2, DROPOFF_Y - ALUM_PLATFORM_SIZE / 2, DROPOFF_X + ALUM_PLATFORM_SIZE / 2, DROPOFF_Y + ALUM_PLATFORM_SIZE / 2);
  ctx.fillStyle = "rgba(148, 163, 184, 0.12)";
  ctx.fillRect(...pedR);

  // Plate origin
  const orig = mmToCanvas(0, 0);
  ctx.fillStyle = THEME.origin;
  ctx.beginPath(); ctx.arc(orig.cx, orig.cy, 3, 0, Math.PI * 2); ctx.fill();
  label2d("(0,0)", orig.cx - 5, orig.cy + 11, THEME.origin, { size: 8.5, mono: true, weight: 600, align: "right" });

  // Corner fiducials
  for (const cf of CORNER_FIDUCIALS) {
    const p = mmToCanvas(cf.x, cf.y);
    const s = FIDUCIAL_SIZE * p.scale;
    ctx.fillStyle = THEME.fidLight;
    ctx.fillRect(p.cx - s / 2, p.cy - s / 2, s, s);
    ctx.fillStyle = THEME.fidDark;
    ctx.fillRect(p.cx - s * 0.36, p.cy - s * 0.36, s * 0.72, s * 0.72);
    ctx.fillStyle = THEME.fidLight;
    ctx.fillRect(p.cx - s * 0.12, p.cy - s * 0.12, s * 0.24, s * 0.24);
    label2d(`#${cf.id}`, p.cx, p.cy + s / 2 + 10, THEME.label, { size: 8, mono: true, align: "center" });
  }

  // Reach envelopes
  const eeCorners = [
    motorCamToPlate(EE_OFFSET_X, EE_OFFSET_Y),
    motorCamToPlate(EE_OFFSET_X + STROKE_X, EE_OFFSET_Y),
    motorCamToPlate(EE_OFFSET_X + STROKE_X, EE_OFFSET_Y + STROKE_Y),
    motorCamToPlate(EE_OFFSET_X, EE_OFFSET_Y + STROKE_Y),
  ].map((p) => mmToCanvas(p.x, p.y));
  ctx.beginPath();
  eeCorners.forEach((c, i) => (i ? ctx.lineTo(c.cx, c.cy) : ctx.moveTo(c.cx, c.cy)));
  ctx.closePath();
  ctx.fillStyle = "rgba(60, 207, 145, 0.035)";
  ctx.fill();
  ctx.strokeStyle = "rgba(60, 207, 145, 0.55)";
  ctx.setLineDash([5, 4]);
  ctx.lineWidth = 1;
  ctx.stroke();
  ctx.setLineDash([]);

  const camCorners = [
    motorCamToPlate(0, 0),
    motorCamToPlate(STROKE_X, 0),
    motorCamToPlate(STROKE_X, STROKE_Y),
    motorCamToPlate(0, STROKE_Y),
  ].map((p) => mmToCanvas(p.x, p.y));
  ctx.beginPath();
  camCorners.forEach((c, i) => (i ? ctx.lineTo(c.cx, c.cy) : ctx.moveTo(c.cx, c.cy)));
  ctx.closePath();
  ctx.fillStyle = "rgba(79, 156, 249, 0.03)";
  ctx.fill();
  ctx.strokeStyle = "rgba(79, 156, 249, 0.55)";
  ctx.setLineDash([2, 3]);
  ctx.stroke();
  ctx.setLineDash([]);

  // Drop zones
  for (const zone of STACK_ZONES) {
    const p = mmToCanvas(zone.x, zone.y);
    const s = BLOCK_SIZE * p.scale;
    const color = COLOR_HEX[zone.color] || "#94a3b8";
    ctx.strokeStyle = color;
    ctx.globalAlpha = 0.85;
    ctx.lineWidth = 1.25;
    ctx.setLineDash([3, 3]);
    ctx.strokeRect(p.cx - s / 2, p.cy - s / 2, s, s);
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
    label2d(zone.label, p.cx, p.cy - s / 2 - 4, color, { size: 8.5, weight: 700, align: "center" });
  }

  // Camera FOV footprint
  const curCamZ = CAM_HOMED_Z - drawZ;
  const fovScale = Math.max(0.05, (curCamZ - CAL_PLANE_Z) / (CAM_HOMED_Z - CAL_PLANE_Z));
  const corners = [[0, 0], [FRAME_W_PX, 0], [FRAME_W_PX, FRAME_H_PX], [0, FRAME_H_PX]]
    .map(([u, v]) => pixelToPlate(u, v, mX, mY, fovScale))
    .map((p) => mmToCanvas(p.x, p.y));
  ctx.beginPath();
  corners.forEach((c, i) => (i ? ctx.lineTo(c.cx, c.cy) : ctx.moveTo(c.cx, c.cy)));
  ctx.closePath();
  ctx.fillStyle = "rgba(79, 156, 249, 0.06)";
  ctx.fill();
  ctx.strokeStyle = "rgba(79, 156, 249, 0.35)";
  ctx.lineWidth = 1;
  ctx.stroke();

  // Live detections projected into the 2D twin
  const liveBlocks = latestState.live_blocks || [];
  for (const b of liveBlocks) {
    if (b.u_px === undefined) continue;
    const pm = pixelToPlate(Number(b.u_px), Number(b.v_px), mX, mY, fovScale);
    const p = mmToCanvas(pm.x, pm.y);
    const s = BLOCK_SIZE * p.scale;
    ctx.save();
    ctx.translate(p.cx, p.cy);
    ctx.rotate((-(Number(b.angle_deg) || 0) * Math.PI) / 180);
    ctx.strokeStyle = COLOR_HEX[b.color] || THEME.text;
    ctx.lineWidth = 1.5;
    ctx.setLineDash([2, 2]);
    ctx.strokeRect(-s / 2, -s / 2, s, s);
    ctx.restore();
  }

  // World / Cataloged blocks
  const blocks = (latestState.world_blocks && latestState.world_blocks.length) ? latestState.world_blocks : (latestState.blocks || []);
  for (const blk of blocks) {
    const p = mmToCanvas(blk.x_mm, blk.y_mm);
    const s = BLOCK_SIZE * p.scale;
    ctx.save();
    ctx.translate(p.cx, p.cy);
    ctx.rotate((-blk.angle_deg * Math.PI) / 180.0);
    ctx.shadowColor = "rgba(0,0,0,0.45)";
    ctx.shadowBlur = 6;
    ctx.shadowOffsetY = 2;
    ctx.fillStyle = COLOR_HEX[blk.color] || "#94a3b8";
    ctx.globalAlpha = blk.status === "UNSCANNED" ? 0.45 : 1.0;
    roundRect(-s / 2, -s / 2, s, s, 2.5);
    ctx.fill();
    ctx.shadowColor = "transparent";
    ctx.strokeStyle = blk.status === "STACKED" ? THEME.ee : "rgba(255,255,255,0.28)";
    ctx.lineWidth = blk.status === "STACKED" ? 2 : 1;
    ctx.stroke();
    ctx.restore();
    ctx.globalAlpha = 1.0;
    label2d(`${blk.id}`, p.cx, p.cy + 0.5, LIGHT_BLOCKS.has(blk.color) ? "#0b0e13" : "#ffffff", {
      size: 9, weight: 700, mono: true, align: "center", baseline: "middle",
    });
  }

  // Gantry skeleton: carriage datum -> end-effector -> camera
  const homeDatum = mmToCanvas(DATUM_X, DATUM_Y);
  const datum = mmToCanvas(DATUM_X + mX, DATUM_Y + mY);
  const eePlate = motorCamToPlate(mX + EE_OFFSET_X, mY + EE_OFFSET_Y);
  const camPlate = motorCamToPlate(mX, mY);
  const ee = mmToCanvas(eePlate.x, eePlate.y);
  const cam = mmToCanvas(camPlate.x, camPlate.y);

  ctx.strokeStyle = THEME.carriage;
  ctx.globalAlpha = 0.6;
  ctx.lineWidth = 1.25;
  ctx.setLineDash([2, 2]);
  ctx.beginPath(); ctx.arc(homeDatum.cx, homeDatum.cy, 5, 0, Math.PI * 2); ctx.stroke();
  ctx.setLineDash([]);
  ctx.globalAlpha = 1;

  ctx.strokeStyle = "rgba(245, 165, 36, 0.16)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(datum.cx, top); ctx.lineTo(datum.cx, bot);
  ctx.moveTo(left, ee.cy); ctx.lineTo(right, ee.cy);
  ctx.stroke();

  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.strokeStyle = "rgba(245, 165, 36, 0.25)";
  ctx.lineWidth = 7;
  ctx.beginPath(); ctx.moveTo(datum.cx, datum.cy); ctx.lineTo(ee.cx, ee.cy); ctx.lineTo(cam.cx, cam.cy); ctx.stroke();
  ctx.strokeStyle = THEME.carriage;
  ctx.lineWidth = 2.5;
  ctx.stroke();
  ctx.lineCap = "butt";
  ctx.lineJoin = "miter";

  ctx.fillStyle = THEME.carriage;
  ctx.beginPath(); ctx.arc(datum.cx, datum.cy, 4.5, 0, Math.PI * 2); ctx.fill();

  const eeSide = 22 * ee.scale;
  ctx.save();
  ctx.translate(ee.cx, ee.cy);
  ctx.rotate((-drawAngle * Math.PI) / 180);
  ctx.fillStyle = "rgba(60, 207, 145, 0.22)";
  ctx.fillRect(-eeSide / 2, -eeSide / 2, eeSide, eeSide);
  ctx.strokeStyle = THEME.ee;
  ctx.lineWidth = 1.8;
  ctx.strokeRect(-eeSide / 2, -eeSide / 2, eeSide, eeSide);
  ctx.beginPath();
  ctx.moveTo(-3.5, 0); ctx.lineTo(3.5, 0);
  ctx.moveTo(0, -3.5); ctx.lineTo(0, 3.5);
  ctx.stroke();
  ctx.restore();

  ctx.fillStyle = "rgba(79, 156, 249, 0.22)";
  ctx.beginPath(); ctx.arc(cam.cx, cam.cy, 7, 0, Math.PI * 2); ctx.fill();
  ctx.strokeStyle = THEME.cam;
  ctx.lineWidth = 1.8;
  ctx.stroke();
  ctx.fillStyle = THEME.cam;
  ctx.beginPath(); ctx.arc(cam.cx, cam.cy, 2.2, 0, Math.PI * 2); ctx.fill();

  if (mouseMm) {
    const p = mmToCanvas(mouseMm.x, mouseMm.y);
    ctx.strokeStyle = "rgba(230, 235, 242, 0.22)";
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.moveTo(p.cx, top); ctx.lineTo(p.cx, bot);
    ctx.moveTo(left, p.cy); ctx.lineTo(right, p.cy);
    ctx.stroke();
    ctx.setLineDash([]);
  }
}

function drawZView() {
  const x = L.zx;
  const y = L.zy;
  const w = L.zw;
  const h = L.zh;

  roundRect(x, y, w, h, 8);
  ctx.fillStyle = THEME.panel;
  ctx.fill();
  ctx.strokeStyle = THEME.panelStroke;
  ctx.lineWidth = 1;
  ctx.stroke();
  label2d("Z ELEV", x + 8, y + 16, THEME.labelStrong, { size: 9, weight: 700 });

  const zTop = y + 32;
  const zBot = y + h - 20;
  const zToY = (z) => zBot - (z / Z_VIEW_MAX) * (zBot - zTop);
  const left = x + 8;
  const right = x + w - 8;
  const colX = left + 16;

  const curCamZ = CAM_HOMED_Z - drawZ;
  const curEeZ = EE_HOMED_Z - drawZ;
  const camY = zToY(curCamZ);
  const eeY = zToY(curEeZ);

  for (let z = 0; z <= Z_VIEW_MAX; z += 25) {
    const yy = Math.round(zToY(z)) + 0.5;
    ctx.strokeStyle = THEME.grid;
    ctx.beginPath(); ctx.moveTo(left, yy); ctx.lineTo(right, yy); ctx.stroke();
    if (Math.abs(yy - camY) > 9 && Math.abs(yy - eeY) > 9) {
      label2d(String(z), right, yy - 2.5, THEME.tick, { size: 8, mono: true, align: "right" });
    }
  }

  const floorY = zToY(0);
  ctx.fillStyle = "#222a36";
  ctx.fillRect(left, zToY(COURSE_FLOOR_Z), right - left, floorY - zToY(COURSE_FLOOR_Z));
  ctx.strokeStyle = "#4a5568";
  ctx.lineWidth = 1.25;
  ctx.beginPath(); ctx.moveTo(left, floorY); ctx.lineTo(right, floorY); ctx.stroke();
  label2d("BED Z0", left, floorY + 12, THEME.label, { size: 8, mono: true });

  const pedY = zToY(DROPOFF_Z_UP);
  const pedX = colX + 24;
  const pedW = Math.max(20, right - pedX - 2);
  ctx.fillStyle = "rgba(148, 163, 184, 0.16)";
  ctx.fillRect(pedX, pedY, pedW, floorY - pedY);
  ctx.strokeStyle = "rgba(148, 163, 184, 0.5)";
  ctx.lineWidth = 1;
  ctx.strokeRect(pedX + 0.5, pedY + 0.5, pedW - 1, floorY - pedY - 1);

  const datumY = zToY(DATUM_Z);
  ctx.strokeStyle = "rgba(245, 165, 36, 0.5)";
  ctx.setLineDash([3, 3]);
  ctx.beginPath(); ctx.moveTo(left, datumY); ctx.lineTo(right, datumY); ctx.stroke();
  ctx.setLineDash([]);

  ctx.strokeStyle = THEME.carriage;
  ctx.lineWidth = 2.5;
  ctx.lineCap = "round";
  ctx.beginPath(); ctx.moveTo(colX, camY); ctx.lineTo(colX, eeY); ctx.stroke();
  ctx.lineCap = "butt";

  ctx.fillStyle = THEME.cam;
  ctx.beginPath(); ctx.arc(colX, camY, 4.2, 0, Math.PI * 2); ctx.fill();
  label2d(`CAM ${curCamZ.toFixed(0)}`, colX + 10, camY + 3, THEME.cam, { size: 8.5, mono: true, weight: 600 });

  ctx.fillStyle = THEME.ee;
  ctx.fillRect(colX - 6, eeY, 12, 7.5);
  label2d(`EE ${curEeZ.toFixed(0)}`, colX + 10, eeY + 7, THEME.ee, { size: 8.5, mono: true, weight: 600 });
}

courseCanvas.addEventListener("mousemove", (e) => {
  const rect = courseCanvas.getBoundingClientRect();
  const p = canvasToMm(e.clientX - rect.left, e.clientY - rect.top);
  const inside = p.x >= VIEW_MIN_X && p.x <= VIEW_MAX_X && p.y >= VIEW_MIN_Y && p.y <= VIEW_MAX_Y;
  mouseMm = inside ? p : null;
  setText("cursorReadout", inside ? `${p.x.toFixed(1)}, ${p.y.toFixed(1)} mm` : "— , — mm");
});
courseCanvas.addEventListener("mouseleave", () => {
  mouseMm = null;
  setText("cursorReadout", "— , — mm");
});

// ============================================================================
//  DIAGNOSTIC RUN REPORTS & 3D HISTORICAL RUN REPLAY
// ============================================================================
let cachedReportsList = [];
let activeReportId = null;
let currentModalReport = null;

function getApiBaseUrl() {
  const wsVal = wsUrlInput.value.trim();
  try {
    const u = new URL(wsVal);
    const proto = u.protocol === "wss:" ? "https:" : "http:";
    return `${proto}//${u.host}`;
  } catch (_) {
    return window.location.origin || "http://localhost:8765";
  }
}

async function fetchRecentReports() {
  try {
    const res = await fetch(`${getApiBaseUrl()}/api/reports`);
    if (!res.ok) return;
    cachedReportsList = await res.json();
    renderRecentReportsList(cachedReportsList);
  } catch (err) {
    console.debug("fetchRecentReports error:", err);
  }
}

function renderRecentReportsList(reports) {
  const listEl = $("recentReportsList");
  const badgeEl = $("reportStatusBadge");
  const summaryEl = $("reportSummaryText");

  if (!reports || reports.length === 0) {
    if (badgeEl) { badgeEl.textContent = "—"; badgeEl.className = "tab-badge"; }
    if (summaryEl) summaryEl.textContent = "Run reports are cached automatically after each scan cycle.";
    if (listEl) listEl.innerHTML = "<span class='muted'>No reports found on Pi.</span>";
    return;
  }

  const latest = reports[0];
  if (badgeEl) {
    badgeEl.textContent = latest.status || "COMPLETED";
    badgeEl.className = `tab-badge ${latest.status === "COMPLETED" ? "ok" : latest.status === "RUNNING" ? "run" : "err"}`;
  }
  if (summaryEl) {
    const skewStr = latest.cam_skew_deg !== undefined ? ` · skew ${Number(latest.cam_skew_deg).toFixed(1)}°` : "";
    const t = (latest.start_time || "").split("T")[1]?.slice(0, 8) || "";
    summaryEl.textContent = `Latest ${latest.run_id} (${t}) · ${latest.blocks_cataloged || 0} blocks · ${latest.duration_s || 0}s${skewStr}`;
  }

  if (listEl) {
    listEl.innerHTML = reports
      .slice(0, 15)
      .map(
        (r) => `
      <button class="recent-report-pill ${r.run_id === activeReportId ? "active" : ""}" onclick="viewReport('${r.run_id}')" title="Click to inspect or replay ${r.run_id}">
        <span class="${r.status === "COMPLETED" ? "ok" : ""}">${r.status === "COMPLETED" ? "✓" : "•"}</span>
        ${r.run_id.replace("run_", "")}
        <span class="muted">${r.blocks_picked || r.blocks_cataloged || 0} blk</span>
      </button>`
      )
      .join("");
  }
}

async function viewLatestReport() {
  try {
    const res = await fetch(`${getApiBaseUrl()}/api/reports/latest`);
    const data = res.ok ? await res.json() : null;
    if (!data || !data.run_id) {
      alert("No report available yet. Run a scan cycle first.");
      return;
    }
    showReportModal(data);
  } catch (err) {
    alert(`Could not fetch latest report: ${err.message}`);
  }
}

async function viewReport(runId) {
  activeReportId = runId;
  renderRecentReportsList(cachedReportsList);
  try {
    const res = await fetch(`${getApiBaseUrl()}/api/reports/${encodeURIComponent(runId)}`);
    if (!res.ok) {
      alert(`Report ${runId} not found.`);
      return;
    }
    showReportModal(await res.json());
  } catch (err) {
    alert(`Error loading report: ${err.message}`);
  }
}

function downloadLatestReport() {
  window.open(`${getApiBaseUrl()}/api/reports/latest/download`, "_blank");
}

function closeReportModal() {
  const modal = $("reportModal");
  if (modal) modal.style.display = "none";
}

function replayModalReport() {
  if (!currentModalReport) return;
  const rep = currentModalReport;
  closeReportModal();
  startReportReplay(rep);
}

function showReportModal(report) {
  const modal = $("reportModal");
  const body = $("modalReportBody");
  if (!modal || !body) return;

  currentModalReport = report;
  activeReportId = report.run_id;
  setText("modalReportTitle", `Run ${report.run_id} · ${report.status}`);
  const dlBtn = $("modalDownloadBtn");
  if (dlBtn) dlBtn.href = `${getApiBaseUrl()}/api/reports/${encodeURIComponent(report.run_id)}/download`;

  const cal = report.calibration || {};
  const blocks = report.cataloged_blocks || [];
  const picks = report.pick_actions || [];
  const cmds = report.motor_commands || [];
  const ok = "var(--success)";
  const bad = "var(--danger)";

  body.innerHTML = `
    <div class="report-section">
      <h4>Overview</h4>
      <div class="report-grid-kv">
        <div><span class="kv-label">Run ID</span><span class="kv-val">${report.run_id}</span></div>
        <div><span class="kv-label">Status</span><span class="kv-val" style="color:${report.status === "COMPLETED" ? ok : bad}">${report.status}</span></div>
        <div><span class="kv-label">Start time</span><span class="kv-val">${report.start_time || "N/A"}</span></div>
        <div><span class="kv-label">Duration</span><span class="kv-val">${report.duration_s || 0} s</span></div>
        <div><span class="kv-label">Mode</span><span class="kv-val">${report.mock_hardware ? "Mock (simulated)" : "Physical ESP32"}</span></div>
        <div><span class="kv-label">Serial commands</span><span class="kv-val">${cmds.length}</span></div>
      </div>
    </div>

    <div class="report-section">
      <h4>1 · Homing, 4-ArUco calibration &amp; camera skew</h4>
      <div class="report-grid-kv">
        <div><span class="kv-label">Calibration mode</span><span class="kv-val">${cal.calibration_mode || "single_marker"} (${(cal.markers_detected || []).join(", ") || ("#" + (cal.marker_id ?? "N/A"))})</span></div>
        <div><span class="kv-label">Plate yaw / RMS err</span><span class="kv-val">${cal.plate_yaw_deg != null ? cal.plate_yaw_deg + "°" : "0.0°"} / ${cal.residual_rms_mm != null ? cal.residual_rms_mm + " mm" : "—"}</span></div>
        <div><span class="kv-label">Camera skew</span><span class="kv-val" style="color:${Math.abs(cal.cam_skew_deg || 0) > 0.05 ? "var(--accent)" : "inherit"}">${cal.cam_skew_deg !== undefined ? cal.cam_skew_deg + "° (compensated)" : "0.0°"}</span></div>
        <div><span class="kv-label">Scale kx / ky</span><span class="kv-val">${cal.kx_mm_per_px || 0.25} / ${cal.ky_mm_per_px || 0.25} mm/px</span></div>
        <div><span class="kv-label">Cam origin</span><span class="kv-val">(${cal.cam_origin_x_mm ?? 32.5}, ${cal.cam_origin_y_mm ?? 33.1}) mm</span></div>
        <div><span class="kv-label">Cam → gripper offset</span><span class="kv-val">(${cal.offset_x_mm ?? -1.5}, ${cal.offset_y_mm ?? -41.6}) mm</span></div>
        <div><span class="kv-label">Course floor</span><span class="kv-val">+6.35 mm (¼″)</span></div>
      </div>
    </div>

    <div class="report-section">
      <h4>2 · Cataloged blocks (${blocks.length})</h4>
      ${blocks.length === 0 ? "<p class='muted'>No blocks cataloged during scan.</p>" : `
      <table class="modal-table">
        <thead><tr><th>ID</th><th>Color</th><th>Plate (X, Y) mm</th><th>Angle</th><th>Views</th></tr></thead>
        <tbody>
          ${blocks.map((b) => `
            <tr>
              <td>#${b.id}</td>
              <td><span class="swatch" style="display:inline-block;width:9px;height:9px;border-radius:2px;margin-right:6px;background:${COLOR_HEX[b.color] || "#94a3b8"}"></span>${b.color}</td>
              <td>(${Number(b.x_mm).toFixed(1)}, ${Number(b.y_mm).toFixed(1)})</td>
              <td>${Number(b.angle_deg).toFixed(1)}°</td>
              <td>${b.confidence_views || 1}</td>
            </tr>`).join("")}
        </tbody>
      </table>`}
    </div>

    <div class="report-section">
      <h4>3 · Picks, fine alignment &amp; weighing (${picks.length})</h4>
      ${picks.length === 0 ? "<p class='muted'>No pick actions executed.</p>" : `
      <table class="modal-table">
        <thead><tr><th>Block</th><th>Fine align</th><th>Contour</th><th>Residual</th><th>Grip (X, Y, θ)</th><th>Tare / Mass</th><th>Class</th><th>Drop zone</th></tr></thead>
        <tbody>
          ${picks.map((p) => `
            <tr>
              <td>#${p.block_id} ${String(p.color).toUpperCase()}</td>
              <td>${p.fine_align_found ? `<span style="color:${ok}">FOUND</span>` : `<span style="color:var(--warning)">COARSE</span>`}</td>
              <td>${p.fine_contour_area_px ? p.fine_contour_area_px.toFixed(0) + " px" : "—"}</td>
              <td>${p.fine_residual_err_mm != null ? p.fine_residual_err_mm.toFixed(2) + " mm" : "—"}</td>
              <td>(${p.gripper_motor_x.toFixed(1)}, ${p.gripper_motor_y.toFixed(1)}, ${p.servo_angle_deg.toFixed(1)}°)</td>
              <td>${p.tare_success ? "✓" : "—"} / ${p.measured_mass_g != null ? p.measured_mass_g.toFixed(2) + " g" : "—"}</td>
              <td><span class="pill pill-${(p.mass_class || "").toLowerCase()}">${p.mass_class || "—"}</span></td>
              <td>${p.status} (${p.stack_zone})</td>
            </tr>`).join("")}
        </tbody>
      </table>`}
    </div>

    <div class="report-section">
      <h4>4 · Motor command sequence (${cmds.length})</h4>
      <div class="cmd-log">
        ${cmds.map((c) => `<div>[${c.timestamp}] <b>TX</b> ${c.command} → <i>${(c.replies || []).join(", ") || "OK"}</i> (${c.duration_ms} ms)</div>`).join("")}
      </div>
    </div>
  `;

  modal.style.display = "flex";
}

async function replayLatestReport() {
  try {
    const res = await fetch(`${getApiBaseUrl()}/api/reports/latest`);
    if (!res.ok) return;
    const rep = await res.json();
    if (rep && rep.run_id) startReportReplay(rep);
  } catch (_) {}
}

function startReportReplay(report) {
  const frames = [];
  const baseBlocks = (report.cataloged_blocks || []).map((b) => ({
    ...b,
    z_mm: COURSE_FLOOR_Z,
    status: "UNSCANNED",
  }));

  const cloneBlocks = (arr) => arr.map((b) => ({ ...b }));
  let simBlocks = cloneBlocks(baseBlocks);

  frames.push({
    state: "CALIBRATING",
    gantry_pose: { x_mm: 0, y_mm: 0, z_mm: 0, angle_deg: 0, grip_closed: true },
    world_blocks: cloneBlocks(simBlocks),
    blocks: [],
    held_block_id: null,
    current_mass_g: null,
    mass_class: null,
  });

  for (const wp of (report.scan_waypoints || [])) {
    for (const raw of (wp.raw_detections || [])) {
      const m = simBlocks.find((b) => Math.hypot(b.x_mm - raw.world_x_mm, b.y_mm - raw.world_y_mm) < 12);
      if (m) m.status = "DETECTED";
    }
    frames.push({
      state: "SCANNING",
      gantry_pose: { x_mm: wp.motor_x_mm, y_mm: wp.motor_y_mm, z_mm: wp.motor_z_mm || 0, angle_deg: 0, grip_closed: true },
      world_blocks: cloneBlocks(simBlocks),
      blocks: cloneBlocks(simBlocks.filter((b) => b.status !== "UNSCANNED")),
      held_block_id: null,
      current_mass_g: null,
      mass_class: null,
    });
  }

  for (const p of (report.pick_actions || [])) {
    const blk = simBlocks.find((b) => b.id === p.block_id);
    frames.push({
      state: "PICKING",
      gantry_pose: { x_mm: p.camera_motor_x, y_mm: p.camera_motor_y, z_mm: 25, angle_deg: 0, grip_closed: false },
      world_blocks: cloneBlocks(simBlocks),
      blocks: cloneBlocks(simBlocks),
      held_block_id: null,
      current_mass_g: 0.0,
      mass_class: null,
    });
    frames.push({
      state: "PICKING",
      gantry_pose: { x_mm: p.gripper_motor_x, y_mm: p.gripper_motor_y, z_mm: STROKE_Z_BOTTOM, angle_deg: p.servo_angle_deg, grip_closed: true },
      world_blocks: cloneBlocks(simBlocks),
      blocks: cloneBlocks(simBlocks),
      held_block_id: p.block_id,
      current_mass_g: 65.0,
      mass_class: null,
    });
    if (blk) {
      blk.status = "HELD";
      blk.mass_g = p.measured_mass_g || 0;
      blk.mass_class = p.mass_class || "unknown";
    }
    frames.push({
      state: "WEIGHING",
      gantry_pose: { x_mm: p.gripper_motor_x, y_mm: p.gripper_motor_y, z_mm: 25, angle_deg: p.servo_angle_deg, grip_closed: true },
      world_blocks: cloneBlocks(simBlocks),
      blocks: cloneBlocks(simBlocks),
      held_block_id: p.block_id,
      current_mass_g: p.measured_mass_g,
      mass_class: p.mass_class,
    });
    const dropMx = clamp(p.drop_plate_x - (CAM_HOMED_X + EE_OFFSET_X), 0, STROKE_X);
    const dropMy = clamp(p.drop_plate_y - (CAM_HOMED_Y + EE_OFFSET_Y), 0, STROKE_Y);
    frames.push({
      state: "STACKING",
      gantry_pose: { x_mm: dropMx, y_mm: dropMy, z_mm: 25, angle_deg: 0, grip_closed: true },
      world_blocks: cloneBlocks(simBlocks),
      blocks: cloneBlocks(simBlocks),
      held_block_id: p.block_id,
      current_mass_g: p.measured_mass_g,
      mass_class: p.mass_class,
    });
    if (blk) {
      blk.x_mm = p.drop_plate_x;
      blk.y_mm = p.drop_plate_y;
      blk.z_mm = p.mass_class === "aluminum" ? DROPOFF_Z_UP : COURSE_FLOOR_Z;
      blk.angle_deg = 0;
      blk.status = "STACKED";
    }
    frames.push({
      state: "STACKING",
      gantry_pose: { x_mm: dropMx, y_mm: dropMy, z_mm: p.drop_z_down_mm || STROKE_Z_BOTTOM, angle_deg: 0, grip_closed: false },
      world_blocks: cloneBlocks(simBlocks),
      blocks: cloneBlocks(simBlocks),
      held_block_id: null,
      current_mass_g: 0.0,
      mass_class: p.mass_class,
    });
  }

  frames.push({
    state: "COMPLETE",
    gantry_pose: { x_mm: 0, y_mm: 0, z_mm: 0, angle_deg: 0, grip_closed: true },
    world_blocks: cloneBlocks(simBlocks),
    blocks: cloneBlocks(simBlocks),
    held_block_id: null,
    current_mass_g: null,
    mass_class: null,
  });

  sandboxPreviewActive = false;
  setSyncButtonVisible(false);
  replayActive = true;
  replayPlaying = true;
  replayFrames = frames;
  replayIdx = 0;
  clearTrail();

  $("replayBar").classList.remove("hidden");
  $("replayScrub").max = String(Math.max(0, frames.length - 1));
  setText("replayRunName", report.run_id);
  applyReplayFrame(0);
  startReplayInterval();
}

function applyReplayFrame(idx) {
  replayIdx = clamp(idx, 0, Math.max(0, replayFrames.length - 1));
  const f = replayFrames[replayIdx];
  if (!f) return;
  $("replayScrub").value = String(replayIdx);
  setText("replayStepLabel", `${replayIdx + 1} / ${replayFrames.length}`);
  latestState = {
    ...latestState,
    ...f,
    logs: latestState.logs,
  };
  updateUI(latestState);
}

function startReplayInterval() {
  if (replayTimer) clearInterval(replayTimer);
  $("replayPlayBtn").innerHTML = "&#10074;&#10074; Pause";
  replayTimer = setInterval(() => {
    if (!replayPlaying) return;
    if (replayIdx < replayFrames.length - 1) {
      applyReplayFrame(replayIdx + 1);
    } else {
      replayPlaying = false;
      $("replayPlayBtn").innerHTML = "&#9654; Play";
    }
  }, 850);
}

function toggleReplayPlay() {
  replayPlaying = !replayPlaying;
  $("replayPlayBtn").innerHTML = replayPlaying ? "&#10074;&#10074; Pause" : "&#9654; Play";
  if (replayPlaying && replayIdx >= replayFrames.length - 1) {
    applyReplayFrame(0);
  }
}

function onReplayScrub(val) {
  replayPlaying = false;
  $("replayPlayBtn").innerHTML = "&#9654; Play";
  applyReplayFrame(Number(val));
}

function exitReplayMode() {
  if (!replayActive) return;
  replayActive = false;
  replayPlaying = false;
  if (replayTimer) {
    clearInterval(replayTimer);
    replayTimer = null;
  }
  $("replayBar").classList.add("hidden");
}

// ============================================================================
//  MAIN 60 FPS ANIMATION LOOP & RESIZE
// ============================================================================
function resizeAll() {
  computeLayout();
  if (renderer3d && viewport3dWrap) {
    const w = Math.max(200, viewport3dWrap.clientWidth);
    const h = Math.max(200, viewport3dWrap.clientHeight);
    renderer3d.setSize(w, h, false);
    if (orbitCam) {
      orbitCam.aspect = w / h;
      orbitCam.updateProjectionMatrix();
    }
    if (eyeCam) {
      eyeCam.aspect = w / h;
      eyeCam.updateProjectionMatrix();
    }
  }
}

new ResizeObserver(resizeAll).observe($("twinStage"));
window.addEventListener("resize", resizeAll);

let prevDrawZ = 0;
let lastFpsStamp = performance.now();
let fpsCounter = 0;

function animate() {
  requestAnimationFrame(animate);

  const k = 0.25;
  drawX += (motorX - drawX) * k;
  drawY += (motorY - drawY) * k;
  drawZ += (motorZDown - drawZ) * k;
  drawAngle += (servoAngle - drawAngle) * k;
  const targetGrip = gripClosed ? 1.0 : 0.0;
  drawGripFactor += (targetGrip - drawGripFactor) * 0.3;

  // Apply to 5-Stage 3D Kinematic Hierarchy (Compiled Onshape CAD)
  if (xStageGroup && zStageGroup && yStageGroup && eeStageGroup) {
    const cmx = compiledCadMeta.cad_motor_x_mm;
    const cmy = compiledCadMeta.cad_motor_y_mm;
    const piv = compiledCadMeta.ee_pivot_three || [-96.6, 106.5, -1.5];
    xStageGroup.position.set(drawX - cmx, 0, 0);
    zStageGroup.position.set(0, -drawZ, 0);
    yStageGroup.position.set(0, 0, -(drawY - cmy));
    eeStageGroup.position.set(piv[0], piv[1], piv[2]);
    eeStageGroup.rotation.y = (drawAngle * Math.PI) / 180.0;
    if (compiledPlungerGroup) {
      const ret = (1.0 - drawGripFactor) * 8.0;
      compiledPlungerGroup.position.set(ret * 0.707, 0, -ret * 0.707);
    }

    if (Math.abs(drawZ - prevDrawZ) > 0.15) {
      const camHomeY = compiledCadMeta?.cam_local_three?.[1] ?? 120.5;
      rebuildFovCone(camHomeY - drawZ);
    }

    sync3DBlocks(latestState);
    updateTrail();

    if (cameraPreset === "follow" && orbitCam && controls3d) {
      const eeWorld = plateToThree(EE_HOMED_X + drawX, EE_HOMED_Y + drawY, EE_HOMED_Z - drawZ - 10);
      controls3d.target.lerp(eeWorld, 0.12);
      const desiredPos = eeWorld.clone().add(new THREE.Vector3(75, 85, -105));
      orbitCam.position.lerp(desiredPos, 0.08);
    }
  }

  prevDrawZ = drawZ;

  if (renderer3d && scene3d && activeCam3d && twinMode !== "2d") {
    if (activeCam3d === orbitCam && controls3d) {
      controls3d.update();
    }
    renderer3d.render(scene3d, activeCam3d);
  }

  renderCourseCanvas();

  fpsCounter++;
  const now = performance.now();
  if (now - lastFpsStamp >= 1000) {
    setText("hudFps", `${fpsCounter} FPS · WebGL`);
    fpsCounter = 0;
    lastFpsStamp = now;
  }
}

document.addEventListener("keydown", (e) => {
  if (e.key === "Escape") {
    toggleCamExpand(false);
    closeReportModal();
    closeCadImportModal();
  }
});

// ============================================================================
//  BOOT
// ============================================================================
init3DScene();
computeLayout();
syncSlidersFromState();
updateReadoutsFromMotor();
connectWebSocket();
fetchRecentReports();
animate();
