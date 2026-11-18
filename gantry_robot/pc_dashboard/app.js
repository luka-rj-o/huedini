// ============================================================================
//  HUE-DINI Gantry Supervisor — dashboard client
//  Digital-twin renderer (responsive, HiDPI) + Pi WebSocket telemetry client
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
// Fiducial plate & inner course (origin 0,0,0 at bottom-left of outer 284 x 167.4 plate).
// Course back edge is coincident with the plate back edge at Y = 0.0.
const PLATE_W_X = 284.0;
const PLATE_H_Y = 167.4;
const COURSE_ORIGIN_X = 15.0; // (284 - 254) / 2
const COURSE_ORIGIN_Y = 0.0;
const COURSE_LEN_X = 254.0;
const COURSE_WID_Y = 152.4;
const FIDUCIAL_SIZE = 15.0;
const BLOCK_SIZE = 20.0;
const COURSE_FLOOR_Z = 6.35; // course is elevated 1/4"

// Motor travel strokes from homed (0, 0, 0)
const STROKE_X = 227.0;
const STROKE_Y = 128.0;
const STROKE_Z_MAX = 76.7;
const STROKE_Z_BOTTOM = 70.36; // full descent to elevated course floor (76.71 - 6.35)

// Homed datum (tower origin)
const DATUM_X = 28.5;
const DATUM_Y = -43.5;
const DATUM_Z = 127.5;

// Nominal homed end-effector & camera
const EE_HOMED_X = 32.5;
const EE_HOMED_Y = -6.0;
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

// Camera sensor
const FRAME_W_PX = 640;
const FRAME_H_PX = 480;
const CAL_PLANE_Z = COURSE_FLOOR_Z + BLOCK_SIZE; // scale is calibrated on 20 mm cube top faces

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
  { label: "RED", color: "red", x: 250.2, y: 115.0 },
  { label: "GREEN", color: "green", x: 210.2, y: 115.0 },
  { label: "BLUE", color: "blue", x: 170.2, y: 115.0 },
  { label: "ALUM", color: "aluminum", x: DROPOFF_X, y: DROPOFF_Y },
];

// Canvas palette (mirrors CSS tokens in styles.css)
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
// Target motor travel from homed (0,0,0) — authoritative values from telemetry
let motorX = 0.0;
let motorY = 0.0;
let motorZDown = 0.0;
// Rendered (eased) values so the twin glides between telemetry packets
let drawX = 0.0;
let drawY = 0.0;
let drawZ = 0.0;

let socket = null;
let latestState = {
  state: "IDLE",
  gantry_pose: { x_mm: 0.0, y_mm: 0.0, z_mm: 0.0, angle_deg: 0.0, grip_closed: true },
  current_mass_g: null,
  mass_class: null,
  calibration: { calibrated: false, kx_mm_per_px: 0.25, ky_mm_per_px: 0.25 },
  steps_per_mm: { x: 50.0, y: 10.0, z: 50.0 },
  blocks: [],
  logs: [],
  vision_frame_b64: null,
};

let currentJogStepMm = 10.0;
let mouseMm = null; // cursor position on map in plate mm

function isOpen() {
  return socket && socket.readyState === WebSocket.OPEN;
}

function sendCmd(command) {
  if (isOpen()) socket.send(JSON.stringify({ command }));
}

// ============================================================================
//  RUN CONTROLS
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

// ============================================================================
//  MANUAL JOG
// ============================================================================
function setJogStep(stepMm) {
  currentJogStepMm = stepMm;
  document.querySelectorAll(".jog-step-btn").forEach((b) => b.classList.remove("active"));
  const activeBtn = $(`jogStep${Math.round(stepMm)}`);
  if (activeBtn) activeBtn.classList.add("active");
}

function jogAxis(axis, dir) {
  const uiDelta = dir * currentJogStepMm;
  // Physical Z motor travel counts downward from top home (0 = top, +Z_down = lower),
  // while the jog pad uses +Z = Up and -Z = Down.
  const motorDelta = axis === "Z" ? -uiDelta : uiDelta;
  if (!isOpen()) return;

  // Optimistic UI update (instant feedback)
  if (axis === "X") motorX = clamp(motorX + motorDelta, 0, STROKE_X);
  else if (axis === "Y") motorY = clamp(motorY + motorDelta, 0, STROKE_Y);
  else if (axis === "Z") motorZDown = clamp(motorZDown + motorDelta, 0, STROKE_Z_BOTTOM);
  updateReadoutsFromMotor();
  scheduleRender();

  socket.send(JSON.stringify({ command: "JOG_AXIS", axis, dist_mm: motorDelta }));
}

// ============================================================================
//  LEAD SCREW CALIBRATION
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

  if (isOpen()) {
    socket.send(JSON.stringify({
      command: "SET_STEPS_CALIBRATION",
      steps_per_mm_x: spm,
      steps_per_mm_y: 10.0,
      steps_per_mm_z: spm,
    }));
  }
}

function promptCustomSpm() {
  const curX = latestState.steps_per_mm?.x || 200.0;
  const input = prompt("Steps/mm for X and Z lead screws (50 = T8×8, 100 = T8×4, 200 = T8×2):", curX.toFixed(1));
  if (input === null) return;
  const val = parseFloat(input);
  if (!isNaN(val) && val > 0) applyLeadPreset(0, val);
}

// ============================================================================
//  CAMERA
// ============================================================================
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

document.addEventListener("keydown", (e) => {
  if (e.key === "Escape") {
    toggleCamExpand(false);
    closeReportModal();
  }
});

// ============================================================================
//  LOAD CELL
// ============================================================================
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
  const massEl = $("liveMassReadout");
  if (massEl) massEl.textContent = "0.00";
  setMassPill("Tared", "pill-dim");
  highlightMassBin(null);
  sendCmd("TARE_SCALE");
}

function sendWeighCmd() {
  const massEl = $("liveMassReadout");
  if (massEl) massEl.textContent = "…";
  setMassPill("Sampling", "pill-accent");
  sendCmd("WEIGH_BLOCK");
}

// ============================================================================
//  CONSOLE
// ============================================================================
function sendRawSerial() {
  const input = $("rawSerialInput");
  const line = input.value.trim();
  if (!line) return;
  if (isOpen()) {
    socket.send(JSON.stringify({ command: "RAW_SERIAL", line }));
    input.value = "";
  }
}

function clearLogs() {
  const logEl = $("logConsole");
  if (logEl) logEl.textContent = "";
  lastLogSignature = "";
}

$("rawSerialInput").addEventListener("keydown", (e) => {
  if (e.key === "Enter") sendRawSerial();
});

// ============================================================================
//  TABS
// ============================================================================
function selectTab(name) {
  document.querySelectorAll(".tab").forEach((t) => t.classList.toggle("active", t.dataset.tab === name));
  document.querySelectorAll(".tab-pane").forEach((p) => p.classList.toggle("active", p.dataset.pane === name));
  document.querySelectorAll(".tab-actions").forEach((a) => a.classList.toggle("hidden", a.dataset.for !== name));
  if (name === "reports") fetchRecentReports();
}

// ============================================================================
//  WEBSOCKET
// ============================================================================
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
  } catch (err) {
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
//  READOUTS
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

  // Z displayed as negative displacement from top home so +Z reads as Up
  setText("motorXReadout", motorX.toFixed(1));
  setText("motorYReadout", motorY.toFixed(1));
  setText("motorZReadout", (motorZDown > 0.005 ? -motorZDown : 0.0).toFixed(2));

  $("motorXBar").style.width = `${clamp((motorX / STROKE_X) * 100, 0, 100)}%`;
  $("motorYBar").style.width = `${clamp((motorY / STROKE_Y) * 100, 0, 100)}%`;
  $("motorZBar").style.width = `${clamp((motorZDown / STROKE_Z_BOTTOM) * 100, 0, 100)}%`;
}

let lastLogSignature = "";
let lastBlocksSignature = "";

function updateUI(data, prevState) {
  const state = data.state || "IDLE";
  if (prevState !== state && (state === "COMPLETE" || state === "ERROR")) {
    setTimeout(fetchRecentReports, 800);
  }

  setText("stateReadout", state);
  const chip = $("stateChip");
  if (chip) chip.dataset.state = state;
  syncPauseResumeButton(state);

  // Pose: motor travel from homed (0,0,0), tracked in all states
  const pose = data.gantry_pose || {};
  if (pose.x_mm !== undefined && pose.y_mm !== undefined) {
    motorX = clamp(Number(pose.x_mm) || 0.0, 0, STROKE_X);
    motorY = clamp(Number(pose.y_mm) || 0.0, 0, STROKE_Y);
    motorZDown = clamp(Number(pose.z_mm) || 0.0, 0, STROKE_Z_MAX);
  }
  updateReadoutsFromMotor();

  setText("poseA", (pose.angle_deg ?? 0).toFixed(1));
  setText("poseGrip", pose.grip_closed ? "Closed" : "Open");

  // Load cell
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
  }

  // Calibration
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

  // Camera frame
  const imgEl = $("visionPreviewImg");
  const placeholderEl = $("camOverlayEmpty");
  if (data.vision_frame_b64) {
    imgEl.src = `data:image/jpeg;base64,${data.vision_frame_b64}`;
    placeholderEl.style.display = "none";
  } else if (!imgEl.getAttribute("src")) {
    placeholderEl.style.display = "flex";
  }

  // Live detection HUD
  const fids = data.live_fiducials || [];
  const liveBlocks = data.live_blocks || [];
  const hudEl = $("liveDetectionText");
  if (hudEl) {
    if (fids.length === 0 && liveBlocks.length === 0) {
      setText("liveDetectionText", liveCvEnabled ? "No fiducials or blocks in view" : "Eco mode — enable CV: Full for detections");
      hudEl.title = "";
    } else {
      const parts = [];
      for (const f of fids) parts.push(`ArUco #${f.id} (${f.u_px}, ${f.v_px}) ${f.angle_deg}°`);
      for (const b of liveBlocks) parts.push(`${String(b.color).toUpperCase()} (${b.u_px}, ${b.v_px}) ${b.angle_deg}°`);
      const txt = parts.join("  ·  ");
      setText("liveDetectionText", txt);
      hudEl.title = parts.join("\n");
    }
  }

  // Block catalog (only re-render when changed)
  const blocks = data.blocks || [];
  const blocksSig = JSON.stringify(blocks);
  if (blocksSig !== lastBlocksSignature) {
    lastBlocksSignature = blocksSig;
    renderBlocksTable(blocks);
  }

  // Console log — preserve scroll position unless already pinned to bottom
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

  scheduleRender();
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
        <td>${b.x_mm.toFixed(0)}, ${b.y_mm.toFixed(0)}</td>
        <td>${b.angle_deg.toFixed(0)}°</td>
        <td>${mass}</td>
        <td><span class="pill ${stacked ? "pill-success" : "pill-dim"}">${b.status}</span></td>
      </tr>`;
    })
    .join("");
}

// ============================================================================
//  DIGITAL TWIN RENDERER
// ============================================================================
// View window in plate mm (covers homed datum at Y=-43.5 up to camera reach at Y≈190)
const VIEW_MIN_X = -14.0;
const VIEW_MAX_X = 298.0;
const VIEW_MIN_Y = -56.0;
const VIEW_MAX_Y = 196.0;
const Z_VIEW_MAX = 145.0;

const L = { w: 0, h: 0, dpr: 1, scale: 1, x0: 0, y0: 0, zx: 0, zy: 0, zw: 0, zh: 0 };

function computeLayout() {
  const rect = canvasWrap.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  L.w = Math.max(200, Math.floor(rect.width));
  L.h = Math.max(200, Math.floor(rect.height));
  L.dpr = dpr;
  courseCanvas.width = Math.round(L.w * dpr);
  courseCanvas.height = Math.round(L.h * dpr);

  const pad = L.w < 600 ? 12 : 18;
  const axisPad = 22; // room for tick labels
  L.zw = clamp(Math.round(L.w * 0.21), 118, 195);
  const gap = L.w < 600 ? 12 : 18;
  const xyW = L.w - pad * 2 - L.zw - gap - axisPad;
  const xyH = L.h - pad * 2 - axisPad;

  const rangeX = VIEW_MAX_X - VIEW_MIN_X;
  const rangeY = VIEW_MAX_Y - VIEW_MIN_Y;
  L.scale = Math.max(0.1, Math.min(xyW / rangeX, xyH / rangeY));
  const plotW = rangeX * L.scale;
  const plotH = rangeY * L.scale;

  L.x0 = pad + axisPad + (xyW - plotW) / 2;
  L.y0 = pad + (xyH + plotH) / 2; // canvas y of VIEW_MIN_Y
  L.plotW = plotW;
  L.plotH = plotH;

  L.zx = L.w - pad - L.zw;
  L.zy = pad;
  L.zh = L.h - pad * 2;
}

function mmToCanvas(x_mm, y_mm) {
  return {
    cx: L.x0 + (x_mm - VIEW_MIN_X) * L.scale,
    cy: L.y0 - (y_mm - VIEW_MIN_Y) * L.scale,
    scale: L.scale,
  };
}

function canvasToMm(cx, cy) {
  return {
    x: VIEW_MIN_X + (cx - L.x0) / L.scale,
    y: VIEW_MIN_Y + (L.y0 - cy) / L.scale,
  };
}

// Project a camera pixel to plate mm (mirrors EyeInHandKinematics.pixel_to_world_mm conventions)
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

function label(text, x, y, color, opts = {}) {
  ctx.fillStyle = color;
  ctx.font = `${opts.weight || 500} ${opts.size || 10}px ${opts.mono ? THEME.mono : THEME.ui}`;
  ctx.textAlign = opts.align || "left";
  ctx.textBaseline = opts.baseline || "alphabetic";
  ctx.fillText(text, x, y);
  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";
}

let renderQueued = false;
function scheduleRender() {
  if (renderQueued) return;
  renderQueued = true;
  requestAnimationFrame(frame);
}

function frame() {
  renderQueued = false;
  // Ease rendered pose toward telemetry target
  const k = 0.28;
  drawX += (motorX - drawX) * k;
  drawY += (motorY - drawY) * k;
  drawZ += (motorZDown - drawZ) * k;
  const settling = Math.abs(motorX - drawX) > 0.05 || Math.abs(motorY - drawY) > 0.05 || Math.abs(motorZDown - drawZ) > 0.05;
  if (!settling) {
    drawX = motorX;
    drawY = motorY;
    drawZ = motorZDown;
  }
  renderCourseCanvas();
  if (settling) scheduleRender();
}

function renderCourseCanvas() {
  ctx.setTransform(L.dpr, 0, 0, L.dpr, 0, 0);
  ctx.clearRect(0, 0, L.w, L.h);
  ctx.lineCap = "butt";
  ctx.lineJoin = "miter";

  drawXYView();
  drawZView();
}

function drawXYView() {
  const o = getCamHomedOrigin();
  const mX = drawX;
  const mY = drawY;

  // --- Grid (10 mm minor, 50 mm major) with tick labels -------------------
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
    if (x % 50 === 0) label(String(x), cx, bot + 14, THEME.tick, { size: 9, mono: true, align: "center" });
  }
  for (let y = gy0; y <= VIEW_MAX_Y; y += 10) {
    const cy = Math.round(mmToCanvas(0, y).cy) + 0.5;
    ctx.strokeStyle = y % 50 === 0 ? THEME.gridMajor : THEME.grid;
    ctx.beginPath(); ctx.moveTo(left, cy); ctx.lineTo(right, cy); ctx.stroke();
    if (y % 50 === 0) label(String(y), left - 6, cy + 3, THEME.tick, { size: 9, mono: true, align: "right" });
  }

  // --- Fiducial plate & course ---------------------------------------------
  let r = rectMm(0, 0, PLATE_W_X, PLATE_H_Y);
  ctx.fillStyle = THEME.plateFill;
  ctx.fillRect(...r);
  ctx.strokeStyle = THEME.plateStroke;
  ctx.lineWidth = 1.5;
  ctx.strokeRect(...r);
  label("FIDUCIAL PLATE  284 × 167.4", r[0] + 2, r[1] - 7, THEME.label, { size: 9.5, weight: 600 });

  r = rectMm(COURSE_ORIGIN_X, COURSE_ORIGIN_Y, COURSE_ORIGIN_X + COURSE_LEN_X, COURSE_ORIGIN_Y + COURSE_WID_Y);
  ctx.fillStyle = THEME.courseFill;
  ctx.fillRect(...r);
  ctx.strokeStyle = THEME.courseStroke;
  ctx.lineWidth = 1;
  ctx.strokeRect(...r);
  label("COURSE  254 × 152.4", r[0] + 8, r[1] + 15, THEME.label, { size: 9.5, weight: 600 });

  // Plate origin
  const orig = mmToCanvas(0, 0);
  ctx.fillStyle = THEME.origin;
  ctx.beginPath(); ctx.arc(orig.cx, orig.cy, 3.5, 0, Math.PI * 2); ctx.fill();
  label("(0,0)", orig.cx - 6, orig.cy + 12, THEME.origin, { size: 9, mono: true, weight: 600, align: "right" });

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
    label(`#${cf.id}`, p.cx, p.cy + s / 2 + 11, THEME.label, { size: 8.5, mono: true, align: "center" });
  }

  // --- Reach envelopes -------------------------------------------------------
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
  label("EE REACH", eeCorners[0].cx + 6, eeCorners[0].cy - 6, "rgba(60, 207, 145, 0.85)", { size: 9, weight: 700 });

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
  label("CAM REACH", camCorners[2].cx - 6, camCorners[2].cy - 6, "rgba(79, 156, 249, 0.9)", { size: 9, weight: 700, align: "right" });

  // --- Drop zones ------------------------------------------------------------
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
    label(zone.label, p.cx, p.cy - s / 2 - 5, color, { size: 9, weight: 700, align: "center" });
  }

  // --- Camera FOV footprint (approx., scales with height above cube tops) -----
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

  // --- Live detections projected into the twin ------------------------------
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

  // --- Cataloged blocks ------------------------------------------------------
  const blocks = latestState.blocks || [];
  for (const blk of blocks) {
    const p = mmToCanvas(blk.x_mm, blk.y_mm);
    const s = BLOCK_SIZE * p.scale;
    ctx.save();
    ctx.translate(p.cx, p.cy);
    ctx.rotate((-blk.angle_deg * Math.PI) / 180.0);
    ctx.shadowColor = "rgba(0,0,0,0.5)";
    ctx.shadowBlur = 8;
    ctx.shadowOffsetY = 2;
    ctx.fillStyle = COLOR_HEX[blk.color] || "#94a3b8";
    roundRect(-s / 2, -s / 2, s, s, 2.5);
    ctx.fill();
    ctx.shadowColor = "transparent";
    ctx.strokeStyle = blk.status === "STACKED" ? THEME.ee : "rgba(255,255,255,0.25)";
    ctx.lineWidth = blk.status === "STACKED" ? 2 : 1;
    ctx.stroke();
    ctx.restore();
    label(`${blk.id}`, p.cx, p.cy + 0.5, LIGHT_BLOCKS.has(blk.color) ? "#0b0e13" : "#ffffff",
      { size: 10, weight: 700, mono: true, align: "center", baseline: "middle" });
  }

  // --- Gantry skeleton: carriage datum → end-effector → camera ---------------
  const homeDatum = mmToCanvas(DATUM_X, DATUM_Y);
  const datum = mmToCanvas(DATUM_X + mX, DATUM_Y + mY);
  const eePlate = motorCamToPlate(mX + EE_OFFSET_X, mY + EE_OFFSET_Y);
  const camPlate = motorCamToPlate(mX, mY);
  const ee = mmToCanvas(eePlate.x, eePlate.y);
  const cam = mmToCanvas(camPlate.x, camPlate.y);

  // Home datum anchor
  ctx.strokeStyle = THEME.carriage;
  ctx.globalAlpha = 0.6;
  ctx.lineWidth = 1.25;
  ctx.setLineDash([2, 2]);
  ctx.beginPath(); ctx.arc(homeDatum.cx, homeDatum.cy, 6, 0, Math.PI * 2); ctx.stroke();
  ctx.setLineDash([]);
  ctx.globalAlpha = 1;
  label("HOME", homeDatum.cx + 10, homeDatum.cy + 3.5, THEME.carriage, { size: 9, weight: 700 });

  // Gantry crosshair guides through the carriage
  ctx.strokeStyle = "rgba(245, 165, 36, 0.18)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(datum.cx, top); ctx.lineTo(datum.cx, bot);
  ctx.moveTo(left, ee.cy); ctx.lineTo(right, ee.cy);
  ctx.stroke();

  // Rigid linkage
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.strokeStyle = "rgba(245, 165, 36, 0.25)";
  ctx.lineWidth = 9;
  ctx.beginPath(); ctx.moveTo(datum.cx, datum.cy); ctx.lineTo(ee.cx, ee.cy); ctx.lineTo(cam.cx, cam.cy); ctx.stroke();
  ctx.strokeStyle = THEME.carriage;
  ctx.lineWidth = 3;
  ctx.stroke();
  ctx.lineCap = "butt";
  ctx.lineJoin = "miter";

  // Carriage joint
  ctx.fillStyle = THEME.carriage;
  ctx.beginPath(); ctx.arc(datum.cx, datum.cy, 5.5, 0, Math.PI * 2); ctx.fill();
  ctx.strokeStyle = THEME.panel;
  ctx.lineWidth = 2;
  ctx.stroke();

  // End-effector (22 × 22 mm footprint), rotated by servo angle
  const eeSide = 22 * ee.scale;
  const angle = Number(latestState.gantry_pose?.angle_deg) || 0;
  ctx.save();
  ctx.translate(ee.cx, ee.cy);
  ctx.rotate((-angle * Math.PI) / 180);
  ctx.fillStyle = "rgba(60, 207, 145, 0.22)";
  ctx.fillRect(-eeSide / 2, -eeSide / 2, eeSide, eeSide);
  ctx.strokeStyle = THEME.ee;
  ctx.lineWidth = 2;
  ctx.strokeRect(-eeSide / 2, -eeSide / 2, eeSide, eeSide);
  ctx.beginPath();
  ctx.moveTo(-4, 0); ctx.lineTo(4, 0);
  ctx.moveTo(0, -4); ctx.lineTo(0, 4);
  ctx.stroke();
  ctx.restore();
  label("EE", ee.cx + eeSide / 2 + 6, ee.cy + 3.5, THEME.ee, { size: 10, weight: 700 });

  // Camera optical centre
  ctx.fillStyle = "rgba(79, 156, 249, 0.22)";
  ctx.beginPath(); ctx.arc(cam.cx, cam.cy, 9, 0, Math.PI * 2); ctx.fill();
  ctx.strokeStyle = THEME.cam;
  ctx.lineWidth = 2;
  ctx.stroke();
  ctx.fillStyle = THEME.cam;
  ctx.beginPath(); ctx.arc(cam.cx, cam.cy, 2.5, 0, Math.PI * 2); ctx.fill();
  label("CAM", cam.cx + 13, cam.cy + 3.5, THEME.cam, { size: 10, weight: 700 });

  // --- Cursor crosshair ------------------------------------------------------
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

  // Panel
  roundRect(x, y, w, h, 10);
  ctx.fillStyle = THEME.panel;
  ctx.fill();
  ctx.strokeStyle = THEME.panelStroke;
  ctx.lineWidth = 1;
  ctx.stroke();
  label("Z ELEVATION", x + 12, y + 20, THEME.labelStrong, { size: 10, weight: 700 });

  const zTop = y + 40;
  const zBot = y + h - 26;
  const zToY = (z) => zBot - (z / Z_VIEW_MAX) * (zBot - zTop);
  const left = x + 12;
  const right = x + w - 12;
  const colX = left + 22; // carriage column

  // Scale ticks every 25 mm
  for (let z = 0; z <= Z_VIEW_MAX; z += 25) {
    const yy = Math.round(zToY(z)) + 0.5;
    ctx.strokeStyle = THEME.grid;
    ctx.beginPath(); ctx.moveTo(left, yy); ctx.lineTo(right, yy); ctx.stroke();
    label(String(z), right, yy - 3, THEME.tick, { size: 8.5, mono: true, align: "right" });
  }

  // Bed floor (Z = 0) & course floor
  const floorY = zToY(0);
  ctx.fillStyle = "#222a36";
  ctx.fillRect(left, zToY(COURSE_FLOOR_Z), right - left, floorY - zToY(COURSE_FLOOR_Z));
  ctx.strokeStyle = "#4a5568";
  ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(left, floorY); ctx.lineTo(right, floorY); ctx.stroke();
  label("BED Z0", left, floorY + 14, THEME.label, { size: 8.5, mono: true });

  // Aluminum pedestal
  const pedY = zToY(DROPOFF_Z_UP);
  const pedX = colX + 34;
  const pedW = Math.max(30, right - pedX - 4);
  ctx.fillStyle = "rgba(148, 163, 184, 0.16)";
  ctx.fillRect(pedX, pedY, pedW, floorY - pedY);
  ctx.strokeStyle = "rgba(148, 163, 184, 0.6)";
  ctx.lineWidth = 1;
  ctx.strokeRect(pedX + 0.5, pedY + 0.5, pedW - 1, floorY - pedY - 1);
  label(`ALUM ${DROPOFF_Z_UP}`, pedX + 4, pedY - 4, THEME.label, { size: 8.5, mono: true });

  // Sample 20 mm block on course floor
  const blkTop = zToY(COURSE_FLOOR_Z + BLOCK_SIZE);
  const blkBot = zToY(COURSE_FLOOR_Z);
  ctx.fillStyle = "rgba(239, 68, 68, 0.3)";
  ctx.strokeStyle = "rgba(239, 68, 68, 0.8)";
  ctx.fillRect(colX - 9, blkTop, 18, blkBot - blkTop);
  ctx.strokeRect(colX - 9 + 0.5, blkTop + 0.5, 17, blkBot - blkTop - 1);

  // Datum
  const datumY = zToY(DATUM_Z);
  ctx.strokeStyle = "rgba(245, 165, 36, 0.55)";
  ctx.setLineDash([3, 3]);
  ctx.beginPath(); ctx.moveTo(left, datumY); ctx.lineTo(right, datumY); ctx.stroke();
  ctx.setLineDash([]);
  label(`DATUM ${DATUM_Z}`, colX + 14, datumY - 4, THEME.carriage, { size: 8.5, mono: true });

  // Carriage: camera + EE
  const curCamZ = CAM_HOMED_Z - drawZ;
  const curEeZ = EE_HOMED_Z - drawZ;
  const camY = zToY(curCamZ);
  const eeY = zToY(curEeZ);

  ctx.strokeStyle = THEME.carriage;
  ctx.lineWidth = 3;
  ctx.lineCap = "round";
  ctx.beginPath(); ctx.moveTo(colX, camY); ctx.lineTo(colX, eeY); ctx.stroke();
  ctx.lineCap = "butt";

  ctx.fillStyle = THEME.cam;
  ctx.beginPath(); ctx.arc(colX, camY, 5, 0, Math.PI * 2); ctx.fill();
  label(`CAM ${curCamZ.toFixed(1)}`, colX + 12, camY + 3.5, THEME.cam, { size: 9.5, mono: true, weight: 600 });

  ctx.fillStyle = THEME.ee;
  ctx.fillRect(colX - 8, eeY, 16, 9);
  label(`EE ${curEeZ.toFixed(1)}`, colX + 12, eeY + 8, THEME.ee, { size: 9.5, mono: true, weight: 600 });
}

// Responsive canvas
function resizeCanvas() {
  computeLayout();
  renderCourseCanvas();
}
new ResizeObserver(resizeCanvas).observe(canvasWrap);
window.addEventListener("resize", resizeCanvas);

// Cursor readout
courseCanvas.addEventListener("mousemove", (e) => {
  const rect = courseCanvas.getBoundingClientRect();
  const cx = e.clientX - rect.left;
  const cy = e.clientY - rect.top;
  const p = canvasToMm(cx, cy);
  const inside = p.x >= VIEW_MIN_X && p.x <= VIEW_MAX_X && p.y >= VIEW_MIN_Y && p.y <= VIEW_MAX_Y;
  mouseMm = inside ? p : null;
  setText("cursorReadout", inside ? `${p.x.toFixed(1)}, ${p.y.toFixed(1)} mm` : "— , — mm");
  scheduleRender();
});
courseCanvas.addEventListener("mouseleave", () => {
  mouseMm = null;
  setText("cursorReadout", "— , — mm");
  scheduleRender();
});

// ============================================================================
//  DIAGNOSTIC RUN REPORTS
// ============================================================================
let cachedReportsList = [];
let activeReportId = null;

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
      <button class="recent-report-pill ${r.run_id === activeReportId ? "active" : ""}" onclick="viewReport('${r.run_id}')" title="${r.status} (${r.duration_s}s) — ${r.start_time}">
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

function showReportModal(report) {
  const modal = $("reportModal");
  const body = $("modalReportBody");
  if (!modal || !body) return;

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

// ============================================================================
//  INIT
// ============================================================================
computeLayout();
updateReadoutsFromMotor();
renderCourseCanvas();
connectWebSocket();
fetchRecentReports();
