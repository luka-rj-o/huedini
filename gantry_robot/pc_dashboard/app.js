// Tier 1 Supervisor & CAD Skeleton Visualizer for HUE-DINI Gantry Robot
// Modern Clean White Aesthetic with Real-Time Position Tracking

const wsUrlInput = document.getElementById("wsUrlInput");
const connectBtn = document.getElementById("connectBtn");
const connBadge = document.getElementById("connBadge");
const courseCanvas = document.getElementById("courseCanvas");
const ctx = courseCanvas.getContext("2d");

if (window.location.host) {
  wsUrlInput.value = `ws://${window.location.host}`;
}

// ============================================================================
//  EXACT CAD GEOMETRY & DATUM CONSTANTS (mm)
// ============================================================================
// 1. Fiducial Plate & Inner Course (Origin 0,0,0 at bottom-left of outer 284 x 167.4 mm plate)
//    Course back edge is coincident with fiducial plate back edge at Y = 0.0!
const PLATE_W_X = 284.0;
const PLATE_H_Y = 167.4;
const COURSE_ORIGIN_X = 15.0; // (284 - 254) / 2 -> X in [15.0 .. 269.0]
const COURSE_ORIGIN_Y = 0.0;  // Coincident back edge at Y = 0.0 -> Y in [0.0 .. 152.4]
const COURSE_LEN_X = 254.0;
const COURSE_WID_Y = 152.4;
const FIDUCIAL_SIZE = 15.0;
const BLOCK_SIZE = 20.0;

// 2. Motor Travel Strokes from Homed (0, 0, 0)
// Course is elevated 1/4" (6.35 mm) off base plate -> STROKE_Z_BOTTOM = 70.36 mm
const STROKE_X = 227.0;
const STROKE_Y = 128.0;
const STROKE_Z_MAX = 76.7;
const STROKE_Z_BOTTOM = 70.36; // Full descent to elevated course floor (76.71 - 6.35 mm)

// 3. Homed Datum Point (Tower Origin) in Course Coordinates (0, 0, 0)
const DATUM_X = 28.5;
const DATUM_Y = -43.5;
const DATUM_Z = 127.5;

// 4. Nominal Homed End-Effector in Course Coordinates
const EE_HOMED_X = 32.5;
const EE_HOMED_Y = -6.0;
const EE_HOMED_Z = 97.6;

// 5. Nominal Homed Camera in Course Coordinates (54.1 mm ahead, 37.8 mm above EE)
const CAM_HOMED_X = 32.5;
const CAM_HOMED_Y = 48.1;
const CAM_HOMED_Z = 135.4;
const EE_OFFSET_X = 0.0;
const EE_OFFSET_Y = -54.1;

// 6. Drop-off Platform wrt Fiducial Plate Origin (0, 0, 0)
const DROPOFF_X = 224.5;
const DROPOFF_Y = 44.5;
const DROPOFF_Z_UP = 12.6;

// Corner fiducials in Ground-Truth Plate Coordinates
const CORNER_FIDUCIALS = [
  { id: 0, x: 7.5, y: 7.5 },
  { id: 1, x: 7.5, y: 159.9 },
  { id: 2, x: 276.5, y: 159.9 },
  { id: 3, x: 276.5, y: 7.5 },
];

const COLOR_HEX = {
  white: "#f8fafc",
  red: "#ef4444",
  green: "#16a34a",
  blue: "#2563eb",
  yellow: "#ca8a04",
  aluminum: "#64748b",
  reject: "#9333ea",
};

const STACK_ZONES = [
  { label: "DROP-OFF (224.5, 44.5, Z+12.6)", color: "green", x: DROPOFF_X, y: DROPOFF_Y },
  { label: "REJECT", color: "reject", x: 55.0, y: 44.5 },
];

// Current motor travel coordinates from homed (0,0,0)
let motorX = 0.0;
let motorY = 0.0;
let motorZDown = 0.0;

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

function setJogStep(stepMm) {
  currentJogStepMm = stepMm;
  document.querySelectorAll(".jog-step-btn").forEach((btn) => {
    btn.classList.remove("active", "btn-primary");
    btn.classList.add("btn-secondary");
  });
  const id = `jogStep${Math.round(stepMm)}`;
  const activeBtn = document.getElementById(id);
  if (activeBtn) {
    activeBtn.classList.remove("btn-secondary");
    activeBtn.classList.add("active", "btn-primary");
  }
}

function jogAxis(axis, dir) {
  const uiDelta = dir * currentJogStepMm;
  // Physical Z motor travel counts downward from top home (0 = top, +Z_down = lower),
  // while GUI jog buttons use +Z = Up and -Z = Down.
  const motorDelta = axis === "Z" ? -uiDelta : uiDelta;
  if (socket && socket.readyState === WebSocket.OPEN) {
    // 1. Instantaneous Optimistic UI update (0ms perceived latency on client)
    if (axis === "X") {
      motorX = Math.max(0, Math.min(STROKE_X, motorX + motorDelta));
    } else if (axis === "Y") {
      motorY = Math.max(0, Math.min(STROKE_Y, motorY + motorDelta));
    } else if (axis === "Z") {
      motorZDown = Math.max(0, Math.min(STROKE_Z_BOTTOM, motorZDown + motorDelta));
    }
    updateReadoutsFromMotor();
    renderCourseCanvas();

    // 2. Dispatch non-blocking motion command to Pi controller
    socket.send(JSON.stringify({ command: "JOG_AXIS", axis, dist_mm: motorDelta }));
  }
}

// Preset selection for Lead Screw Calibration
function applyLeadPreset(leadMm, spm) {
  document.querySelectorAll(".preset-btn").forEach(b => b.classList.remove("active"));
  const btn = document.getElementById(`presetT8x${Math.round(leadMm)}`);
  if (btn) btn.classList.add("active");

  const tag = document.getElementById("currentSpmTag");
  if (tag) tag.textContent = `X/Z: ${spm.toFixed(1)} • Y: 10.0 spm`;

  if (socket && socket.readyState === WebSocket.OPEN) {
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
  const input = prompt("Enter steps/mm for X and Z lead screws (e.g. 50.0 for T8x8, 100.0 for T8x4, 200.0 for T8x2):", curX.toFixed(1));
  if (input !== null) {
    const val = parseFloat(input);
    if (!isNaN(val) && val > 0) {
      applyLeadPreset(0, val);
      const tag = document.getElementById("currentSpmTag");
      if (tag) tag.textContent = `X/Z: ${val.toFixed(1)} • Y: 10.0 spm`;
    }
  }
}

function getCamHomedOrigin() {
  const cal = latestState.calibration || {};
  const ox = (cal.cam_origin_x_mm !== undefined && cal.cam_origin_x_mm !== null) ? Number(cal.cam_origin_x_mm) : CAM_HOMED_X;
  const oy = (cal.cam_origin_y_mm !== undefined && cal.cam_origin_y_mm !== null) ? Number(cal.cam_origin_y_mm) : CAM_HOMED_Y;
  return {
    x: ox,
    y: oy,
    calibrated: !!cal.origin_calibrated,
  };
}

function updateReadoutsFromMotor() {
  const camOrigin = getCamHomedOrigin();

  // Camera in Fiducial Plate Coordinates
  const camX = camOrigin.x + motorX;
  const camY = camOrigin.y + motorY;
  const camZ = CAM_HOMED_Z - motorZDown;

  // End-Effector in Course Coordinates (offset: 0.0 mm X, -54.1 mm Y from camera)
  const eeX = camOrigin.x + EE_OFFSET_X + motorX;
  const eeY = camOrigin.y + EE_OFFSET_Y + motorY;
  const eeZ = EE_HOMED_Z - motorZDown;

  const eeEl = document.getElementById("eePlateReadout");
  if (eeEl) eeEl.textContent = `(${eeX.toFixed(1)}, ${eeY.toFixed(1)}, ${eeZ.toFixed(1)})`;

  const camEl = document.getElementById("camPlateReadout");
  if (camEl) camEl.textContent = `(${camX.toFixed(1)}, ${camY.toFixed(1)}, ${camZ.toFixed(1)})`;

  const camHomedEl = document.getElementById("camHomedOriginReadout");
  if (camHomedEl) camHomedEl.textContent = `(${camOrigin.x.toFixed(1)}, ${camOrigin.y.toFixed(1)}, ${CAM_HOMED_Z.toFixed(1)}) mm`;

  const eeHomedEl = document.getElementById("eeHomedOriginReadout");
  if (eeHomedEl) eeHomedEl.textContent = `(${(camOrigin.x + EE_OFFSET_X).toFixed(1)}, ${(camOrigin.y + EE_OFFSET_Y).toFixed(1)}, ${EE_HOMED_Z.toFixed(1)}) mm`;

  // Live Motor Telemetry Readouts (display Z as negative displacement from top home so +Z is Up)
  const mxEl = document.getElementById("motorXReadout");
  if (mxEl) mxEl.textContent = motorX.toFixed(1);

  const myEl = document.getElementById("motorYReadout");
  if (myEl) myEl.textContent = motorY.toFixed(1);

  const mzEl = document.getElementById("motorZReadout");
  const displayZ = motorZDown > 0.005 ? -motorZDown : 0.0;
  if (mzEl) mzEl.textContent = displayZ.toFixed(2);

  // Progress Bar Visual Fill
  const mxBar = document.getElementById("motorXBar");
  if (mxBar) mxBar.style.width = `${Math.min(100, Math.max(0, (motorX / STROKE_X) * 100))}%`;

  const myBar = document.getElementById("motorYBar");
  if (myBar) myBar.style.width = `${Math.min(100, Math.max(0, (motorY / STROKE_Y) * 100))}%`;

  const mzBar = document.getElementById("motorZBar");
  if (mzBar) mzBar.style.width = `${Math.min(100, Math.max(0, (motorZDown / STROKE_Z_BOTTOM) * 100))}%`;
}

let reconnectTimer = null;

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
  socket = new WebSocket(url);

  socket.onopen = () => {
    connBadge.textContent = "CONNECTED";
    connBadge.className = "badge badge-connected";
  };

  socket.onclose = () => {
    connBadge.textContent = "RECONNECTING...";
    connBadge.className = "badge badge-disconnected";
    if (!reconnectTimer) {
      reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        connectWebSocket();
      }, 2500);
    }
  };

  socket.onerror = () => {
    // onclose triggers automatic reconnection
  };

  socket.onmessage = (event) => {
    try {
      const payload = JSON.parse(event.data);
      latestState = payload;
      updateUI(payload);
    } catch (err) {
      console.error("Invalid telemetry JSON:", err);
    }
  };
}

connectBtn.addEventListener("click", connectWebSocket);
document.getElementById("rawSerialInput").addEventListener("keydown", (e) => {
  if (e.key === "Enter") sendRawSerial();
});

function sendCmd(command) {
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify({ command }));
  }
}

let liveStreamActive = true;
function toggleLiveStream() {
  liveStreamActive = !liveStreamActive;
  sendCmd("TOGGLE_LIVE_STREAM");
  const btn = document.getElementById("toggleLiveBtn");
  const pill = document.getElementById("liveCamPill");
  if (btn) {
    btn.textContent = liveStreamActive ? "Stream: Active" : "Stream: Paused";
    btn.className = liveStreamActive ? "btn btn-primary btn-sm" : "btn btn-secondary btn-sm";
  }
  if (pill) {
    pill.textContent = liveStreamActive ? "• LIVE (~3 FPS)" : "PAUSED";
    pill.className = liveStreamActive ? "badge badge-connected" : "badge badge-disconnected";
  }
}

let liveCvEnabled = false;
function toggleLiveCv() {
  liveCvEnabled = !liveCvEnabled;
  sendCmd("TOGGLE_LIVE_CV");
  const btn = document.getElementById("toggleLiveCvBtn");
  if (btn) {
    btn.textContent = liveCvEnabled ? "CV: Full" : "CV: Eco (Fast)";
    btn.className = liveCvEnabled ? "btn btn-primary btn-sm" : "btn btn-secondary btn-sm";
  }
}

function sendTareCmd() {
  const massEl = document.getElementById("liveMassReadout");
  if (massEl) massEl.textContent = "0.00";
  const poseMass = document.getElementById("poseMass");
  if (poseMass) poseMass.textContent = "0.0 g";
  const pill = document.getElementById("massClassPill");
  if (pill) {
    pill.textContent = "TARED";
    pill.className = "badge badge-dim";
  }
  sendCmd("TARE_SCALE");
}

function sendWeighCmd() {
  const massEl = document.getElementById("liveMassReadout");
  if (massEl) massEl.textContent = "...";
  const pill = document.getElementById("massClassPill");
  if (pill) {
    pill.textContent = "SAMPLING";
    pill.className = "badge badge-accent";
  }
  sendCmd("WEIGH_BLOCK");
}

function sendRawSerial() {
  const input = document.getElementById("rawSerialInput");
  const line = input.value.trim();
  if (!line) return;
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify({ command: "RAW_SERIAL", line }));
    input.value = "";
  }
}

function clearLogs() {
  const logEl = document.getElementById("logConsole");
  if (logEl) logEl.textContent = "";
}

function updateUI(data) {
  const prevState = latestState ? latestState.state : "IDLE";
  if (prevState !== data.state && (data.state === "COMPLETE" || data.state === "ERROR")) {
    setTimeout(fetchRecentReports, 800);
  }

  const stateEl = document.getElementById("stateReadout");
  if (stateEl) stateEl.textContent = data.state || "IDLE";

  const pose = data.gantry_pose || {};

  // ACCURATE POSITION TRACKING:
  // pose.x_mm, pose.y_mm, pose.z_mm represent motor travel from homed (0,0,0).
  // Update in ALL machine states (IDLE, CALIBRATING, SCANNING, MOVING, etc.)
  if (pose.x_mm !== undefined && pose.y_mm !== undefined) {
    motorX = Math.max(0, Math.min(STROKE_X, Number(pose.x_mm) || 0.0));
    motorY = Math.max(0, Math.min(STROKE_Y, Number(pose.y_mm) || 0.0));
    motorZDown = Math.max(0, Math.min(STROKE_Z_MAX, Number(pose.z_mm) || 0.0));
  }

  updateReadoutsFromMotor();

  const poseAEl = document.getElementById("poseA");
  if (poseAEl) poseAEl.textContent = (pose.angle_deg ?? 0).toFixed(1);

  const poseGripEl = document.getElementById("poseGrip");
  if (poseGripEl) poseGripEl.textContent = pose.grip_closed ? "CLOSED (NC)" : "OPEN";

  // Live Mass Readout from HX711 Load Cell
  const massVal = (data.current_mass_g !== undefined && data.current_mass_g !== null)
    ? Number(data.current_mass_g)
    : null;

  const massEl = document.getElementById("liveMassReadout");
  const poseMassEl = document.getElementById("poseMass");
  const massPill = document.getElementById("massClassPill");
  const massSub = document.getElementById("massStatusSub");

  if (massVal !== null) {
    const massStr = massVal.toFixed(2);
    if (massEl) massEl.textContent = massStr;
    if (poseMassEl) poseMassEl.textContent = `${massStr} g`;

    if (massPill) {
      const cls = (data.mass_class || "measured").toLowerCase();
      let label = cls.replace("_", " ").toUpperCase();
      if (cls === "low_infill") label = "LOW (5g)";
      else if (cls === "med_infill") label = "MED (10g)";
      else if (cls === "high_infill") label = "HIGH (15g)";
      else if (cls === "aluminum") label = "ALUM (21.6g)";
      else if (cls === "reject") label = "REJECT";

      massPill.textContent = label;
      massPill.className = `badge badge-${cls}`;

      if (massSub) {
        if (cls === "low_infill") massSub.textContent = "Low Infill (2.0 – 7.5g)";
        else if (cls === "med_infill") massSub.textContent = "Medium Infill (7.5 – 12.5g)";
        else if (cls === "high_infill") massSub.textContent = "High Infill (12.5 – 18.3g)";
        else if (cls === "aluminum") massSub.textContent = "Solid Aluminum (18.3 – 35.0g)";
        else if (cls === "reject") massSub.textContent = "Out of Range (<2g or >35g)";
        else massSub.textContent = "Bins: ≤7.5g • 10g • 15g • ≥18.3g";
      }
    }
  }

  const cal = data.calibration || {};
  const calStatusEl = document.getElementById("calStatus");
  if (calStatusEl) calStatusEl.textContent = cal.calibrated ? "CALIBRATED" : "Default";

  const calOriginStatusEl = document.getElementById("calOriginStatus");
  if (calOriginStatusEl) {
    if (cal.origin_calibrated) {
      calOriginStatusEl.textContent = "Fiducial Calibrated";
      calOriginStatusEl.className = "badge badge-connected";
    } else {
      calOriginStatusEl.textContent = "CAD Nominal";
      calOriginStatusEl.className = "badge badge-dim";
    }
  }

  const calKxEl = document.getElementById("calKx");
  if (calKxEl) calKxEl.textContent = (cal.kx_mm_per_px ?? 0.25).toFixed(4);

  const calKyEl = document.getElementById("calKy");
  if (calKyEl) calKyEl.textContent = (cal.ky_mm_per_px ?? 0.25).toFixed(4);

  const calCamSkewEl = document.getElementById("calCamSkew");
  if (calCamSkewEl) {
    const skew = Number(cal.cam_skew_deg) || 0.0;
    calCamSkewEl.textContent = `${skew >= 0 ? "+" : ""}${skew.toFixed(1)}°`;
  }

  const calSkewStatusEl = document.getElementById("calSkewStatus");
  if (calSkewStatusEl) {
    if (Math.abs(cal.cam_skew_deg || 0) > 0.05) {
      calSkewStatusEl.textContent = "Compensated";
      calSkewStatusEl.className = "badge badge-connected";
    } else {
      calSkewStatusEl.textContent = "Nominal";
      calSkewStatusEl.className = "badge badge-dim";
    }
  }

  // Update active steps/mm tag if received
  if (data.steps_per_mm) {
    const spmTag = document.getElementById("currentSpmTag");
    if (spmTag) {
      spmTag.textContent = `X/Z: ${Number(data.steps_per_mm.x).toFixed(1)} • Y: ${Number(data.steps_per_mm.y).toFixed(1)} spm`;
    }
  }

  // Update Live CV mode button state
  if (data.live_cv_enabled !== undefined) {
    liveCvEnabled = !!data.live_cv_enabled;
    const cvBtn = document.getElementById("toggleLiveCvBtn");
    if (cvBtn) {
      cvBtn.textContent = liveCvEnabled ? "CV: Full" : "CV: Eco (Fast)";
      cvBtn.className = liveCvEnabled ? "btn btn-primary btn-sm" : "btn btn-secondary btn-sm";
    }
  }

  // Camera preview handling
  const imgEl = document.getElementById("visionPreviewImg");
  const placeholderEl = document.getElementById("camOverlayEmpty");
  if (data.vision_frame_b64) {
    imgEl.src = `data:image/jpeg;base64,${data.vision_frame_b64}`;
    if (placeholderEl) placeholderEl.style.display = "none";
  } else {
    if (placeholderEl && !imgEl.src) placeholderEl.style.display = "flex";
  }

  // Live fiducials and blocks readout HUD
  const hudEl = document.getElementById("liveDetectionText");
  if (hudEl) {
    const fids = data.live_fiducials || [];
    const liveBlocks = data.live_blocks || [];
    if (fids.length === 0 && liveBlocks.length === 0) {
      hudEl.textContent = "No fiducials or blocks in view";
    } else {
      const parts = [];
      for (const f of fids) {
        parts.push(`ARUCO #${f.id} @ (${f.u_px}, ${f.v_px}) px, θ=${f.angle_deg}°`);
      }
      for (const b of liveBlocks) {
        parts.push(`${b.color.toUpperCase()} @ (${b.u_px}, ${b.v_px}) px, θ=${b.angle_deg}°`);
      }
      hudEl.textContent = parts.join("   |   ");
    }
  }

  // Detected Block Catalog Table
  const blocks = data.blocks || [];
  const countBadge = document.getElementById("blockCountBadge");
  if (countBadge) countBadge.textContent = `${blocks.length} Blocks`;

  const tbody = document.getElementById("blocksTableBody");
  if (tbody) {
    if (blocks.length === 0) {
      tbody.innerHTML = `<tr><td colspan="6" class="empty-cell">No blocks cataloged yet. Click START SCAN.</td></tr>`;
    } else {
      tbody.innerHTML = blocks
        .map(
          (b) => `
        <tr>
          <td><strong>#${b.id}</strong></td>
          <td><span style="color:${COLOR_HEX[b.color] || "#0f172a"};font-weight:700">${b.color.toUpperCase()}</span></td>
          <td>(${b.x_mm.toFixed(1)}, ${b.y_mm.toFixed(1)}) mm</td>
          <td>${b.angle_deg.toFixed(1)}&deg;</td>
          <td>${b.mass_g > 0 ? b.mass_g.toFixed(1) + "g (" + (b.mass_class || "") + ")" : "&mdash;"}</td>
          <td><span class="badge ${b.status === "STACKED" ? "badge-connected" : "badge-dim"}">${b.status}</span></td>
        </tr>`
        )
        .join("");
    }
  }

  // Telemetry Console Logs
  if (Array.isArray(data.logs) && data.logs.length > 0) {
    const logEl = document.getElementById("logConsole");
    if (logEl) {
      logEl.textContent = data.logs.join("\n");
      logEl.scrollTop = logEl.scrollHeight;
    }
  }

  renderCourseCanvas();
}

// Coordinate bounding box for Top-Down XY view (includes Y = -55 mm for Homed Datum up to Y = +198 mm for Camera reach)
const VIEW_MIN_X = -15.0;
const VIEW_MAX_X = 300.0;
const VIEW_MIN_Y = -60.0;
const VIEW_MAX_Y = 198.0;

function mmToCanvas(x_mm, y_mm) {
  const leftPad = 38;
  const topPad = 24;
  const xyPlotW = 635; // Reserve right 185px for Z-Axis Elevation View
  const xyPlotH = courseCanvas.height - topPad - 28;

  const scaleX = xyPlotW / (VIEW_MAX_X - VIEW_MIN_X);
  const scaleY = xyPlotH / (VIEW_MAX_Y - VIEW_MIN_Y);
  const scale = Math.min(scaleX, scaleY);

  const cx = leftPad + (x_mm - VIEW_MIN_X) * scale;
  const cy = courseCanvas.height - 28 - (y_mm - VIEW_MIN_Y) * scale;
  return { cx, cy, scale };
}

function renderCourseCanvas() {
  ctx.clearRect(0, 0, courseCanvas.width, courseCanvas.height);

  // Fill canvas with clean crisp white
  ctx.fillStyle = "#ffffff";
  ctx.fillRect(0, 0, courseCanvas.width, courseCanvas.height);

  // ==========================================================================
  // PART A: TOP-DOWN XY CAD SKELETON & ENVELOPE (Clean White Aesthetic)
  // ==========================================================================

  // 1. Subtle Engineering Grid (20mm grid spacing)
  ctx.strokeStyle = "#f1f5f9";
  ctx.lineWidth = 1;
  for (let x = 0; x <= PLATE_W_X; x += 20) {
    const p0 = mmToCanvas(x, VIEW_MIN_Y);
    const p1 = mmToCanvas(x, VIEW_MAX_Y);
    ctx.beginPath();
    ctx.moveTo(p0.cx, p0.cy);
    ctx.lineTo(p1.cx, p1.cy);
    ctx.stroke();
  }
  for (let y = -40; y <= 180; y += 20) {
    const p0 = mmToCanvas(VIEW_MIN_X, y);
    const p1 = mmToCanvas(VIEW_MAX_X, y);
    ctx.beginPath();
    ctx.moveTo(p0.cx, p0.cy);
    ctx.lineTo(p1.cx, p1.cy);
    ctx.stroke();
  }

  // 2. Outer Fiducial Plate (284 x 167.4 mm, Origin 0,0 at bottom-left)
  const outerTL = mmToCanvas(0, PLATE_H_Y);
  const outerBR = mmToCanvas(PLATE_W_X, 0);
  ctx.fillStyle = "#f8fafc";
  ctx.fillRect(outerTL.cx, outerTL.cy, outerBR.cx - outerTL.cx, outerBR.cy - outerTL.cy);
  ctx.strokeStyle = "#cbd5e1";
  ctx.lineWidth = 2;
  ctx.strokeRect(outerTL.cx, outerTL.cy, outerBR.cx - outerTL.cx, outerBR.cy - outerTL.cy);

  // Plate label
  ctx.fillStyle = "#94a3b8";
  ctx.font = "600 10px -apple-system, BlinkMacSystemFont, sans-serif";
  ctx.fillText("Fiducial Plate 284 × 167.4 mm", outerTL.cx + 8, outerTL.cy - 6);

  // Plate Origin (0, 0, 0) marker
  const orig = mmToCanvas(0, 0);
  ctx.fillStyle = "#e11d48";
  ctx.beginPath();
  ctx.arc(orig.cx, orig.cy, 5, 0, Math.PI * 2);
  ctx.fill();
  ctx.font = "bold 9px monospace";
  ctx.fillText("PLATE (0,0,0)", orig.cx - 14, orig.cy + 14);

  // 3. Inner Playable Course (254 x 152.4 mm)
  const innerTL = mmToCanvas(COURSE_ORIGIN_X, COURSE_ORIGIN_Y + COURSE_WID_Y);
  const innerBR = mmToCanvas(COURSE_ORIGIN_X + COURSE_LEN_X, COURSE_ORIGIN_Y);
  ctx.fillStyle = "#ffffff";
  ctx.fillRect(innerTL.cx, innerTL.cy, innerBR.cx - innerTL.cx, innerBR.cy - innerTL.cy);
  ctx.strokeStyle = "#94a3b8";
  ctx.lineWidth = 1.5;
  ctx.strokeRect(innerTL.cx, innerTL.cy, innerBR.cx - innerTL.cx, innerBR.cy - innerTL.cy);
  ctx.fillStyle = "#64748b";
  ctx.font = "10px monospace";
  ctx.fillText("Inner Course 254 × 152.4 mm", innerTL.cx + 8, innerTL.cy + 14);

  // 4. Four Corner Fiducials (15 x 15 mm squares)
  for (const cf of CORNER_FIDUCIALS) {
    const fid = mmToCanvas(cf.x, cf.y);
    const fpx = FIDUCIAL_SIZE * fid.scale;
    // Outer black square
    ctx.fillStyle = "#0f172a";
    ctx.fillRect(fid.cx - fpx / 2, fid.cy - fpx / 2, fpx, fpx);
    // Inner white square
    ctx.fillStyle = "#ffffff";
    ctx.fillRect(fid.cx - fpx * 0.3, fid.cy - fpx * 0.3, fpx * 0.6, fpx * 0.6);
    // Center black dot
    ctx.fillStyle = "#0f172a";
    ctx.fillRect(fid.cx - fpx * 0.12, fid.cy - fpx * 0.12, fpx * 0.24, fpx * 0.24);
  }

  // 5. End-Effector Reachable Travel Envelope
  const camOrigin = getCamHomedOrigin();
  const curEeHomedX = camOrigin.x + EE_OFFSET_X;
  const curEeHomedY = camOrigin.y + EE_OFFSET_Y;
  const eeBoxTL = mmToCanvas(curEeHomedX, curEeHomedY + STROKE_Y);
  const eeBoxBR = mmToCanvas(curEeHomedX + STROKE_X, curEeHomedY);
  ctx.fillStyle = "rgba(22, 163, 74, 0.04)";
  ctx.fillRect(eeBoxTL.cx, eeBoxTL.cy, eeBoxBR.cx - eeBoxTL.cx, eeBoxBR.cy - eeBoxTL.cy);
  ctx.strokeStyle = "#16a34a";
  ctx.lineWidth = 1.5;
  ctx.setLineDash([5, 4]);
  ctx.strokeRect(eeBoxTL.cx, eeBoxTL.cy, eeBoxBR.cx - eeBoxTL.cx, eeBoxBR.cy - eeBoxTL.cy);
  ctx.setLineDash([]);
  ctx.fillStyle = "#15803d";
  ctx.font = "600 9.5px monospace";
  ctx.fillText(
    `EE Reach Envelope: X[${curEeHomedX.toFixed(1)}..${(curEeHomedX + STROKE_X).toFixed(1)}] Y[${curEeHomedY.toFixed(1)}..${(curEeHomedY + STROKE_Y).toFixed(1)}]`,
    eeBoxTL.cx + 6,
    eeBoxBR.cy - 6
  );

  // 6. Camera Reachable Travel Envelope
  const curCamHomedX = camOrigin.x;
  const curCamHomedY = camOrigin.y;
  const camBoxTL = mmToCanvas(curCamHomedX, curCamHomedY + STROKE_Y);
  const camBoxBR = mmToCanvas(curCamHomedX + STROKE_X, curCamHomedY);
  ctx.fillStyle = "rgba(2, 132, 199, 0.03)";
  ctx.fillRect(camBoxTL.cx, camBoxTL.cy, camBoxBR.cx - camBoxTL.cx, camBoxBR.cy - camBoxTL.cy);
  ctx.strokeStyle = "#0284c7";
  ctx.lineWidth = 1.5;
  ctx.setLineDash([3, 3]);
  ctx.strokeRect(camBoxTL.cx, camBoxTL.cy, camBoxBR.cx - camBoxTL.cx, camBoxBR.cy - camBoxTL.cy);
  ctx.setLineDash([]);
  ctx.fillStyle = "#0369a1";
  ctx.fillText(
    `Cam Reach Envelope: X[${curCamHomedX.toFixed(1)}..${(curCamHomedX + STROKE_X).toFixed(1)}] Y[${curCamHomedY.toFixed(1)}..${(curCamHomedY + STROKE_Y).toFixed(1)}]`,
    camBoxTL.cx + 6,
    camBoxTL.cy - 5
  );

  // 7. Homed Datum Origin (28.5, -43.5, 127.5) & Current Carriage Datum
  const curDatumX = DATUM_X + motorX;
  const curDatumY = DATUM_Y + motorY;
  const homeDatumPt = mmToCanvas(DATUM_X, DATUM_Y);
  const curDatumPt = mmToCanvas(curDatumX, curDatumY);

  // Home datum anchor
  ctx.strokeStyle = "#d97706";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(homeDatumPt.cx, homeDatumPt.cy, 6, 0, Math.PI * 2);
  ctx.stroke();
  ctx.fillStyle = "#d97706";
  ctx.font = "bold 9px monospace";
  ctx.fillText(`HOMED DATUM (${DATUM_X.toFixed(1)}, ${DATUM_Y.toFixed(1)})`, homeDatumPt.cx + 9, homeDatumPt.cy + 3);

  // 8. Draw Drop-Off & Reject Zones
  for (const zone of STACK_ZONES) {
    const zp = mmToCanvas(zone.x, zone.y);
    const bpx = BLOCK_SIZE * zp.scale;
    const color = COLOR_HEX[zone.color] || "#64748b";
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.setLineDash([3, 3]);
    ctx.strokeRect(zp.cx - bpx / 2, zp.cy - bpx / 2, bpx, bpx);
    ctx.setLineDash([]);
    ctx.fillStyle = color;
    ctx.font = "600 8.5px -apple-system, sans-serif";
    ctx.fillText(zone.label, zp.cx - 28, zp.cy - bpx / 2 - 4);
  }

  // 9. Draw Cataloged 20x20 mm Blocks
  const blocks = latestState.blocks || [];
  for (const blk of blocks) {
    const bp = mmToCanvas(blk.x_mm, blk.y_mm);
    const sidePx = BLOCK_SIZE * bp.scale;

    ctx.save();
    ctx.translate(bp.cx, bp.cy);
    ctx.rotate((-blk.angle_deg * Math.PI) / 180.0);

    // Block shadow
    ctx.fillStyle = "rgba(0, 0, 0, 0.15)";
    ctx.fillRect(-sidePx / 2 + 2, -sidePx / 2 + 2, sidePx, sidePx);

    // Block body
    ctx.fillStyle = COLOR_HEX[blk.color] || "#94a3b8";
    ctx.fillRect(-sidePx / 2, -sidePx / 2, sidePx, sidePx);
    ctx.strokeStyle = "#0f172a";
    ctx.lineWidth = 1.5;
    ctx.strokeRect(-sidePx / 2, -sidePx / 2, sidePx, sidePx);
    ctx.restore();

    ctx.fillStyle = "#0f172a";
    ctx.font = "bold 10px monospace";
    ctx.fillText(`#${blk.id}`, bp.cx - 7, bp.cy + 3);
  }

  // 10. CURRENT CAD SKELETON LINK (The Star Focus of the GUI!)
  // Links: Carriage Datum -> Square End-Effector -> Camera
  const eePt = mmToCanvas(curEeHomedX + motorX, curEeHomedY + motorY);
  const camPt = mmToCanvas(curCamHomedX + motorX, curCamHomedY + motorY);

  // Rigid gantry bridge linkage (gold/amber with dark edge)
  ctx.strokeStyle = "#d97706";
  ctx.lineWidth = 4.5;
  ctx.lineCap = "round";
  ctx.beginPath();
  ctx.moveTo(curDatumPt.cx, curDatumPt.cy);
  ctx.lineTo(eePt.cx, eePt.cy);
  ctx.lineTo(camPt.cx, camPt.cy);
  ctx.stroke();

  // Highlight centerline
  ctx.strokeStyle = "#fef08a";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(curDatumPt.cx, curDatumPt.cy);
  ctx.lineTo(eePt.cx, eePt.cy);
  ctx.lineTo(camPt.cx, camPt.cy);
  ctx.stroke();

  // Carriage Datum Joint
  ctx.fillStyle = "#ea580c";
  ctx.beginPath();
  ctx.arc(curDatumPt.cx, curDatumPt.cy, 6, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 2;
  ctx.stroke();

  // Draw Square End-Effector (22x22 mm footprint) at eePt
  const eeSide = 22 * eePt.scale;
  ctx.fillStyle = "rgba(22, 163, 74, 0.25)";
  ctx.fillRect(eePt.cx - eeSide / 2, eePt.cy - eeSide / 2, eeSide, eeSide);
  ctx.strokeStyle = "#16a34a";
  ctx.lineWidth = 2.5;
  ctx.strokeRect(eePt.cx - eeSide / 2, eePt.cy - eeSide / 2, eeSide, eeSide);
  // Center crosshair
  ctx.beginPath();
  ctx.moveTo(eePt.cx - 4, eePt.cy);
  ctx.lineTo(eePt.cx + 4, eePt.cy);
  ctx.moveTo(eePt.cx, eePt.cy - 4);
  ctx.lineTo(eePt.cx, eePt.cy + 4);
  ctx.stroke();
  ctx.fillStyle = "#15803d";
  ctx.font = "bold 10px monospace";
  ctx.fillText("EE", eePt.cx - 6, eePt.cy - eeSide / 2 - 4);

  // Draw Camera Optical Center & FOV at camPt
  ctx.fillStyle = "rgba(2, 132, 199, 0.2)";
  ctx.beginPath();
  ctx.arc(camPt.cx, camPt.cy, 10, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = "#0284c7";
  ctx.lineWidth = 2.5;
  ctx.stroke();
  // Camera optical reticle
  ctx.beginPath();
  ctx.arc(camPt.cx, camPt.cy, 4, 0, Math.PI * 2);
  ctx.stroke();
  ctx.fillStyle = "#0369a1";
  ctx.font = "bold 10px monospace";
  ctx.fillText("CAM", camPt.cx + 14, camPt.cy + 4);

  // ==========================================================================
  // PART B: SIDE-VIEW Z-AXIS ELEVATION DIAGRAM (Right Panel of Canvas)
  // ==========================================================================
  const zColX = 715;
  const zTopY = 36;
  const zBotY = courseCanvas.height - 36;
  const maxZMm = 145.0;

  function zMmToY(z_mm) {
    return zBotY - (z_mm / maxZMm) * (zBotY - zTopY);
  }

  // Z Panel card background
  ctx.fillStyle = "#f8fafc";
  ctx.strokeStyle = "#cbd5e1";
  ctx.lineWidth = 1;
  ctx.fillRect(zColX - 30, zTopY - 16, 165, zBotY - zTopY + 32);
  ctx.strokeRect(zColX - 30, zTopY - 16, 165, zBotY - zTopY + 32);

  ctx.fillStyle = "#0f172a";
  ctx.font = "bold 11px -apple-system, BlinkMacSystemFont, sans-serif";
  ctx.fillText("Z Elevation Profile", zColX - 20, zTopY - 2);

  // Course Floor (Z = 0 mm)
  const floorY = zMmToY(0);
  ctx.strokeStyle = "#475569";
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.moveTo(zColX - 20, floorY);
  ctx.lineTo(zColX + 125, floorY);
  ctx.stroke();
  ctx.fillStyle = "#64748b";
  ctx.font = "600 9.5px monospace";
  ctx.fillText("Z=0 Bed Floor", zColX + 22, floorY - 5);

  // Drop-off Platform at Z = +12.6 mm
  const dropOffZY = zMmToY(DROPOFF_Z_UP);
  ctx.fillStyle = "rgba(22, 163, 74, 0.2)";
  ctx.fillRect(zColX + 20, dropOffZY, 95, floorY - dropOffZY);
  ctx.strokeStyle = "#16a34a";
  ctx.lineWidth = 1.5;
  ctx.strokeRect(zColX + 20, dropOffZY, 95, floorY - dropOffZY);
  ctx.fillStyle = "#15803d";
  ctx.font = "600 9px monospace";
  ctx.fillText("Drop Z=+12.6", zColX + 26, dropOffZY - 4);

  // 20mm Block on Floor
  const blkTopY = zMmToY(20.0);
  ctx.fillStyle = "rgba(239, 68, 68, 0.35)";
  ctx.fillRect(zColX - 10, blkTopY, 20, floorY - blkTopY);
  ctx.strokeStyle = "#ef4444";
  ctx.lineWidth = 1.5;
  ctx.strokeRect(zColX - 10, blkTopY, 20, floorY - blkTopY);

  // Homed Datum Z (127.5 mm)
  const datumZY = zMmToY(DATUM_Z);
  ctx.strokeStyle = "#d97706";
  ctx.lineWidth = 1;
  ctx.setLineDash([3, 3]);
  ctx.beginPath();
  ctx.moveTo(zColX - 20, datumZY);
  ctx.lineTo(zColX + 125, datumZY);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = "#d97706";
  ctx.font = "600 9px monospace";
  ctx.fillText("Datum Z=127.5", zColX + 26, datumZY - 4);

  // Current Camera Z & EE Top Z
  const curCamZ = CAM_HOMED_Z - motorZDown;
  const curEeZ = EE_HOMED_Z - motorZDown;
  const camZY = zMmToY(curCamZ);
  const eeZY = zMmToY(curEeZ);

  // Vertical Z carriage bar (gold/amber)
  ctx.strokeStyle = "#d97706";
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.moveTo(zColX + 2, camZY);
  ctx.lineTo(zColX + 2, eeZY);
  ctx.stroke();

  // Camera marker in Z
  ctx.fillStyle = "#0284c7";
  ctx.beginPath();
  ctx.arc(zColX + 2, camZY, 5, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillText(`Cam Z=${curCamZ.toFixed(1)}`, zColX + 14, camZY + 3);

  // EE Top marker in Z
  ctx.fillStyle = "#16a34a";
  ctx.fillRect(zColX - 6, eeZY, 16, 10);
  ctx.fillText(`EE Top=${curEeZ.toFixed(1)}`, zColX + 14, eeZY + 8);
}

// ============================================================================
//  DIAGNOSTIC RUN REPORTS CLIENT
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
  const base = getApiBaseUrl();
  try {
    const res = await fetch(`${base}/api/reports`);
    if (!res.ok) return;
    const reports = await res.json();
    cachedReportsList = reports;
    renderRecentReportsList(reports);
  } catch (err) {
    console.debug("fetchRecentReports error:", err);
  }
}

function renderRecentReportsList(reports) {
  const listEl = document.getElementById("recentReportsList");
  const badgeEl = document.getElementById("reportStatusBadge");
  const summaryEl = document.getElementById("reportSummaryText");

  if (!reports || reports.length === 0) {
    if (badgeEl) {
      badgeEl.textContent = "NO RUNS CACHED";
      badgeEl.className = "report-status-badge badge badge-dim";
    }
    if (summaryEl) summaryEl.textContent = "Run reports cache automatically upon each scan cycle.";
    if (listEl) listEl.innerHTML = "<span style='color:var(--text-muted);font-size:0.75rem'>No reports found on Pi.</span>";
    return;
  }

  const latest = reports[0];
  if (badgeEl) {
    badgeEl.textContent = latest.status || "COMPLETED";
    badgeEl.className = `report-status-badge badge ${latest.status === "COMPLETED" ? "badge-connected" : latest.status === "RUNNING" ? "badge-accent" : "badge-disconnected"}`;
  }
  if (summaryEl) {
    const skewStr = (latest.cam_skew_deg !== undefined) ? ` • Cam Skew: ${Number(latest.cam_skew_deg).toFixed(1)}°` : "";
    summaryEl.textContent = `Latest: ${latest.run_id} (${latest.start_time.split("T")[1]?.slice(0, 8) || ""}) • ${latest.blocks_cataloged || 0} blocks cataloged • ${latest.duration_s || 0}s${skewStr}`;
  }

  if (listEl) {
    listEl.innerHTML = reports
      .slice(0, 15)
      .map(
        (r) => `
      <button class="recent-report-pill ${r.run_id === activeReportId ? "active" : ""}" onclick="viewReport('${r.run_id}')" title="${r.status} (${r.duration_s}s) - ${r.start_time}">
        <span>${r.status === "COMPLETED" ? "✓" : "•"}</span>
        <strong>${r.run_id.replace("run_", "")}</strong>
        <span style="opacity:0.65">(${r.blocks_picked || r.blocks_cataloged || 0} blks)</span>
      </button>`
      )
      .join("");
  }
}

async function viewLatestReport() {
  const base = getApiBaseUrl();
  try {
    const res = await fetch(`${base}/api/reports/latest`);
    if (!res.ok) {
      alert("No report available yet. Run a scan cycle first.");
      return;
    }
    const data = await res.json();
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
  const base = getApiBaseUrl();
  try {
    const res = await fetch(`${base}/api/reports/${encodeURIComponent(runId)}`);
    if (!res.ok) {
      alert(`Report ${runId} not found.`);
      return;
    }
    const data = await res.json();
    showReportModal(data);
  } catch (err) {
    alert(`Error loading report: ${err.message}`);
  }
}

function downloadLatestReport() {
  const base = getApiBaseUrl();
  window.open(`${base}/api/reports/latest/download`, "_blank");
}

function closeReportModal() {
  const modal = document.getElementById("reportModal");
  if (modal) modal.style.display = "none";
}

function showReportModal(report) {
  const modal = document.getElementById("reportModal");
  const title = document.getElementById("modalReportTitle");
  const body = document.getElementById("modalReportBody");
  const dlBtn = document.getElementById("modalDownloadBtn");

  if (!modal || !body) return;

  activeReportId = report.run_id;
  if (title) title.textContent = `Diagnostic Report: ${report.run_id} [${report.status}]`;

  const base = getApiBaseUrl();
  if (dlBtn) dlBtn.href = `${base}/api/reports/${encodeURIComponent(report.run_id)}/download`;

  const cal = report.calibration || {};
  const waypoints = report.scan_waypoints || [];
  const blocks = report.cataloged_blocks || [];
  const picks = report.pick_actions || [];
  const cmds = report.motor_commands || [];

  body.innerHTML = `
    <!-- Summary Header -->
    <div class="report-section">
      <h4>Run Overview</h4>
      <div class="report-grid-kv">
        <div><span class="kv-label">Run ID</span><span class="kv-val">${report.run_id}</span></div>
        <div><span class="kv-label">Status</span><span class="kv-val" style="color:${report.status === "COMPLETED" ? "#16a34a" : "#ef4444"}">${report.status}</span></div>
        <div><span class="kv-label">Start Time</span><span class="kv-val">${report.start_time || "N/A"}</span></div>
        <div><span class="kv-label">Duration</span><span class="kv-val">${report.duration_s || 0} seconds</span></div>
        <div><span class="kv-label">Mode</span><span class="kv-val">${report.mock_hardware ? "MOCK SIMULATED" : "PHYSICAL ESP32"}</span></div>
        <div><span class="kv-label">Serial Commands</span><span class="kv-val">${cmds.length} executed</span></div>
      </div>
    </div>

    <!-- Fiducial & Skew Calibration -->
    <div class="report-section">
      <h4>1. Homing, Fiducial Calibration & Camera Housing Skew</h4>
      <div class="report-grid-kv">
        <div><span class="kv-label">ArUco Marker ID</span><span class="kv-val">#${cal.marker_id !== null ? cal.marker_id : "N/A"}</span></div>
        <div><span class="kv-label">Detected Pixel Center</span><span class="kv-val">(${cal.u_px ?? "—"}, ${cal.v_px ?? "—"}) px</span></div>
        <div><span class="kv-label">Marker Orientation</span><span class="kv-val">${cal.marker_angle_deg !== null ? cal.marker_angle_deg + "°" : "N/A"}</span></div>
        <div><span class="kv-label">Camera Skew (Physical Tilt)</span><span class="kv-val" style="color:${Math.abs(cal.cam_skew_deg || 0) > 0.05 ? "#0284c7" : "#0f172a"}">${cal.cam_skew_deg !== undefined ? cal.cam_skew_deg + "° (Compensated)" : "0.0°"}</span></div>
        <div><span class="kv-label">Auto-Scale Kx / Ky</span><span class="kv-val">${cal.kx_mm_per_px || 0.25} / ${cal.ky_mm_per_px || 0.25} mm/px</span></div>
        <div><span class="kv-label">Calibrated Cam Origin</span><span class="kv-val">(${cal.cam_origin_x_mm ?? 32.5}, ${cal.cam_origin_y_mm ?? 48.1}) mm</span></div>
        <div><span class="kv-label">Camera-to-Gripper Offset</span><span class="kv-val">(${cal.offset_x_mm ?? 0.0}, ${cal.offset_y_mm ?? -54.1}) mm</span></div>
        <div><span class="kv-label">Course Floor Elevation</span><span class="kv-val">+6.35 mm (1/4" Elevated)</span></div>
      </div>
    </div>

    <!-- Cataloged Blocks -->
    <div class="report-section">
      <h4>2. Cataloged Blocks (${blocks.length})</h4>
      ${blocks.length === 0 ? "<p style='color:var(--text-muted);font-size:0.8rem'>No blocks cataloged during scan.</p>" : `
      <table class="modal-table">
        <thead>
          <tr>
            <th>ID</th>
            <th>Color</th>
            <th>Plate (X, Y) mm</th>
            <th>Angle</th>
            <th>Confidence Views</th>
          </tr>
        </thead>
        <tbody>
          ${blocks.map(b => `
            <tr>
              <td><strong>#${b.id}</strong></td>
              <td><span style="font-weight:700;color:${COLOR_HEX[b.color] || "#0f172a"}">${b.color.toUpperCase()}</span></td>
              <td>(${Number(b.x_mm).toFixed(1)}, ${Number(b.y_mm).toFixed(1)})</td>
              <td>${Number(b.angle_deg).toFixed(1)}°</td>
              <td>${b.confidence_views || 1} views</td>
            </tr>
          `).join("")}
        </tbody>
      </table>
      `}
    </div>

    <!-- Pick, Fine Alignment & Weighing Actions -->
    <div class="report-section">
      <h4>3. Pick Actions, Fine Alignment &amp; Weighing Verification (${picks.length})</h4>
      ${picks.length === 0 ? "<p style='color:var(--text-muted);font-size:0.8rem'>No pick actions executed.</p>" : `
      <table class="modal-table">
        <thead>
          <tr>
            <th>Block</th>
            <th>Fine Align</th>
            <th>Contour Area</th>
            <th>Fine Residual Err</th>
            <th>Grip Motor (X, Y, θ)</th>
            <th>Tare / Mass (g)</th>
            <th>Class</th>
            <th>Drop Zone &amp; Status</th>
          </tr>
        </thead>
        <tbody>
          ${picks.map(p => `
            <tr>
              <td><strong>#${p.block_id}</strong> ${p.color.toUpperCase()}</td>
              <td>${p.fine_align_found ? "<span style='color:#16a34a;font-weight:700'>FOUND</span>" : "<span style='color:#d97706'>COARSE FALLBACK</span>"}</td>
              <td>${p.fine_contour_area_px ? p.fine_contour_area_px.toFixed(0) + " px" : "—"}</td>
              <td>${p.fine_residual_err_mm !== null && p.fine_residual_err_mm !== undefined ? p.fine_residual_err_mm.toFixed(2) + " mm" : "—"}</td>
              <td>(${p.gripper_motor_x.toFixed(1)}, ${p.gripper_motor_y.toFixed(1)}, ${p.servo_angle_deg.toFixed(1)}°)</td>
              <td>${p.tare_success ? "✓" : "—"} / <strong>${p.measured_mass_g !== null && p.measured_mass_g !== undefined ? p.measured_mass_g.toFixed(2) + "g" : "—"}</strong></td>
              <td><span class="badge badge-${(p.mass_class || "").toLowerCase()}">${p.mass_class || "—"}</span></td>
              <td><strong>${p.status}</strong> (${p.stack_zone})</td>
            </tr>
          `).join("")}
        </tbody>
      </table>
      `}
    </div>

    <!-- Motor Commands Sample -->
    <div class="report-section">
      <h4>4. Motor Command Sequence (${cmds.length} commands)</h4>
      <div style="max-height: 140px; overflow-y: auto; background: var(--bg-subtle); padding: 8px; border-radius: var(--radius-sm); font-family: monospace; font-size: 0.73rem;">
        ${cmds.map(c => `<div>[${c.timestamp}] <strong>TX:</strong> ${c.command} &rarr; <em>${(c.replies || []).join(", ") || "OK"}</em> (${c.duration_ms}ms)</div>`).join("")}
      </div>
    </div>
  `;

  modal.style.display = "flex";
}

// Initial draw and connection
updateReadoutsFromMotor();
renderCourseCanvas();
connectWebSocket();
fetchRecentReports();
