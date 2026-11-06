// Tier 1 Supervisor & CAD Skeleton Visualizer for HUE-DINI Gantry Robot

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
const STROKE_X = 236.0;
const STROKE_Y = 128.0;
const STROKE_Z_MAX = 84.0;
const STROKE_Z_BOTTOM = 76.71; // Practical descent from Z=0 (top) to course floor

// 3. Homed Datum Point in Fiducial Plate Coordinates (0, 0, 0)
const DATUM_X = 20.0;
const DATUM_Y = -43.5;
const DATUM_Z = 127.5;

// 4. Homed End-Effector (top of square EE) in Fiducial Plate Coordinates
const EE_HOMED_X = 28.0;
const EE_HOMED_Y = 6.0;
const EE_HOMED_Z = 97.6;

// 5. Homed Camera in Fiducial Plate Coordinates
const CAM_HOMED_X = 27.8;
const CAM_HOMED_Y = 60.1;
const CAM_HOMED_Z = 135.3;

// 6. Drop-off Platform wrt Fiducial Plate Origin (0, 0, 0)
const DROPOFF_X = 224.5;
const DROPOFF_Y = 44.5;
const DROPOFF_Z_UP = 12.6;

const CORNER_FIDUCIALS = [
  { id: 0, x: 7.5, y: 7.5 },
  { id: 1, x: 276.5, y: 7.5 },
  { id: 2, x: 276.5, y: 159.9 },
  { id: 3, x: 7.5, y: 159.9 },
];

const COLOR_HEX = {
  red: "#ef4444",
  green: "#22c55e",
  blue: "#3b82f6",
  yellow: "#eab308",
  reject: "#a855f7",
};

const STACK_ZONES = [
  { label: "DROP-OFF (224.5, 44.5, Z+12.6)", color: "green", x: DROPOFF_X, y: DROPOFF_Y },
  { label: "REJECT", color: "reject", x: 55, y: 44.5 },
];

// Current motor travel state (0..211, 0..126, 0..76.71)
let motorX = 0.0;
let motorY = 0.0;
let motorZDown = 0.0;
let manualJogActive = true;

let socket = null;
let latestState = {
  state: "IDLE",
  gantry_pose: { x_mm: CAM_HOMED_X, y_mm: CAM_HOMED_Y, z_mm: 0, angle_deg: 0, grip_closed: true },
  calibration: { calibrated: false, kx_mm_per_px: 0.25, ky_mm_per_px: 0.25 },
  blocks: [],
  logs: [],
  vision_frame_b64: null,
};

// Setup interactive Jog sliders
const jogXSlider = document.getElementById("jogX");
const jogYSlider = document.getElementById("jogY");
const jogZSlider = document.getElementById("jogZ");

function onJogInput() {
  manualJogActive = true;
  motorX = parseFloat(jogXSlider.value);
  motorY = parseFloat(jogYSlider.value);
  motorZDown = parseFloat(jogZSlider.value);
  document.getElementById("jogXVal").textContent = motorX.toFixed(1);
  document.getElementById("jogYVal").textContent = motorY.toFixed(1);
  document.getElementById("jogZVal").textContent = motorZDown.toFixed(2);
  updateReadoutsFromMotor();
  renderCourseCanvas();
}

function resetJogHome() {
  jogXSlider.value = 0;
  jogYSlider.value = 0;
  jogZSlider.value = 0;
  onJogInput();
}

jogXSlider.addEventListener("input", onJogInput);
jogYSlider.addEventListener("input", onJogInput);
jogZSlider.addEventListener("input", onJogInput);

function updateReadoutsFromMotor() {
  const eeX = EE_HOMED_X + motorX;
  const eeY = EE_HOMED_Y + motorY;
  const eeZ = EE_HOMED_Z - motorZDown;
  const camX = CAM_HOMED_X + motorX;
  const camY = CAM_HOMED_Y + motorY;
  const camZ = CAM_HOMED_Z - motorZDown;

  document.getElementById("eePlateReadout").textContent =
    `(${eeX.toFixed(1)}, ${eeY.toFixed(1)}, ${eeZ.toFixed(1)})`;
  document.getElementById("camPlateReadout").textContent =
    `(${camX.toFixed(1)}, ${camY.toFixed(1)}, ${camZ.toFixed(1)})`;
}

function connectWebSocket() {
  if (socket) socket.close();
  socket = new WebSocket(wsUrlInput.value.trim());

  socket.onopen = () => {
    connBadge.textContent = "CONNECTED";
    connBadge.className = "badge badge-connected";
  };

  socket.onclose = () => {
    connBadge.textContent = "DISCONNECTED";
    connBadge.className = "badge badge-disconnected";
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
  if (command === "START_SCAN") manualJogActive = false;
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify({ command }));
  }
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

function updateUI(data) {
  document.getElementById("stateReadout").textContent = data.state || "IDLE";
  const pose = data.gantry_pose || {};

  // When state machine is active, sync motor sliders to incoming pose
  if (data.state && data.state !== "IDLE") {
    manualJogActive = false;
    motorX = Math.max(0, Math.min(STROKE_X, (pose.x_mm ?? CAM_HOMED_X) - CAM_HOMED_X));
    motorY = Math.max(0, Math.min(STROKE_Y, (pose.y_mm ?? CAM_HOMED_Y) - CAM_HOMED_Y));
    motorZDown = Math.max(0, Math.min(STROKE_Z_MAX, pose.z_mm ?? 0));
    jogXSlider.value = motorX;
    jogYSlider.value = motorY;
    jogZSlider.value = motorZDown;
    document.getElementById("jogXVal").textContent = motorX.toFixed(1);
    document.getElementById("jogYVal").textContent = motorY.toFixed(1);
    document.getElementById("jogZVal").textContent = motorZDown.toFixed(2);
  }

  updateReadoutsFromMotor();
  document.getElementById("poseA").textContent = (pose.angle_deg ?? 0).toFixed(1);
  document.getElementById("poseGrip").textContent = pose.grip_closed ? "CLOSED (NC)" : "OPEN";

  const cal = data.calibration || {};
  document.getElementById("calStatus").textContent = cal.calibrated ? "CALIBRATED" : "Default";
  document.getElementById("calKx").textContent = (cal.kx_mm_per_px ?? 0.25).toFixed(4);
  document.getElementById("calKy").textContent = (cal.ky_mm_per_px ?? 0.25).toFixed(4);

  if (data.vision_frame_b64) {
    document.getElementById("visionPreviewImg").src = `data:image/jpeg;base64,${data.vision_frame_b64}`;
  }

  const blocks = data.blocks || [];
  document.getElementById("blockCountBadge").textContent = `${blocks.length} Blocks`;
  const tbody = document.getElementById("blocksTableBody");
  if (blocks.length === 0) {
    tbody.innerHTML = `<tr><td colspan="6" class="empty-cell">No blocks cataloged yet. Click START SCAN.</td></tr>`;
  } else {
    tbody.innerHTML = blocks
      .map(
        (b) => `
      <tr>
        <td>#${b.id}</td>
        <td><span style="color:${COLOR_HEX[b.color] || "#fff"};font-weight:700">${b.color.toUpperCase()}</span></td>
        <td>(${b.x_mm.toFixed(1)}, ${b.y_mm.toFixed(1)}) mm</td>
        <td>${b.angle_deg.toFixed(1)}&deg;</td>
        <td>${b.mass_g > 0 ? b.mass_g.toFixed(1) + "g (" + (b.mass_class || "") + ")" : "&mdash;"}</td>
        <td>${b.status}</td>
      </tr>`
      )
      .join("");
  }

  if (Array.isArray(data.logs) && data.logs.length > 0) {
    const logEl = document.getElementById("logConsole");
    logEl.textContent = data.logs.join("\n");
    logEl.scrollTop = logEl.scrollHeight;
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
  const xyPlotW = 640; // Reserve right 180px for Z-Axis Elevation View
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

  // ==========================================================================
  // PART A: TOP-DOWN XY CAD SKELETON & ENVELOPE
  // ==========================================================================
  // 1. Background 20mm grid across full XY view
  ctx.strokeStyle = "#162032";
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

  // 2. Outer Fiducial Border (284 x 167.4 mm, Origin 0,0 at bottom-left)
  const outerTL = mmToCanvas(0, PLATE_H_Y);
  const outerBR = mmToCanvas(PLATE_W_X, 0);
  ctx.fillStyle = "rgba(30, 41, 59, 0.45)";
  ctx.fillRect(outerTL.cx, outerTL.cy, outerBR.cx - outerTL.cx, outerBR.cy - outerTL.cy);
  ctx.strokeStyle = "#64748b";
  ctx.lineWidth = 2;
  ctx.strokeRect(outerTL.cx, outerTL.cy, outerBR.cx - outerTL.cx, outerBR.cy - outerTL.cy);

  // Plate Origin (0, 0, 0) marker
  const orig = mmToCanvas(0, 0);
  ctx.fillStyle = "#f43f5e";
  ctx.beginPath();
  ctx.arc(orig.cx, orig.cy, 5, 0, Math.PI * 2);
  ctx.fill();
  ctx.font = "bold 10px monospace";
  ctx.fillText("PLATE ORIGIN (0,0,0)", orig.cx - 10, orig.cy + 15);

  // 3. Inner Playable Course (254 x 152.4 mm)
  const innerTL = mmToCanvas(COURSE_ORIGIN_X, COURSE_ORIGIN_Y + COURSE_WID_Y);
  const innerBR = mmToCanvas(COURSE_ORIGIN_X + COURSE_LEN_X, COURSE_ORIGIN_Y);
  ctx.strokeStyle = "#94a3b8";
  ctx.lineWidth = 1.5;
  ctx.strokeRect(innerTL.cx, innerTL.cy, innerBR.cx - innerTL.cx, innerBR.cy - innerTL.cy);
  ctx.fillStyle = "#94a3b8";
  ctx.font = "10px monospace";
  ctx.fillText("Inner Course 254 x 152.4 mm", innerTL.cx + 6, innerTL.cy + 13);

  // 4. Four Corner Fiducials (15 x 15 mm squares)
  for (const cf of CORNER_FIDUCIALS) {
    const fid = mmToCanvas(cf.x, cf.y);
    const fpx = FIDUCIAL_SIZE * fid.scale;
    ctx.fillStyle = "#f8fafc";
    ctx.fillRect(fid.cx - fpx / 2, fid.cy - fpx / 2, fpx, fpx);
    ctx.fillStyle = "#0f172a";
    ctx.fillRect(fid.cx - fpx * 0.3, fid.cy - fpx * 0.3, fpx * 0.6, fpx * 0.6);
  }

  // 5. End-Effector Reachable Travel Envelope (236 x 161 mm from (24.0, 6.0))
  const eeBoxTL = mmToCanvas(EE_HOMED_X, EE_HOMED_Y + STROKE_Y);
  const eeBoxBR = mmToCanvas(EE_HOMED_X + STROKE_X, EE_HOMED_Y);
  ctx.strokeStyle = "rgba(34, 197, 94, 0.75)";
  ctx.lineWidth = 1.5;
  ctx.setLineDash([6, 4]);
  ctx.strokeRect(eeBoxTL.cx, eeBoxTL.cy, eeBoxBR.cx - eeBoxTL.cx, eeBoxBR.cy - eeBoxTL.cy);
  ctx.setLineDash([]);
  ctx.fillStyle = "#22c55e";
  ctx.font = "10px monospace";
  ctx.fillText(
    `EE Reach (${STROKE_X}x${STROKE_Y}mm): X[${EE_HOMED_X.toFixed(1)}..${(EE_HOMED_X + STROKE_X).toFixed(1)}] Y[${EE_HOMED_Y.toFixed(1)}..${(EE_HOMED_Y + STROKE_Y).toFixed(1)}]`,
    eeBoxTL.cx + 6,
    eeBoxBR.cy - 6
  );

  // 6. Camera Optical Center Reachable Travel Envelope (236 x 161 mm from (23.8, 60.1))
  const camBoxTL = mmToCanvas(CAM_HOMED_X, CAM_HOMED_Y + STROKE_Y);
  const camBoxBR = mmToCanvas(CAM_HOMED_X + STROKE_X, CAM_HOMED_Y);
  ctx.strokeStyle = "rgba(56, 189, 248, 0.65)";
  ctx.lineWidth = 1.5;
  ctx.setLineDash([3, 3]);
  ctx.strokeRect(camBoxTL.cx, camBoxTL.cy, camBoxBR.cx - camBoxTL.cx, camBoxBR.cy - camBoxTL.cy);
  ctx.setLineDash([]);
  ctx.fillStyle = "#38bdf8";
  ctx.fillText(
    `Cam Reach (${STROKE_X}x${STROKE_Y}mm): X[${CAM_HOMED_X.toFixed(1)}..${(CAM_HOMED_X + STROKE_X).toFixed(1)}] Y[${CAM_HOMED_Y.toFixed(1)}..${(CAM_HOMED_Y + STROKE_Y).toFixed(1)}]`,
    camBoxTL.cx + 6,
    camBoxTL.cy - 5
  );

  // 7. Homed Datum Origin (27.0, -43.5, 127.5) & Current Carriage Datum
  const curDatumX = DATUM_X + motorX;
  const curDatumY = DATUM_Y + motorY;
  const homeDatumPt = mmToCanvas(DATUM_X, DATUM_Y);
  const curDatumPt = mmToCanvas(curDatumX, curDatumY);

  ctx.fillStyle = "#f59e0b";
  ctx.beginPath();
  ctx.arc(homeDatumPt.cx, homeDatumPt.cy, 5, 0, Math.PI * 2);
  ctx.fill();
  ctx.font = "bold 10px monospace";
  ctx.fillText(`HOMED DATUM (${DATUM_X.toFixed(1)}, ${DATUM_Y.toFixed(1)}, ${DATUM_Z.toFixed(1)})`, homeDatumPt.cx + 8, homeDatumPt.cy + 4);

  // 8. Draw Drop-Off / Stacking Zones
  for (const zone of STACK_ZONES) {
    const zp = mmToCanvas(zone.x, zone.y);
    const bpx = BLOCK_SIZE * zp.scale;
    ctx.strokeStyle = COLOR_HEX[zone.color] || "#94a3b8";
    ctx.lineWidth = 1.2;
    ctx.setLineDash([3, 3]);
    ctx.strokeRect(zp.cx - bpx / 2, zp.cy - bpx / 2, bpx, bpx);
    ctx.setLineDash([]);
    ctx.fillStyle = COLOR_HEX[zone.color] || "#94a3b8";
    ctx.font = "9px sans-serif";
    ctx.fillText(zone.label, zp.cx - 28, zp.cy - bpx / 2 - 3);
  }

  // 9. Draw Cataloged 20x20 mm Blocks
  const blocks = latestState.blocks || [];
  for (const blk of blocks) {
    const bp = mmToCanvas(blk.x_mm, blk.y_mm);
    const sidePx = BLOCK_SIZE * bp.scale;

    ctx.save();
    ctx.translate(bp.cx, bp.cy);
    ctx.rotate((-blk.angle_deg * Math.PI) / 180.0);
    ctx.fillStyle = COLOR_HEX[blk.color] || "#e2e8f0";
    ctx.fillRect(-sidePx / 2, -sidePx / 2, sidePx, sidePx);
    ctx.strokeStyle = "#0f172a";
    ctx.lineWidth = 1.5;
    ctx.strokeRect(-sidePx / 2, -sidePx / 2, sidePx, sidePx);
    ctx.restore();

    ctx.fillStyle = "#f8fafc";
    ctx.font = "bold 10px monospace";
    ctx.fillText(`#${blk.id}`, bp.cx - 6, bp.cy + 3);
  }

  // 10. Draw Current CAD Skeleton Link: Carriage Datum -> Square End-Effector -> Camera
  const eePt = mmToCanvas(EE_HOMED_X + motorX, EE_HOMED_Y + motorY);
  const camPt = mmToCanvas(CAM_HOMED_X + motorX, CAM_HOMED_Y + motorY);

  // Rigid arm connecting Carriage Datum -> EE -> Camera
  ctx.strokeStyle = "#eab308";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(curDatumPt.cx, curDatumPt.cy);
  ctx.lineTo(eePt.cx, eePt.cy);
  ctx.lineTo(camPt.cx, camPt.cy);
  ctx.stroke();

  // Draw Square End-Effector (20x20 mm footprint) at eePt
  const eeSide = 22 * eePt.scale;
  ctx.fillStyle = "rgba(34, 197, 94, 0.35)";
  ctx.strokeStyle = "#22c55e";
  ctx.lineWidth = 2;
  ctx.fillRect(eePt.cx - eeSide / 2, eePt.cy - eeSide / 2, eeSide, eeSide);
  ctx.strokeRect(eePt.cx - eeSide / 2, eePt.cy - eeSide / 2, eeSide, eeSide);
  ctx.fillStyle = "#bbf7d0";
  ctx.font = "bold 10px monospace";
  ctx.fillText("EE", eePt.cx - 7, eePt.cy + 4);

  // Draw Camera Optical Center & FOV at camPt
  ctx.strokeStyle = "#38bdf8";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(camPt.cx, camPt.cy, 9, 0, Math.PI * 2);
  ctx.stroke();
  ctx.fillStyle = "#38bdf8";
  ctx.fillText("CAM", camPt.cx + 12, camPt.cy + 4);

  // ==========================================================================
  // PART B: SIDE-VIEW Z-AXIS ELEVATION DIAGRAM (Right Column of Canvas)
  // ==========================================================================
  const zColX = 710;
  const zTopY = 36;
  const zBotY = courseCanvas.height - 36;
  const maxZMm = 145.0;

  function zMmToY(z_mm) {
    return zBotY - (z_mm / maxZMm) * (zBotY - zTopY);
  }

  // Z Panel background
  ctx.fillStyle = "#0b1120";
  ctx.strokeStyle = "#334155";
  ctx.lineWidth = 1;
  ctx.fillRect(zColX - 25, zTopY - 16, 160, zBotY - zTopY + 32);
  ctx.strokeRect(zColX - 25, zTopY - 16, 160, zBotY - zTopY + 32);

  ctx.fillStyle = "#e2e8f0";
  ctx.font = "bold 11px sans-serif";
  ctx.fillText("Z-Axis Elevation (mm)", zColX - 15, zTopY - 2);

  // Course Floor (Z = 0 mm)
  const floorY = zMmToY(0);
  ctx.strokeStyle = "#64748b";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(zColX - 15, floorY);
  ctx.lineTo(zColX + 120, floorY);
  ctx.stroke();
  ctx.fillStyle = "#94a3b8";
  ctx.font = "10px monospace";
  ctx.fillText("Z=0 Plate Floor", zColX + 18, floorY - 5);

  // Drop-off Platform at Z = +12.6 mm
  const dropOffZY = zMmToY(DROPOFF_Z_UP);
  ctx.fillStyle = "rgba(34, 197, 94, 0.35)";
  ctx.fillRect(zColX + 18, dropOffZY, 95, floorY - dropOffZY);
  ctx.strokeStyle = "#22c55e";
  ctx.lineWidth = 1.5;
  ctx.strokeRect(zColX + 18, dropOffZY, 95, floorY - dropOffZY);
  ctx.fillStyle = "#86efac";
  ctx.fillText("Drop Z=+12.6", zColX + 24, dropOffZY - 4);

  // 20mm Block on Floor
  const blkTopY = zMmToY(20.0);
  ctx.fillStyle = "rgba(239, 68, 68, 0.5)";
  ctx.fillRect(zColX - 8, blkTopY, 20, floorY - blkTopY);
  ctx.strokeStyle = "#ef4444";
  ctx.strokeRect(zColX - 8, blkTopY, 20, floorY - blkTopY);

  // Homed Datum Z (127.5 mm)
  const datumZY = zMmToY(DATUM_Z);
  ctx.strokeStyle = "#f59e0b";
  ctx.setLineDash([3, 3]);
  ctx.beginPath();
  ctx.moveTo(zColX - 15, datumZY);
  ctx.lineTo(zColX + 120, datumZY);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = "#f59e0b";
  ctx.fillText("Datum Z=127.5", zColX + 25, datumZY - 4);

  // Current Camera Z & EE Top Z
  const curCamZ = CAM_HOMED_Z - motorZDown;
  const curEeZ = EE_HOMED_Z - motorZDown;
  const camZY = zMmToY(curCamZ);
  const eeZY = zMmToY(curEeZ);

  // Vertical Z carriage bar
  ctx.strokeStyle = "#eab308";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(zColX + 2, camZY);
  ctx.lineTo(zColX + 2, eeZY);
  ctx.stroke();

  // Camera marker in Z
  ctx.fillStyle = "#38bdf8";
  ctx.beginPath();
  ctx.arc(zColX + 2, camZY, 5, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillText(`Cam Z=${curCamZ.toFixed(1)}`, zColX + 14, camZY + 3);

  // EE Top marker in Z
  ctx.fillStyle = "#22c55e";
  ctx.fillRect(zColX - 6, eeZY, 16, 12);
  ctx.fillText(`EE Top=${curEeZ.toFixed(1)}`, zColX + 14, eeZY + 9);
}

updateReadoutsFromMotor();
renderCourseCanvas();
connectWebSocket();
