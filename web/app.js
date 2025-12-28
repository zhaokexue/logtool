const el = (id)=>document.getElementById(id);

const slider = el('slider');
const tsecEl = el('tsec');
const poseTsEl = el('pose_ts');
const scanTsEl = el('scan_ts');
const mapTsEl = el('map_ts');
const poseTxtEl = el('pose_txt');
const imuPitchEl = el('imu_pitch');
const imuRollEl = el('imu_roll');
const imuYawEl = el('imu_yaw');
const odomEl = el('odom_txt');
const slipEl = el('slip_txt');
const txtTimeEl = el('txtTime');
const txtStateEl = el('txtState');
const speedLabelEl = el('speedLabel');

// extra status text fields in top toolbar (optional)
const txtExceptionEl = el('txtException');
const txtMotionStateEl = el('txtMotionState');

// right info panel (optional; keep null-safe)
const ctrlVelEl = el('ctrl_vel');
const odoPoseEl2 = el('odo_pose');
const odoFusePoseEl2 = el('odo_fuse_pose');
const robotPoseEl = el('robot_pose');
const imuPryYawVelEl = el('imu_pry_yawvel');
const imuAccEl = el('imu_acc');
const bumperWheelEl = el('bumper_wheel');
const irSonarEl = el('ir_sonar');
const cliffIrEl = el('cliff_ir');
const slipMergeEl = el('slip_merge');
const dockMergeEl = el('dock_merge');
const batteryVoltageEl = el('battery_voltage');

const btnPlay = el('btnPlay');
const btnRecord = el('btnRecord');
const btnResetTraj = el('btnResetTraj');
const selSpeed = el('selSpeed');

const canvas = el('canvas');
const ctx = canvas.getContext('2d');

// V3 viewer controls
const ckMap = el('ckMap');
const ckCloud = el('ckCloud');
const ckTraj = el('ckTraj');
const ckRobot = el('ckRobot');
const ckGrid = el('ckGrid');
const ckAxes = el('ckAxes');
const ckScale = el('ckScale');
const btnFit = el('btnFit');
const btnFitMap = el('btnFitMap');
const btnFitTraj = el('btnFitTraj');
const btnResetView = el('btnResetView');
const ckGlobalPath = el('ckGlobalPath');
const ckLocalPath  = el('ckLocalPath');
const ckDebugPoints = el('ckDebugPoints');

// Mouse world coordinate (debug)
const mouseWorldEl = el('mouseWorld');

// ---------------------- Measure UI (insert after ResetTraj, no HTML change) ----------------------
const btnMeasure = (function(){
  let b = el('btnMeasure');
  if (!b && btnResetTraj && btnResetTraj.parentElement){
    b = document.createElement('button');
    b.id = 'btnMeasure';
    b.type = 'button';
    b.textContent = '测量';

    // --- keep UI consistent with existing toolbar buttons ---
    if (btnResetTraj.className) b.className = btnResetTraj.className;
    // copy common attributes if your CSS relies on them
    for (const k of ['aria-label', 'title', 'data-variant', 'data-size']){
      const v = btnResetTraj.getAttribute(k);
      if (v !== null) b.setAttribute(k, v);
    }

    const parent = btnResetTraj.parentElement;
    if (btnResetTraj.nextSibling) parent.insertBefore(b, btnResetTraj.nextSibling);
    else parent.appendChild(b);
  }
  return b;
})();

// UI: overlay accordion (layers / helpers / view). No impact to viewer logic.
function initOverlayAccordion(){
  const headers = document.querySelectorAll('.viewer-overlay .acc-header');
  for (const h of headers){
    h.addEventListener('click', ()=>{
      const item = h.closest('.acc-item');
      if (!item) return;
      const nowCollapsed = item.classList.toggle('is-collapsed');
      h.setAttribute('aria-expanded', String(!nowCollapsed));
    });
  }
}

let meta = null;
let playing = false;
let speed = 1.0;
let lastAnimMs = performance.now();
let dragging = false;
let inFlight = null;
let requestSeq = 0;

// Camera (world -> screen):
//  sx = offsetX + wx*scale
//  sy = offsetY - wy*scale  (world Y up)
//
// Upgrade: add rotation (rot, rad) and keep "world Y up" convention.
//  world -> camera: [rx; ry] = R(rot) * [wx; wy]
//  sx = offsetX + rx*scale
//  sy = offsetY - ry*scale
let cam = {
  scale: 80.0,   // px per meter
  offsetX: 0.0,  // px
  offsetY: 0.0,  // px
  rot: 0.0,      // rad, +CCW (view rotation)
};

// Pan (LMB drag) / Rotate (Shift + LMB drag)
let panState = {
  active:false,
  startX:0, startY:0,
  startOX:0, startOY:0,
  // rotate mode
  mode:'pan',         // 'pan' | 'rot'
  startRot:0,
  // rotate around cursor anchor
  anchorSX:0,
  anchorSY:0,
  anchorW:null,       // {x,y} in world(m)
};

// derived state
let lastFrame = null;
let trajPts = []; // appended while playing
let currentTsNs = 0n;
// extra drawable data (can come from frame or external JS APIs)
let extraGlobalPath = null;  // [{x,y}, ...]
let extraLocalPath  = null;  // [{x,y}, ...]
let extraPointSets  = {};    // key -> {color, radius, points:[{x,y}, ...]}

// ---------------------- Measure tool state ----------------------
const measure = {
  enabled: false,
  a: null,       // {x,y} in world(m)
  b: null,       // {x,y} in world(m)
  dist_m: 0,
};

function setMeasureLabel(txt){}

function clearMeasure(keepEnabled=true){
  measure.a = null;
  measure.b = null;
  measure.dist_m = 0;
}

function updateMeasureDistanceLabel(){}

// Trajectory-from-backend (prefix) for correct seek/drag rendering.
let trajReq = { ac:null, lastMs:0 };
async function refreshTrajPrefix(tsNs, force=false){
  if (!meta) return;
  const now = performance.now();
  const minMs = force ? 0 : (dragging ? 120 : 300);
  if (!force && (now - trajReq.lastMs) < minMs) return;
  trajReq.lastMs = now;

  if (trajReq.ac) try { trajReq.ac.abort(); } catch(_) {}
  const ac = new AbortController();
  trajReq.ac = ac;

  // 20Hz trajectory for smooth recording and light rendering.
  const step_ms = 50;
  const max_points = 12000;
  try {
    const r = await fetch(`/api/traj?ts_ns=${tsNs.toString()}&step_ms=${step_ms}&max_points=${max_points}`, {signal: ac.signal});
    const j = await r.json();
    if (j.error) return;
    const bytes = b64ToBytes(j.xy_f32_b64);
    const f32 = new Float32Array(bytes.buffer, bytes.byteOffset, Math.floor(bytes.byteLength/4));
    const pts = [];
    for (let i=0; i+1<f32.length; i+=2){
      pts.push({x: f32[i], y: f32[i+1]});
    }
    trajPts = pts;
  } catch(e){
    // ignore abort or network errors during drag
  }
}

// map cache
let mapCache = null; // {ts, w,h,res, origin:[x,y], data:Uint8Array/int8Array}

function fmt2(v){
  if (v === null || v === undefined) return '-';
  if (typeof v === 'number') return v.toFixed(2);
  return String(v);
}

// Tolerant getter for runtime status fields (won't break old backend/logs)
function getStatusObj(frame){
  if (!frame) return null;
  return frame.status || frame.runtime || frame.rt || frame.telemetry || null;
}

function bool01(v){
  if (v === null || v === undefined) return '-';
  if (typeof v === 'boolean') return v ? 1 : 0;
  if (typeof v === 'number') return v ? 1 : 0;
  if (typeof v === 'string'){
    const t = v.trim().toLowerCase();
    if (t === '' || t === '0' || t === 'false' || t === 'no') return 0;
    return 1;
  }
  return '-';
}

async function getMeta(){
  const r = await fetch('/api/meta');
  return await r.json();
}

function secFromSlider(){
  const t = Number(slider.value) / Number(slider.max);
  return t * meta.duration_sec;
}

function sliderFromSec(sec){
  const t = meta.duration_sec > 0 ? (sec / meta.duration_sec) : 0;
  return Math.max(0, Math.min(Number(slider.max), Math.floor(t * Number(slider.max))));
}

function tsFromSec(sec){
  return BigInt(meta.t0_ns) + BigInt(Math.floor(sec * 1e9));
}

async function fetchJson(url){
  const r = await fetch(url);
  return await r.json();
}

function b64ToBytes(b64){
  const bin = atob(b64);
  const out = new Uint8Array(bin.length);
  for (let i=0;i<bin.length;i++) out[i] = bin.charCodeAt(i);
  return out;
}

async function ensureMap(tsNs){
  if (!meta.has_map) return;
  if (mapCache && mapCache.ts === meta.map_latest_ts && tsNs >= BigInt(mapCache.ts)) {
    // good enough
    return;
  }
  // Always request map nearest current ts
  const j = await fetchJson(`/api/map?ts_ns=${tsNs.toString()}`);
  if (j.error) return;

  const bytes = b64ToBytes(j.data_b64);
  // data is int8 serialized as unsigned bytes, reinterpret by mapping 0..255 -> -128..127
  const data = new Int8Array(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  mapCache = {
    ts: j.map_ts,
    w: j.w,
    h: j.h,
    res: j.res,
    ox: j.ox,
    oy: j.oy,
    data,
  };
}

function setInfo(frame){
  if (!frame || frame.error) return;
  // NOTE: some panels/fields may be removed from HTML; keep all updates null-safe.
  if (poseTsEl) poseTsEl.textContent = frame.pose_ts ?? '-';
  if (scanTsEl) scanTsEl.textContent = frame.scan_ts ?? '-';
  if (mapTsEl) mapTsEl.textContent = frame.map_ts ?? (mapCache ? mapCache.ts : '-');

  if (poseTxtEl && frame.pose){
    poseTxtEl.textContent = `(${frame.pose.x.toFixed(2)}, ${frame.pose.y.toFixed(2)}, ${frame.pose.yaw.toFixed(2)})`;
  }

  // Robot pose (per new info panel)
  if (robotPoseEl && frame.pose){
    robotPoseEl.textContent = `(${frame.pose.x.toFixed(2)}, ${frame.pose.y.toFixed(2)}, ${frame.pose.yaw.toFixed(2)})`;
  }

  if (frame.imu){
    if (imuPitchEl) imuPitchEl.textContent = fmt2(frame.imu.pitch);
    if (imuRollEl) imuRollEl.textContent = fmt2(frame.imu.roll);
    if (imuYawEl) imuYawEl.textContent = fmt2(frame.imu.yaw);
  } else {
    if (imuPitchEl) imuPitchEl.textContent = '-';
    if (imuRollEl) imuRollEl.textContent = '-';
    if (imuYawEl) imuYawEl.textContent = '-';
  }

  if (frame.odom){
    const p = frame.odom;
    if (odomEl) odomEl.textContent = `(${p.x.toFixed(2)}, ${p.y.toFixed(2)}, ${p.yaw.toFixed(2)})`;
  } else {
    if (odomEl) odomEl.textContent = '-';
  }

  if (slipEl) slipEl.textContent = (frame.slip !== null && frame.slip !== undefined) ? String(frame.slip) : '-';

  if (txtStateEl) txtStateEl.textContent = frame.state ?? '-';

  // time display
  if (txtTimeEl) txtTimeEl.textContent = frame.time_text ?? '-';

  // extra status text fields (optional)
  if (txtExceptionEl) txtExceptionEl.textContent = (frame.exception !== null && frame.exception !== undefined) ? String(frame.exception) : '-';
  if (txtMotionStateEl) txtMotionStateEl.textContent = (frame.motion_state !== null && frame.motion_state !== undefined) ? String(frame.motion_state) : '-';

  // New info panel fields: order & grouping per UI spec
  const s = getStatusObj(frame);

  // ctrl vel
  if (ctrlVelEl){
    if (s){
      const v = (s.ctrl_v ?? s.ctrl_vel_v ?? s.v);
      const w = (s.ctrl_w ?? s.ctrl_vel_w ?? s.w);
      ctrlVelEl.textContent = `(${fmt2(v)}, ${fmt2(w)})`;
    } else {
      ctrlVelEl.textContent = '-';
    }
  }

  // odo poses
  if (odoPoseEl2){
    if (s){
      const x = (s.odo_x ?? s.odo_pose_x ?? s.odoX);
      const y = (s.odo_y ?? s.odo_pose_y ?? s.odoY);
      const p = (s.odo_phi ?? s.odo_pose_phi ?? s.odoPhi);
      odoPoseEl2.textContent = `(${fmt2(x)}, ${fmt2(y)}, ${fmt2(p)})`;
    } else {
      odoPoseEl2.textContent = '-';
    }
  }

  if (odoFusePoseEl2){
    if (s){
      const x = (s.fuse_x ?? s.odo_imu_fuse_x ?? s.fuseX);
      const y = (s.fuse_y ?? s.odo_imu_fuse_y ?? s.fuseY);
      const p = (s.fuse_phi ?? s.odo_imu_fuse_phi ?? s.fusePhi);
      odoFusePoseEl2.textContent = `(${fmt2(x)}, ${fmt2(y)}, ${fmt2(p)})`;
    } else {
      odoFusePoseEl2.textContent = '-';
    }
  }

  // IMU combined: pitch/roll/yaw/yaw_vel
  if (imuPryYawVelEl){
    const yawv = s ? (s.imu_yaw_vel ?? s.yaw_vel ?? s.gyro_z) : null;
    if (frame.imu){
      imuPryYawVelEl.textContent = `${fmt2(frame.imu.pitch)}, ${fmt2(frame.imu.roll)}, ${fmt2(frame.imu.yaw)}, ${fmt2(yawv)}`;
    } else {
      imuPryYawVelEl.textContent = '-';
    }
  }

  // IMU acc combined
  if (imuAccEl){
    if (s){
      const ax = (s.imu_acc_x ?? s.acc_x ?? s.ax);
      const ay = (s.imu_acc_y ?? s.acc_y ?? s.ay);
      const az = (s.imu_acc_z ?? s.acc_z ?? s.az);
      imuAccEl.textContent = `${fmt2(ax)}, ${fmt2(ay)}, ${fmt2(az)}`;
    } else {
      imuAccEl.textContent = '-';
    }
  }

  // Bumper / Wheel_up
  if (bumperWheelEl){
    if (s){
      const b = `B(${bool01(s.left_bumper)},${bool01(s.right_bumper)})`;
      const w = `W(${bool01(s.left_wheel_up)},${bool01(s.right_wheel_up)})`;
      bumperWheelEl.textContent = `${b}  ,  ${w}`;
    } else {
      bumperWheelEl.textContent = '-';
    }
  }

  // Right_IR / Sonar
  if (irSonarEl){
    if (s){
      const ir = `IR(${bool01(s.right_ir)})`;
      const so = `S(${fmt2(s.sonar)})`;
      irSonarEl.textContent = `${ir}  ,  ${so}`;
    } else {
      irSonarEl.textContent = '-';
    }
  }

  // Cliff_IR(LR,LF,RF,RR)
  if (cliffIrEl){
    if (s){
      cliffIrEl.textContent = `(${bool01(s.cliff_lr)}, ${bool01(s.cliff_lf)}, ${bool01(s.cliff_rf)}, ${bool01(s.cliff_rr)})`;
    } else {
      cliffIrEl.textContent = '-';
    }
  }

  // Line_slip / Rotate_slip
  if (slipMergeEl){
    if (s){
      const ls = `L(${bool01(s.line_slip_fwd)},${bool01(s.line_slip_back)})`;
      const rs = `R(${bool01(s.rotate_slip_cw)},${bool01(s.rotate_slip_ccw)})`;
      slipMergeEl.textContent = `${ls}  ,  ${rs}`;
    } else {
      slipMergeEl.textContent = '-';
    }
  }

  // Dock_IR / Dock_clip_state
  if (dockMergeEl){
    if (s){
      const d = `D(${bool01(s.dock_ir1)},${bool01(s.dock_ir2)},${bool01(s.dock_ir3)},${bool01(s.dock_ir4)})`;
      const c = `C(${bool01(s.dock_clip_state)})`;
      dockMergeEl.textContent = `${d}  ,  ${c}`;
    } else {
      dockMergeEl.textContent = '-';
    }
  }

  // Battery_voltage
  if (batteryVoltageEl){
    if (s){
      batteryVoltageEl.textContent = fmt2(s.battery_voltage ?? s.battery_v);
    } else {
      batteryVoltageEl.textContent = '-';
    }
  }
}

// ---------- rendering ----------
function clear(){
  ctx.clearRect(0,0,canvas.width,canvas.height);
}

function screenToWorld(sx, sy){
  // Upgrade: support rotation (inverse transform).
  // screen -> camera (meters)
  const dx = (sx - cam.offsetX) / cam.scale;
  const dy = (cam.offsetY - sy) / cam.scale;

  // camera -> world (inverse rotate)
  const c = Math.cos(cam.rot);
  const s = Math.sin(cam.rot);

  return {
    x:  c * dx + s * dy,
    y: -s * dx + c * dy,
  };
}

function worldToScreen(wx, wy){
  // Upgrade: support rotation (forward transform).
  const c = Math.cos(cam.rot);
  const s = Math.sin(cam.rot);

  // world -> camera (rotate)
  const rx = c * wx - s * wy;
  const ry = s * wx + c * wy;

  return {
    x: cam.offsetX + rx*cam.scale,
    y: cam.offsetY - ry*cam.scale,
  };
}

// Helper: convert browser client (CSS pixels) -> canvas pixel coords
function canvasClientToCanvasPx(e){
  const rect = canvas.getBoundingClientRect();
  const sx = (e.clientX - rect.left) * (canvas.width / rect.width);
  const sy = (e.clientY - rect.top) * (canvas.height / rect.height);
  return {sx, sy};
}

// Update mouse position (world coordinates) under cursor, for debugging.
function updateMouseWorld(e){
  if (!mouseWorldEl) return;
  const {sx, sy} = canvasClientToCanvasPx(e);
  const w = screenToWorld(sx, sy);
  mouseWorldEl.textContent = `(${w.x.toFixed(3)}, ${w.y.toFixed(3)})`;
}

// Helper: keep a world point fixed under a screen pixel (used by zoom + rotate-around-cursor)
function setCamOffsetForAnchor(wx, wy, sx, sy){
  const c = Math.cos(cam.rot);
  const s = Math.sin(cam.rot);
  const rx = c * wx - s * wy;
  const ry = s * wx + c * wy;
  cam.offsetX = sx - rx * cam.scale;
  cam.offsetY = sy + ry * cam.scale;
}

function pickNiceStepMeters(targetMeters){
  // choose from a 1-2-5 series (and decades)
  const base = [1,2,5];
  const exp = Math.floor(Math.log10(targetMeters));
  const decade = Math.pow(10, exp);
  let best = base[0]*decade;
  for (const b of base){
    const v = b*decade;
    if (Math.abs(v - targetMeters) < Math.abs(best - targetMeters)) best = v;
  }
  // If target is far smaller than decade range, check down a decade.
  const decadeDown = decade/10;
  for (const b of base){
    const v = b*decadeDown;
    if (Math.abs(v - targetMeters) < Math.abs(best - targetMeters)) best = v;
  }
  // If target is far larger, check up a decade.
  const decadeUp = decade*10;
  for (const b of base){
    const v = b*decadeUp;
    if (Math.abs(v - targetMeters) < Math.abs(best - targetMeters)) best = v;
  }
  return Math.max(1e-6, best);
}

function drawRvizGrid(){
  if (!ckGrid || !ckGrid.checked) return;

  const w = canvas.width, h = canvas.height;
  const tl = screenToWorld(0,0);
  const br = screenToWorld(w,h);
  const minx = Math.min(tl.x, br.x);
  const maxx = Math.max(tl.x, br.x);
  const miny = Math.min(tl.y, br.y);
  const maxy = Math.max(tl.y, br.y);

  // Keep major grid spacing around ~80px.
  const majorMeters = pickNiceStepMeters(80.0 / cam.scale);
  const minorMeters = majorMeters / 5.0;

  const majorPx = majorMeters * cam.scale;
  const minorPx = minorMeters * cam.scale;
  const showMinor = minorPx >= 8;

  // Grid lines aligned to world coordinates (like RViz).
  const startXMinor = Math.floor(minx / minorMeters) * minorMeters;
  const startYMinor = Math.floor(miny / minorMeters) * minorMeters;

  ctx.save();

  // ---------------- RViz-like gray theme ----------------
  // Assumed canvas bg is light gray (e.g. #e6e6e6 in CSS).
  // Minor grid: very light gray (still distinguishable from bg)
  // Major grid: slightly darker gray
  const minorStroke = 'rgba(255,255,255,0.12)';  // was 0.08 (too faint on gray bg)
  const majorStroke = 'rgba(255,255,255,0.18)';  // was 0.18
  const labelFill   = 'rgba(255,255,255,0.55)';  // softer than before (0.55)

  // minor
  // NOTE: after adding rotation, grid lines must be drawn by endpoints in world,
  // not by assuming screen-aligned X/Y.
  if (showMinor){
    ctx.strokeStyle = minorStroke;
    ctx.lineWidth = 1;
    for (let x = startXMinor; x <= maxx; x += minorMeters){
      const A = worldToScreen(x, miny);
      const B = worldToScreen(x, maxy);
      ctx.beginPath(); ctx.moveTo(A.x, A.y); ctx.lineTo(B.x, B.y); ctx.stroke();
    }
    for (let y = startYMinor; y <= maxy; y += minorMeters){
      const A = worldToScreen(minx, y);
      const B = worldToScreen(maxx, y);
      ctx.beginPath(); ctx.moveTo(A.x, A.y); ctx.lineTo(B.x, B.y); ctx.stroke();
    }
  }

  // major
  const startXMajor = Math.floor(minx / majorMeters) * majorMeters;
  const startYMajor = Math.floor(miny / majorMeters) * majorMeters;
  ctx.strokeStyle = majorStroke;
  ctx.lineWidth = 1.5;
  for (let x = startXMajor; x <= maxx; x += majorMeters){
    const A = worldToScreen(x, miny);
    const B = worldToScreen(x, maxy);
    ctx.beginPath(); ctx.moveTo(A.x, A.y); ctx.lineTo(B.x, B.y); ctx.stroke();
  }
  for (let y = startYMajor; y <= maxy; y += majorMeters){
    const A = worldToScreen(minx, y);
    const B = worldToScreen(maxx, y);
    ctx.beginPath(); ctx.moveTo(A.x, A.y); ctx.lineTo(B.x, B.y); ctx.stroke();
  }

  // coordinate labels on major grid (lightweight, edges only)
  // NOTE: under rotation, "edges only" is not as meaningful; keep it lightweight and correct numerically.
  ctx.fillStyle = labelFill;
  ctx.font = '12px system-ui, sans-serif';
  ctx.textBaseline = 'top';
  for (let x = startXMajor; x <= maxx; x += majorMeters){
    const sp = worldToScreen(x, maxy);
    if (sp.x < 0 || sp.x > w || sp.y < 0 || sp.y > h) continue;
    ctx.fillText(`${x.toFixed(1)}m`, sp.x + 2, sp.y + 2);
  }
  ctx.textBaseline = 'bottom';
  for (let y = startYMajor; y <= maxy; y += majorMeters){
    const sp = worldToScreen(minx, y);
    if (sp.x < 0 || sp.x > w || sp.y < 0 || sp.y > h) continue;
    ctx.fillText(`${y.toFixed(1)}m`, sp.x + 2, sp.y - 2);
  }

  ctx.restore();
}

function drawAxes(){
  if (!ckAxes || !ckAxes.checked) return;

  // NOTE: after adding rotation, axes should represent WORLD axes, not screen axes.
  const o = worldToScreen(0,0);
  const L = 5.0; // meters
  const xEnd = worldToScreen(L, 0);
  const yEnd = worldToScreen(0, L);

  ctx.save();
  ctx.lineWidth = 2;

  // X axis (red)
  ctx.strokeStyle = '#d62728';
  ctx.beginPath(); ctx.moveTo(o.x, o.y); ctx.lineTo(xEnd.x, xEnd.y); ctx.stroke();

  // Y axis (green)
  ctx.strokeStyle = '#2ca02c';
  ctx.beginPath(); ctx.moveTo(o.x, o.y); ctx.lineTo(yEnd.x, yEnd.y); ctx.stroke();

  ctx.fillStyle = '#d62728';
  ctx.font = 'bold 14px system-ui, sans-serif';
  ctx.fillText('X', xEnd.x + 6, xEnd.y + 6);

  ctx.fillStyle = '#2ca02c';
  ctx.fillText('Y', yEnd.x + 6, yEnd.y + 6);

  ctx.restore();
}

function drawScaleBar(){
  if (!ckScale || !ckScale.checked) return;
  const w = canvas.width, h = canvas.height;

  // Aim for ~140px bar.
  const meters = pickNiceStepMeters(140.0 / cam.scale);
  const px = meters * cam.scale;
  const margin = 16;
  const x1 = w - margin;
  const x0 = x1 - px;
  const y = h - margin;

  ctx.save();
  ctx.strokeStyle = 'rgba(255,255,255,0.85)';
  ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(x0, y); ctx.lineTo(x1, y); ctx.stroke();
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(x0, y-8); ctx.lineTo(x0, y+8); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(x1, y-8); ctx.lineTo(x1, y+8); ctx.stroke();

  ctx.fillStyle = 'rgba(255,255,255,0.85)';
  ctx.font = '12px system-ui, sans-serif';
  ctx.textAlign = 'right';
  ctx.textBaseline = 'bottom';
  ctx.fillText(`${meters >= 1 ? meters.toFixed(0) : meters.toFixed(1)} m`, x1, y-10);
  ctx.restore();
}

function computeBoundsFromMap(){
  if (!mapCache) return null;
  const minx = mapCache.ox;
  const miny = mapCache.oy;
  const maxx = mapCache.ox + mapCache.w * mapCache.res;
  const maxy = mapCache.oy + mapCache.h * mapCache.res;
  return {minx, maxx, miny, maxy};
}

function computeBoundsFromTrajOrPose(frame){
  const pts = trajPts.length ? trajPts : (frame && frame.pose ? [{x: frame.pose.x, y: frame.pose.y}] : [{x:0,y:0}]);
  let minx=pts[0].x, maxx=pts[0].x, miny=pts[0].y, maxy=pts[0].y;
  for (const p of pts){
    minx=Math.min(minx,p.x); maxx=Math.max(maxx,p.x);
    miny=Math.min(miny,p.y); maxy=Math.max(maxy,p.y);
  }
  const pad = 1.0;
  return {minx:minx-pad, maxx:maxx+pad, miny:miny-pad, maxy:maxy+pad};
}

function fitCameraToBounds(b){
  if (!b) return;
  const w = canvas.width, h = canvas.height;
  const margin = 40;
  const dx = Math.max(1e-6, b.maxx - b.minx);
  const dy = Math.max(1e-6, b.maxy - b.miny);
  const sx = (w - 2*margin) / dx;
  const sy = (h - 2*margin) / dy;
  cam.scale = Math.max(5, Math.min(800, Math.min(sx, sy)));
  const cx = (b.minx + b.maxx) * 0.5;
  const cy = (b.miny + b.maxy) * 0.5;
  cam.offsetX = w/2 - cx*cam.scale;
  cam.offsetY = h/2 + cy*cam.scale;
}

function drawMap(){
  if (!ckMap || !ckMap.checked) return;
  if (!mapCache) return;
  const {w,h,res,ox,oy,data} = mapCache;

  // Render map to an offscreen ImageData at map resolution (1 px per cell), then scale by transform.
  // For speed: cache image once per mapCache.
  if (!mapCache._img || mapCache._img_w !== w || mapCache._img_h !== h){
    const img = ctx.createImageData(w, h);
    const out = img.data;

    // UI-layer fix: flip Y when sampling grid cells.
    // Numpy-style arrays often have row 0 at TOP, while our world convention is Y-up.
    // We keep the stored map as-is, and only flip during rendering.
    for (let y = 0; y < h; y++){
      const yy = (h - 1 - y); // <-- flip Y
      for (let x = 0; x < w; x++){
        const src = yy * w + x;
        const v = data[src];

        // convention: -1 unknown, 0 free, 100 occupied
        let c = 255;
        if (v < 0) c = 240;
        else if (v === 0) c = 255;
        else c = 60;

        const di = (y * w + x) * 4;
        out[di + 0] = c;
        out[di + 1] = c;
        out[di + 2] = c;
        out[di + 3] = 255;
      }
    }
    mapCache._img = img;
    mapCache._img_w = w;
    mapCache._img_h = h;
  }

  // compute screen rect of map bounds
  // NOTE: with rotation, the map is no longer screen-axis-aligned.
  // We draw it using canvas transforms consistent with worldToScreen().
  ctx.save();
  ctx.imageSmoothingEnabled = false;

  // putImageData requires 1:1, so draw via offscreen canvas
  if (!mapCache._can){
    const c = document.createElement('canvas');
    c.width = w; c.height = h;
    const cctx = c.getContext('2d');
    cctx.putImageData(mapCache._img, 0, 0);
    mapCache._can = c;
  }

  // Draw in a way that exactly matches:
  //  sx = offsetX + (c*wx - s*wy)*scale
  //  sy = offsetY - (s*wx + c*wy)*scale
  ctx.translate(cam.offsetX, cam.offsetY);
  ctx.scale(cam.scale, -cam.scale);
  ctx.rotate(cam.rot);

  // now unit is "meter" in world, y-up.
  // map origin in meters:
  ctx.translate(ox, oy);
  // convert meters -> cells:
  ctx.scale(res, res);

  // canvas image space is y-down; convert to y-up in this local space
  ctx.scale(1, -1);
  ctx.drawImage(mapCache._can, 0, -h, w, h);

  ctx.restore();
}

function drawTrajectory(){
  if (!ckTraj || !ckTraj.checked) return;
  if (trajPts.length < 2) return;
  ctx.save();
  ctx.strokeStyle = '#000000';

  // 轨迹线稍微细一点，避免和点大小产生明显差异
  ctx.lineWidth = 1.5;

  ctx.beginPath();
  const p0 = worldToScreen(trajPts[0].x, trajPts[0].y);
  ctx.moveTo(p0.x, p0.y);
  for (let i=1;i<trajPts.length;i++){
    const p = worldToScreen(trajPts[i].x, trajPts[i].y);
    ctx.lineTo(p.x, p.y);
  }
  ctx.stroke();

  // pose 点：从半径 3 缩小到 2
  ctx.fillStyle = '#007aff';
  const step = Math.max(1, Math.floor(trajPts.length/300));
  const r = 2; 

  for (let i=0;i<trajPts.length;i+=step){
    ctx.beginPath();
    const p = worldToScreen(trajPts[i].x, trajPts[i].y);
    ctx.arc(p.x, p.y, r, 0, Math.PI*2);
    ctx.fill();
  }
  ctx.restore();
}

function drawPath(points, color, width){
  if (!points || points.length < 2) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  let p = worldToScreen(points[0].x, points[0].y);
  ctx.moveTo(p.x, p.y);
  for (let i=1; i<points.length; ++i){
    p = worldToScreen(points[i].x, points[i].y);
    ctx.lineTo(p.x, p.y);
  }
  ctx.stroke();
  ctx.restore();
}

function drawGlobalPath(frame){
  if (!ckGlobalPath || !ckGlobalPath.checked) return;
  const pts = (frame && frame.global_path) || extraGlobalPath;
  // 采用青色，区分轨迹/局部路径
  drawPath(pts, '#00ffff', 2.5);
}

function drawLocalPath(frame){
  if (!ckLocalPath || !ckLocalPath.checked) return;
  const pts = (frame && frame.local_path) || extraLocalPath;
  // 采用黄色，局部路径更醒目
  drawPath(pts, '#ffd54f', 2.5);
}

function drawDebugPoints(frame){
  if (!ckDebugPoints || !ckDebugPoints.checked) return;

  // 1) frame 自带 point_sets（可选）
  if (frame && Array.isArray(frame.point_sets)){
    for (const s of frame.point_sets){
      if (!s || !Array.isArray(s.points)) continue;
      const color  = s.color || '#ff0000';
      const radius = s.radius_px || 2.0;
      ctx.save();
      ctx.fillStyle = color;
      for (const p of s.points){
        const wx = p.x ?? p[0];
        const wy = p.y ?? p[1];
        const sp = worldToScreen(wx, wy);
        ctx.beginPath();
        ctx.arc(sp.x, sp.y, radius, 0, Math.PI*2);
        ctx.fill();
      }
      ctx.restore();
    }
  }

  // 2) 额外通过 JS 接口注入的 point sets
  for (const key in extraPointSets){
    const layer = extraPointSets[key];
    if (!layer || !Array.isArray(layer.points)) continue;
    const color  = layer.color  || '#ff0000';
    const radius = layer.radius || 2.0;
    ctx.save();
    ctx.fillStyle = color;
    for (const p of layer.points){
      const sp = worldToScreen(p.x, p.y);
      ctx.beginPath();
      ctx.arc(sp.x, sp.y, radius, 0, Math.PI*2);
      ctx.fill();
    }
    ctx.restore();
  }
}

function drawScan(frame){
  if (!ckCloud || !ckCloud.checked) return;
  if (!frame.points || !frame.pose) return;
  const px = frame.pose.x, py = frame.pose.y, yaw = frame.pose.yaw;
  const cy = Math.cos(yaw), sy = Math.sin(yaw);

  ctx.save();
  // Point cloud stays RED (do not inherit other drawing colors)
  ctx.fillStyle = '#ff0000';
  for (let i=0;i<frame.points.length;i+=1){
    const p = frame.points[i];
    const lx = p[0], ly = p[1];
    // local -> world: x forward, y left
    const wx = px + lx*cy - ly*sy;
    const wy = py + lx*sy + ly*cy;
    const sp = worldToScreen(wx, wy);
    ctx.beginPath();
    ctx.arc(sp.x, sp.y, 2, 0, Math.PI*2);
    ctx.fill();
  }
  ctx.restore();
}

function drawRobot(frame){
  if (!ckRobot || !ckRobot.checked) return;
  const p = worldToScreen(frame.pose.x, frame.pose.y);
  const x = p.x;
  const y = p.y;
  const yaw = frame.pose.yaw;

  ctx.save();
  ctx.translate(x, y);
  // IMPORTANT: include view rotation so robot heading is correct after rotating the view
  ctx.rotate(-(yaw + cam.rot)); // screen y is down

  // triangle robot icon
  ctx.fillStyle = '#2457ff';
  ctx.beginPath();
  ctx.moveTo(12, 0);
  ctx.lineTo(-10, 7);
  ctx.lineTo(-10, -7);
  ctx.closePath();
  ctx.fill();

  // center point
  ctx.fillStyle = '#ffffff';
  ctx.beginPath();
  ctx.arc(0,0,2,0,Math.PI*2);
  ctx.fill();

  ctx.restore();
}

// ---------------------- Measure overlay drawing ----------------------
function drawMeasureOverlay(){
  if (!measure.enabled) return;
  if (!measure.a) return;

  const BLUE = '#2457ff'; // 与你机器人主体蓝色保持一致
  const A = worldToScreen(measure.a.x, measure.a.y);

  ctx.save();
  ctx.lineWidth = 2;

  // --- point A: blue circle ---
  ctx.setLineDash([]);              // points are solid
  ctx.strokeStyle = BLUE;
  ctx.fillStyle = 'rgba(255,255,255,0.85)'; // subtle fill for visibility
  ctx.beginPath();
  ctx.arc(A.x, A.y, 6, 0, Math.PI*2);
  ctx.fill();
  ctx.stroke();

  if (!measure.b){
    ctx.restore();
    return;
  }

  const B = worldToScreen(measure.b.x, measure.b.y);

  // --- point B: blue circle ---
  ctx.beginPath();
  ctx.arc(B.x, B.y, 6, 0, Math.PI*2);
  ctx.fill();
  ctx.stroke();

  // --- segment AB: blue dashed line ---
  ctx.strokeStyle = BLUE;
  ctx.setLineDash([8, 6]); // dash pattern
  ctx.beginPath();
  ctx.moveTo(A.x, A.y);
  ctx.lineTo(B.x, B.y);
  ctx.stroke();

  // --- label ---
  ctx.setLineDash([]);
  const mx = (A.x + B.x) * 0.5;
  const my = (A.y + B.y) * 0.5;
  const txt = `${measure.dist_m.toFixed(3)} m`;

  ctx.font = '14px system-ui, sans-serif';
  ctx.textBaseline = 'middle';
  ctx.fillStyle = BLUE;
  ctx.fillText(txt, mx + 8, my - 10);

  ctx.restore();
}

function render(frame){
  if (!frame || frame.error) return;
  clear();

  // V3: RViz-like helpers (grid/axes/scale) anchored in world frame.
  drawRvizGrid();
  drawAxes();
  drawScaleBar();

  drawMap();
  drawGlobalPath(frame);
  drawLocalPath(frame);
  drawTrajectory();
  drawScan(frame);
  drawRobot(frame);
  drawDebugPoints(frame);
  // measure overlay on top
  drawMeasureOverlay();
}

// ---------- data refresh ----------
async function refreshAtTs(tsNs, isDragging){
  if (!meta) return;

  // throttle while dragging
  const minIntervalMs = isDragging ? 33 : 0;
  const now = performance.now();
  if (refreshAtTs._last && (now - refreshAtTs._last) < minIntervalMs) return;
  refreshAtTs._last = now;

  // cancel in-flight
  if (inFlight && inFlight.abort) inFlight.abort();
  const ac = new AbortController();
  inFlight = ac;

  const seq = ++requestSeq;
  try {
    await ensureMap(tsNs);
    const r = await fetch(`/api/frame?ts_ns=${tsNs.toString()}`, {signal: ac.signal});
    const frame = await r.json();
    if (seq !== requestSeq) return; // stale

    lastFrame = frame;
    setInfo(frame);
    render(frame);

    // append trajectory only when playing and not dragging
    if (playing && !dragging && frame.pose){
      const x = frame.pose.x;
      const y = frame.pose.y;
      const last = trajPts.length ? trajPts[trajPts.length - 1] : null;
      if (!last || (Math.abs(last.x - x) + Math.abs(last.y - y)) > 1e-6){
        trajPts.push({x, y});
        if (trajPts.length > 200000) trajPts.shift();
      }
    }
  } catch (e) {
    // ignore abort
  }
}

function updateTimeline(sec){
  tsecEl.textContent = sec.toFixed(2);
  currentTsNs = tsFromSec(sec);
}

// ---------- playback & controls ----------
btnPlay.addEventListener('click', ()=>{
  playing = !playing;
  btnPlay.textContent = playing ? '暂停' : '播放';
  lastAnimMs = performance.now();
});

btnResetTraj.addEventListener('click', ()=>{
  trajPts = [];
  if (lastFrame && lastFrame.pose){
    trajPts.push({x: lastFrame.pose.x, y: lastFrame.pose.y});
    render(lastFrame);
  }
});

selSpeed.addEventListener('change', ()=>{
  speed = parseFloat(selSpeed.value);
  speedLabelEl.textContent = `x${speed}`;
});

// ---------------------- Measure button logic ----------------------
if (btnMeasure){
  btnMeasure.addEventListener('click', ()=>{
    measure.enabled = !measure.enabled;
    // reset points on toggle to avoid stale overlay
    measure.a = null;
    measure.b = null;
    measure.dist_m = 0;
    if (measure.enabled) setMeasureLabel('请选择点A...');
    else setMeasureLabel('');
    if (lastFrame) render(lastFrame);
  });
}

// Screen recorder: record the entire screen (user selects monitor/window/tab), 20 Hz
let recorder = null;
let recChunks = [];
let recStream = null;

function pickRecorderOptions(){
  const candidates = [
    'video/webm;codecs=vp9',
    'video/webm;codecs=vp8',
    'video/webm'
  ];
  for (const mt of candidates){
    if (window.MediaRecorder && MediaRecorder.isTypeSupported && MediaRecorder.isTypeSupported(mt)){
      return { mimeType: mt };
    }
  }
  return {};
}

async function startScreenRecording(){
  // User must select "Entire Screen" in the picker to achieve true full-screen recording.
  // We keep audio disabled by default to avoid permissions surprises.
  recStream = await navigator.mediaDevices.getDisplayMedia({
    video: {
      frameRate: 20
    },
    audio: false
  });

  const opts = pickRecorderOptions();
  recorder = new MediaRecorder(recStream, opts);
  recChunks = [];

  recorder.ondataavailable = (e)=>{ if (e.data && e.data.size) recChunks.push(e.data); };

  // If the user stops sharing from the browser UI, end gracefully.
  const tracks = recStream.getVideoTracks();
  if (tracks && tracks[0]){
    tracks[0].addEventListener('ended', ()=>{
      if (recorder && recorder.state === 'recording'){
        recorder.stop();
      }
    });
  }

  recorder.onstop = ()=>{
    try{
      const mime = (opts && opts.mimeType) ? opts.mimeType : 'video/webm';
      const blob = new Blob(recChunks, {type: mime});
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `screen_record_${Date.now()}.webm`;
      a.click();
      URL.revokeObjectURL(url);
    } finally {
      // Clean up tracks
      if (recStream){
        recStream.getTracks().forEach(t=>t.stop());
      }
      recStream = null;
      recorder = null;
      recChunks = [];
      btnRecord.textContent = '录制';
    }
  };

  recorder.start();
  btnRecord.textContent = '停止';
}

btnRecord.addEventListener('click', async ()=>{
  // Stop
  if (recorder && recorder.state === 'recording'){
    recorder.stop();
    return;
  }

  // Start
  try{
    await startScreenRecording();
  } catch (e){
    // User canceled or permission denied; restore UI state
    recorder = null;
    recStream = null;
    recChunks = [];
    btnRecord.textContent = '录制';
    console.warn('Screen recording canceled/failed:', e);
  }
});

slider.addEventListener('pointerdown', ()=>{ dragging = true; });
slider.addEventListener('pointerup', ()=>{
  dragging = false;
  // After seek/drag, ensure trajectory matches [t0, current] and continue accumulating on play.
  (async()=>{
    await refreshTrajPrefix(currentTsNs, true);
    if (playing) lastAnimMs = performance.now();
    await refreshAtTs(currentTsNs, false);
  })();
});
slider.addEventListener('input', async ()=>{
  const sec = secFromSlider();
  updateTimeline(sec);
  const tsNs = tsFromSec(sec);
  await refreshTrajPrefix(tsNs, false);
  await refreshAtTs(tsNs, true);
});

// ---------- V3: canvas pan/zoom interactions ----------
function clampScale(s){
  return Math.max(2, Math.min(2000, s));
}

function zoomAtScreenPoint(factor, sx, sy){
  // Keep the same world point under cursor.
  // Upgrade: with rotation, still keep anchor via setCamOffsetForAnchor().
  const w = screenToWorld(sx, sy);
  cam.scale = clampScale(cam.scale * factor);
  setCamOffsetForAnchor(w.x, w.y, sx, sy);
}

canvas.addEventListener('pointerdown', (e)=>{
  // left button drag pan OR (Shift+left) rotate
  if (e.button !== 0) return;

  // If measuring, do NOT start pan (otherwise cannot reliably pick points)
  if (measure.enabled) return;

  // Shift+LMB drag => rotate-around-cursor (do not change original pan/zoom behavior)
  panState.active = true;
  panState.startX = e.clientX;
  panState.startY = e.clientY;

  if (e.shiftKey){
    panState.mode = 'rot';
    panState.startRot = cam.rot;

    // Anchor: rotate around cursor point (press-time)
    const {sx, sy} = canvasClientToCanvasPx(e);
    panState.anchorSX = sx;
    panState.anchorSY = sy;
    panState.anchorW = screenToWorld(sx, sy);
  } else {
    panState.mode = 'pan';
    panState.startOX = cam.offsetX;
    panState.startOY = cam.offsetY;
  }

  canvas.setPointerCapture(e.pointerId);
});

canvas.addEventListener('pointermove', (e)=>{
  updateMouseWorld(e);
  if (!panState.active) return;

  const dx = e.clientX - panState.startX;
  const dy = e.clientY - panState.startY;

  if (panState.mode === 'pan'){
    cam.offsetX = panState.startOX + dx;
    cam.offsetY = panState.startOY + dy;
  } else if (panState.mode === 'rot'){
    // Rotate sensitivity: rad per pixel. Tune if needed.
    const k = 0.005;
    cam.rot = panState.startRot + dx * k;

    // Keep the same world point under the cursor (rotate-around-cursor)
    if (panState.anchorW){
      setCamOffsetForAnchor(panState.anchorW.x, panState.anchorW.y, panState.anchorSX, panState.anchorSY);
    }
  }

  if (lastFrame) render(lastFrame);
});

function endPan(e){
  if (!panState.active) return;
  panState.active = false;
  panState.mode = 'pan';
  panState.anchorW = null;
  try { canvas.releasePointerCapture(e.pointerId); } catch (_) {}
}
canvas.addEventListener('pointerup', endPan);
canvas.addEventListener('pointercancel', endPan);
canvas.addEventListener('pointerleave', endPan);

canvas.addEventListener('wheel', (e)=>{
  e.preventDefault();
  const rect = canvas.getBoundingClientRect();
  const sx = (e.clientX - rect.left) * (canvas.width / rect.width);
  const sy = (e.clientY - rect.top) * (canvas.height / rect.height);
  const factor = e.deltaY < 0 ? 1.1 : 1/1.1;
  zoomAtScreenPoint(factor, sx, sy);
  if (lastFrame) render(lastFrame);
}, {passive:false});

canvas.addEventListener('dblclick', ()=>{
  // double click -> fit based on map if present, else traj/pose
  const b = computeBoundsFromMap() || computeBoundsFromTrajOrPose(lastFrame);
  fitCameraToBounds(b);
  if (lastFrame) render(lastFrame);
});

// ---------------------- Measure interactions on canvas ----------------------
// Click to select points (only when measure.enabled). Use click to avoid interfering with wheel/drag logic.
canvas.addEventListener('click', (e)=>{
  if (!measure.enabled) return;

  // Map presence is not strictly required for unit correctness (world is meters),
  // but user asked "地图显示页面的测量工具" - keep it map-page oriented:
  if (!mapCache && meta && meta.has_map) {
    // if map exists but not loaded yet, ignore until loaded
    return;
  }

  const rect = canvas.getBoundingClientRect();
  const sx = (e.clientX - rect.left) * (canvas.width / rect.width);
  const sy = (e.clientY - rect.top) * (canvas.height / rect.height);
  const w = screenToWorld(sx, sy);

  // first click or restart
  if (!measure.a || (measure.a && measure.b)){
    measure.a = {x: w.x, y: w.y};
    measure.b = null;
    measure.dist_m = 0;
    updateMeasureDistanceLabel();
    if (lastFrame) render(lastFrame);
    return;
  }

  // second click
  measure.b = {x: w.x, y: w.y};
  const dx = measure.b.x - measure.a.x;
  const dy = measure.b.y - measure.a.y;
  measure.dist_m = Math.hypot(dx, dy); // meters
  updateMeasureDistanceLabel();
  if (lastFrame) render(lastFrame);
});

// Right-click clears measurement but keeps mode
canvas.addEventListener('contextmenu', (e)=>{
  if (!measure.enabled) return;
  e.preventDefault();
  clearMeasure(true);
  if (lastFrame) render(lastFrame);
});

// ESC exits measure mode and clears
window.addEventListener('keydown', (e)=>{
  if (e.key === 'Escape'){
    if (!measure.enabled && !measure.a && !measure.b) return;
    measure.enabled = false;
    clearMeasure(false);
    if (lastFrame) render(lastFrame);
  }
});

// Clear mouse world coordinate when cursor leaves canvas.
canvas.addEventListener('mouseleave', ()=>{
  if (mouseWorldEl) mouseWorldEl.textContent = '(- , -)';
});

// Overlay controls: re-render immediately
for (const c of [ckMap, ckCloud, ckTraj, ckRobot, ckGrid, ckAxes, ckScale]){
  if (!c) continue;
  c.addEventListener('change', ()=>{ if (lastFrame) render(lastFrame); });
}
btnFit?.addEventListener('click', ()=>{
  const b = computeBoundsFromMap() || computeBoundsFromTrajOrPose(lastFrame);
  fitCameraToBounds(b);
  if (lastFrame) render(lastFrame);
});
btnFitMap?.addEventListener('click', ()=>{
  fitCameraToBounds(computeBoundsFromMap());
  if (lastFrame) render(lastFrame);
});
btnFitTraj?.addEventListener('click', ()=>{
  fitCameraToBounds(computeBoundsFromTrajOrPose(lastFrame));
  if (lastFrame) render(lastFrame);
});
btnResetView?.addEventListener('click', ()=>{
  cam.scale = 80.0;
  cam.offsetX = canvas.width/2;
  cam.offsetY = canvas.height/2;
  cam.rot = 0.0; // reset rotation
  if (lastFrame) render(lastFrame);
});

function nsToTimeText(tsNs){
  // If meta.base_unix_ms exists, prefer it.
  // Otherwise show relative time.
  if (meta && meta.base_unix_ms){
    const ms = Number(BigInt(meta.base_unix_ms) + (BigInt(tsNs) - BigInt(meta.t0_ns))/1000000n);
    const d = new Date(ms);
    return d.toISOString().replace('T',' ').replace('Z','');
  }
  const rel = (Number(BigInt(tsNs) - BigInt(meta.t0_ns)))/1e9;
  return `t0+${rel.toFixed(3)}s`;
}

function loop(){
  if (playing && meta && !dragging){
    const now = performance.now();
    const dt = (now - lastAnimMs) / 1000.0;
    lastAnimMs = now;

    let sec = secFromSlider();
    sec += dt * speed;
    if (sec > meta.duration_sec) sec = 0;
    slider.value = sliderFromSec(sec);
    updateTimeline(sec);

    // update time label from local conversion (server also provides)
    txtTimeEl.textContent = nsToTimeText(tsFromSec(sec));

    refreshAtTs(tsFromSec(sec), false);
  }
  requestAnimationFrame(loop);
}

(async function init(){
  initOverlayAccordion();
  meta = await getMeta();
  slider.max = 10000;
  slider.value = 0;

  speed = parseFloat(selSpeed.value);
  speedLabelEl.textContent = `x${speed}`;

  // initial
  const sec = 0;
  updateTimeline(sec);
  txtTimeEl.textContent = nsToTimeText(tsFromSec(sec));

  // initialize camera center
  cam.scale = 80.0;
  cam.offsetX = canvas.width/2;
  cam.offsetY = canvas.height/2;
  cam.rot = 0.0;

  await ensureMap(tsFromSec(sec));
  await refreshTrajPrefix(tsFromSec(sec), true);
  await refreshAtTs(tsFromSec(sec), false);

  // Auto-fit once on load (prefer map, else traj/pose)
  const b0 = computeBoundsFromMap() || computeBoundsFromTrajOrPose(lastFrame);
  fitCameraToBounds(b0);
  if (lastFrame) render(lastFrame);

  // first-time fit based on map (preferred) or traj/pose
  const b = computeBoundsFromMap() || computeBoundsFromTrajOrPose(lastFrame);
  fitCameraToBounds(b);
  if (lastFrame) render(lastFrame);

  // init measure label state
  clearMeasure(false);

  requestAnimationFrame(loop);
})();

// -------- public helper APIs for debug / external scripts --------
window.viewerSetGlobalPath = function(points){
  // points: [{x,y}, ...]
  extraGlobalPath = Array.isArray(points) ? points : null;
  if (lastFrame) render(lastFrame);
};

window.viewerSetLocalPath = function(points){
  extraLocalPath = Array.isArray(points) ? points : null;
  if (lastFrame) render(lastFrame);
};

window.viewerClearPaths = function(){
  extraGlobalPath = null;
  extraLocalPath  = null;
  if (lastFrame) render(lastFrame);
};

window.viewerSetPointSet = function(key, color, radiusPx, points){
  if (!key) key = 'default';
  extraPointSets[key] = {
    color:  color  || '#ff0000',
    radius: radiusPx || 2.0,
    points: (Array.isArray(points) ? points.map(p => ({x:p.x ?? p[0], y:p.y ?? p[1]})) : [])
  };
  if (lastFrame) render(lastFrame);
};

window.viewerClearPointSet = function(key){
  if (key){
    delete extraPointSets[key];
  }else{
    extraPointSets = {};
  }
  if (lastFrame) render(lastFrame);
};
