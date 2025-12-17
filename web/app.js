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
let cam = {
  scale: 80.0,   // px per meter
  offsetX: 0.0,  // px
  offsetY: 0.0,  // px
};
let panState = { active:false, startX:0, startY:0, startOX:0, startOY:0 };

// derived state
let lastFrame = null;
let trajPts = []; // appended while playing
let currentTsNs = 0n;

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
  poseTsEl.textContent = frame.pose_ts ?? '-';
  scanTsEl.textContent = frame.scan_ts ?? '-';
  mapTsEl.textContent = frame.map_ts ?? (mapCache ? mapCache.ts : '-');

  poseTxtEl.textContent = `(${frame.pose.x.toFixed(2)}, ${frame.pose.y.toFixed(2)}, ${frame.pose.yaw.toFixed(2)})`;

  if (frame.imu){
    imuPitchEl.textContent = fmt2(frame.imu.pitch);
    imuRollEl.textContent = fmt2(frame.imu.roll);
    imuYawEl.textContent = fmt2(frame.imu.yaw);
  } else {
    imuPitchEl.textContent = imuRollEl.textContent = imuYawEl.textContent = '-';
  }

  if (frame.odom){
    const p = frame.odom;
    odomEl.textContent = `(${p.x.toFixed(2)}, ${p.y.toFixed(2)}, ${p.yaw.toFixed(2)})`;
  } else {
    odomEl.textContent = '-';
  }

  slipEl.textContent = (frame.slip !== null && frame.slip !== undefined) ? String(frame.slip) : '-';

  txtStateEl.textContent = frame.state ?? '-';

  // time display
  txtTimeEl.textContent = frame.time_text ?? '-';
}

// ---------- rendering ----------
function clear(){
  ctx.clearRect(0,0,canvas.width,canvas.height);
}

function screenToWorld(sx, sy){
  return {
    x: (sx - cam.offsetX) / cam.scale,
    y: (cam.offsetY - sy) / cam.scale,
  };
}

function worldToScreen(wx, wy){
  return {
    x: cam.offsetX + wx*cam.scale,
    y: cam.offsetY - wy*cam.scale,
  };
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
  ctx.lineWidth = 1;

  // minor
  if (showMinor){
    ctx.strokeStyle = 'rgba(0,0,0,0.08)';
    for (let x = startXMinor; x <= maxx; x += minorMeters){
      const sx = worldToScreen(x, 0).x;
      ctx.beginPath(); ctx.moveTo(sx, 0); ctx.lineTo(sx, h); ctx.stroke();
    }
    for (let y = startYMinor; y <= maxy; y += minorMeters){
      const sy = worldToScreen(0, y).y;
      ctx.beginPath(); ctx.moveTo(0, sy); ctx.lineTo(w, sy); ctx.stroke();
    }
  }

  // major
  const startXMajor = Math.floor(minx / majorMeters) * majorMeters;
  const startYMajor = Math.floor(miny / majorMeters) * majorMeters;
  ctx.strokeStyle = 'rgba(0,0,0,0.18)';
  for (let x = startXMajor; x <= maxx; x += majorMeters){
    const sx = worldToScreen(x, 0).x;
    ctx.beginPath(); ctx.moveTo(sx, 0); ctx.lineTo(sx, h); ctx.stroke();
  }
  for (let y = startYMajor; y <= maxy; y += majorMeters){
    const sy = worldToScreen(0, y).y;
    ctx.beginPath(); ctx.moveTo(0, sy); ctx.lineTo(w, sy); ctx.stroke();
  }

  // coordinate labels on major grid (lightweight, edges only)
  ctx.fillStyle = 'rgba(0,0,0,0.55)';
  ctx.font = '12px system-ui, sans-serif';
  ctx.textBaseline = 'top';
  for (let x = startXMajor; x <= maxx; x += majorMeters){
    const sx = worldToScreen(x, 0).x;
    if (sx < 0 || sx > w) continue;
    ctx.fillText(`${x.toFixed(1)}m`, sx + 2, 2);
  }
  ctx.textBaseline = 'bottom';
  for (let y = startYMajor; y <= maxy; y += majorMeters){
    const sy = worldToScreen(0, y).y;
    if (sy < 0 || sy > h) continue;
    ctx.fillText(`${y.toFixed(1)}m`, 2, sy - 2);
  }

  ctx.restore();
}

function drawAxes(){
  if (!ckAxes || !ckAxes.checked) return;
  const w = canvas.width, h = canvas.height;
  const o = worldToScreen(0,0);
  ctx.save();
  ctx.lineWidth = 2;

  // X axis (red)
  ctx.strokeStyle = '#d62728';
  ctx.beginPath(); ctx.moveTo(0, o.y); ctx.lineTo(w, o.y); ctx.stroke();
  // Y axis (green)
  ctx.strokeStyle = '#2ca02c';
  ctx.beginPath(); ctx.moveTo(o.x, 0); ctx.lineTo(o.x, h); ctx.stroke();

  ctx.fillStyle = '#d62728';
  ctx.font = 'bold 14px system-ui, sans-serif';
  ctx.fillText('X', Math.min(w-18, o.x + 6), Math.min(h-6, o.y + 18));
  ctx.fillStyle = '#2ca02c';
  ctx.fillText('Y', Math.min(w-18, o.x + 18), Math.max(16, o.y - 6));

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
  ctx.strokeStyle = 'rgba(0,0,0,0.75)';
  ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(x0, y); ctx.lineTo(x1, y); ctx.stroke();
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(x0, y-8); ctx.lineTo(x0, y+8); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(x1, y-8); ctx.lineTo(x1, y+8); ctx.stroke();

  ctx.fillStyle = 'rgba(0,0,0,0.75)';
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
  const x0 = worldToScreen(ox, oy + h*res).x;
  const y0 = worldToScreen(ox, oy + h*res).y; // top-left in screen
  const x1 = worldToScreen(ox + w*res, oy).x;
  const y1 = worldToScreen(ox + w*res, oy).y;
  const sw = x1 - x0;
  const sh = y1 - y0;

  // draw with nearest-neighbor style
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
  ctx.drawImage(mapCache._can, x0, y0, sw, sh);
  ctx.restore();
}

function drawTrajectory(){
  if (!ckTraj || !ckTraj.checked) return;
  if (trajPts.length < 2) return;
  ctx.save();
  ctx.strokeStyle = '#000000';
  ctx.lineWidth = 2;
  ctx.beginPath();
  const p0 = worldToScreen(trajPts[0].x, trajPts[0].y);
  ctx.moveTo(p0.x, p0.y);
  for (let i=1;i<trajPts.length;i++){
    const p = worldToScreen(trajPts[i].x, trajPts[i].y);
    ctx.lineTo(p.x, p.y);
  }
  ctx.stroke();
  // red points
  ctx.fillStyle = '#000000';
  for (let i=0;i<trajPts.length;i+=Math.max(1, Math.floor(trajPts.length/300))){
    ctx.beginPath();
    const p = worldToScreen(trajPts[i].x, trajPts[i].y);
    ctx.arc(p.x, p.y, 3, 0, Math.PI*2);
    ctx.fill();
  }
  ctx.restore();
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
  ctx.rotate(-yaw); // screen y is down

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

function render(frame){
  if (!frame || frame.error) return;
  clear();

  // V3: RViz-like helpers (grid/axes/scale) anchored in world frame.
  drawRvizGrid();
  drawAxes();
  drawScaleBar();

  drawMap();
  drawTrajectory();
  drawScan(frame);
  drawRobot(frame);
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
  const world = screenToWorld(sx, sy);
  cam.scale = clampScale(cam.scale * factor);
  cam.offsetX = sx - world.x * cam.scale;
  cam.offsetY = sy + world.y * cam.scale;
}

canvas.addEventListener('pointerdown', (e)=>{
  // left button drag pan
  if (e.button !== 0) return;
  panState.active = true;
  panState.startX = e.clientX;
  panState.startY = e.clientY;
  panState.startOX = cam.offsetX;
  panState.startOY = cam.offsetY;
  canvas.setPointerCapture(e.pointerId);
});

canvas.addEventListener('pointermove', (e)=>{
  if (!panState.active) return;
  const dx = e.clientX - panState.startX;
  const dy = e.clientY - panState.startY;
  cam.offsetX = panState.startOX + dx;
  cam.offsetY = panState.startOY + dy;
  if (lastFrame) render(lastFrame);
});

function endPan(e){
  if (!panState.active) return;
  panState.active = false;
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

  requestAnimationFrame(loop);
})();
