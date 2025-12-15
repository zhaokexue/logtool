const slider = document.getElementById("slider");
const tsecEl = document.getElementById("tsec");
const poseTsEl = document.getElementById("pose_ts");
const scanTsEl = document.getElementById("scan_ts");
const poseTxtEl = document.getElementById("pose_txt");
const playBtn = document.getElementById("play");
const resetTrajBtn = document.getElementById("reset_traj");

const cloud = document.getElementById("cloud");
const ctx = cloud.getContext("2d");

const traj = document.getElementById("traj");
const tctx = traj.getContext("2d");

let meta = null;
let playing = false;
let lastFetchMs = 0;
let lastFrame = null;

// trajectory incremental cache (world)
let trajPts = []; // [{x,y}]
let appendEnabled = false; // only when playing

function nowMs(){ return performance.now(); }

async function getMeta(){
  const r = await fetch("/api/meta");
  return await r.json();
}

function tsFromSec(sec){
  return BigInt(meta.t0_ns) + BigInt(Math.floor(sec * 1e9));
}

async function fetchFrame(ts_ns){
  const r = await fetch(`/api/frame?ts_ns=${ts_ns.toString()}`);
  return await r.json();
}

// Canvas helpers
function clearCanvas(c, cctx){
  cctx.clearRect(0,0,c.width,c.height);
}

function drawAxes(c, cctx){
  const w=c.width,h=c.height;
  cctx.strokeStyle="#eee";
  cctx.beginPath();
  cctx.moveTo(w/2,0); cctx.lineTo(w/2,h);
  cctx.moveTo(0,h/2); cctx.lineTo(w,h/2);
  cctx.stroke();

  cctx.fillStyle="#444";
  cctx.fillText("+x", w-20, h/2-6);
  cctx.fillText("+y", w/2+6, 14);
}

// local cloud render
function drawCloud(points){
  clearCanvas(cloud, ctx);
  drawAxes(cloud, ctx);

  const w=cloud.width,h=cloud.height;
  const scale = 35; // px per meter (adjust)
  const ox = w/2, oy = h/2;

  // robot origin
  ctx.fillStyle="blue";
  ctx.beginPath();
  ctx.arc(ox, oy, 4, 0, Math.PI*2);
  ctx.fill();

  // hits
  ctx.fillStyle="red";
  for (const p of points){
    const x = ox + p[0]*scale;
    const y = oy - p[1]*scale; // y-left -> screen up
    ctx.beginPath();
    ctx.arc(x, y, 2, 0, Math.PI*2);  // 点大小在这里调
    ctx.fill();
  }
}

// trajectory render (world)
function drawTraj(pose){
  clearCanvas(traj, tctx);

  const w=traj.width,h=traj.height;
  const margin=30;

  // simple auto-fit based on cached points
  const all = trajPts.length ? trajPts : [{x:pose.x, y:pose.y}];
  let minx=all[0].x, maxx=all[0].x, miny=all[0].y, maxy=all[0].y;
  for (const p of all){
    minx=Math.min(minx,p.x); maxx=Math.max(maxx,p.x);
    miny=Math.min(miny,p.y); maxy=Math.max(maxy,p.y);
  }
  const dx=Math.max(1e-3, maxx-minx);
  const dy=Math.max(1e-3, maxy-miny);
  const sx=(w-2*margin)/dx;
  const sy=(h-2*margin)/dy;
  const s=Math.min(sx,sy);

  function tx(x){ return margin + (x-minx)*s; }
  function ty(y){ return h-margin - (y-miny)*s; }

  // polyline
  if (trajPts.length >= 2){
    tctx.strokeStyle="green";
    tctx.lineWidth=2;
    tctx.beginPath();
    tctx.moveTo(tx(trajPts[0].x), ty(trajPts[0].y));
    for (let i=1;i<trajPts.length;i++){
      tctx.lineTo(tx(trajPts[i].x), ty(trajPts[i].y));
    }
    tctx.stroke();
  }

  // current pose
  tctx.fillStyle="blue";
  tctx.beginPath();
  tctx.arc(tx(pose.x), ty(pose.y), 4, 0, Math.PI*2);
  tctx.fill();
}

function setInfo(frame){
  if (frame.error) return;
  poseTsEl.textContent = frame.pose_ts;
  scanTsEl.textContent = frame.scan_ts;
  poseTxtEl.textContent = `(${frame.pose.x.toFixed(2)}, ${frame.pose.y.toFixed(2)}) yaw=${frame.pose.yaw.toFixed(2)}`;
}

// Throttle fetch during dragging (e.g., 30Hz)
async function refreshAt(sec, isDragging){
  tsecEl.textContent = sec.toFixed(2);

  const ts = tsFromSec(sec);
  const ms = nowMs();
  const minInterval = isDragging ? 33 : 0; // 30Hz while dragging
  if (ms - lastFetchMs < minInterval) return;
  lastFetchMs = ms;

  const frame = await fetchFrame(ts);
  lastFrame = frame;
  setInfo(frame);
  if (!frame.error){
    drawCloud(frame.points);
    drawTraj(frame.pose);

    // append trajectory only if playing
    if (appendEnabled){
      trajPts.push({x: frame.pose.x, y: frame.pose.y});
      // 限制缓存大小（2小时播放也不会无限增长）
      if (trajPts.length > 200000) trajPts.shift();
    }
  }
}

// Slider interactions
let dragging = false;
slider.addEventListener("pointerdown", ()=>{ dragging=true; });
slider.addEventListener("pointerup",   ()=>{ dragging=false; });

// slider is 0..10000 -> map to duration
function secFromSlider(v){
  const t = Number(v) / 10000.0;
  return t * meta.duration_sec;
}

slider.addEventListener("input", async ()=>{
  const sec = secFromSlider(slider.value);
  // dragging: refresh realtime but not append trajectory
  appendEnabled = false;
  await refreshAt(sec, true);
});

playBtn.addEventListener("click", ()=>{
  playing = !playing;
  playBtn.textContent = playing ? "Pause" : "Play";
  appendEnabled = playing;
});

resetTrajBtn.addEventListener("click", ()=>{
  trajPts = [];
  if (lastFrame && !lastFrame.error){
    drawTraj(lastFrame.pose);
  }
});

// Playback loop
async function loop(){
  if (playing && meta){
    const sec = secFromSlider(slider.value);
    let next = sec + 0.05; // 20Hz
    if (next > meta.duration_sec) next = 0.0;
    slider.value = Math.floor((next / meta.duration_sec) * 10000);
    appendEnabled = true;
    await refreshAt(next, false);
  }
  requestAnimationFrame(loop);
}

(async function init(){
  meta = await getMeta();
  slider.max = 10000;
  slider.value = 0;

  // initial frame
  appendEnabled = false;
  await refreshAt(0.0, false);

  requestAnimationFrame(loop);
})();

