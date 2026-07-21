// Custom read-only dashboard. Polls /state (~15 Hz) for smooth realtime pose
// windows and reloads /map.png only when the map revision changes. No build
// step, no external bridge. The reported track/odom rates are the BACKEND
// cadence (from message header stamps), not what this page receives.
"use strict";

const el = (id) => document.getElementById(id);
const canvases = {
  map: el("map"),
  odom: el("odom"),
  orb: el("orb"),
  corrected: el("corrected"),
  loopgraph: el("loopgraph"),
};
const chartCanvases = {
  trackHz: el("chartTrackHz"),
  odomHz: el("chartOdomHz"),
  kp: el("chartKp"),
  loops: el("chartLoops"),
};
const mapCtx = canvases.map.getContext("2d");

let mapImg = null;
let mapMeta = null;
let requestedRevision = -1;
let loadedRevision = -1;
let mapRequestToken = 0;
let lastState = null;
let activeTab = "map";

// ── charts history (client-side rolling buffers, ~15 Hz poll cadence) ───────
const CHART_HISTORY_MAX = 450; // ~30s at 15 Hz
const history = {
  t: [],
  trackHz: [],
  odomHz: [],
  kp: [],
  loops: [],
};
function pushHistory(st) {
  const t = st.tracking || {};
  history.t.push(performance.now());
  history.trackHz.push(t.hz ?? 0);
  history.odomHz.push((st.odom && st.odom.hz) || 0);
  history.kp.push(t.keypoints || 0);
  history.loops.push((st.graph && st.graph.loops) ? st.graph.loops.length : 0);
  while (history.t.length > CHART_HISTORY_MAX) {
    for (const k of Object.keys(history)) history[k].shift();
  }
}

// ── canvas sizing (DPR-aware) ────────────────────────────────────────────────
function sizeCanvas(canvas) {
  const r = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(1, Math.round(r.width * dpr));
  const h = Math.max(1, Math.round(r.height * dpr));
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }
}
function resizeAll() {
  for (const c of Object.values(canvases)) sizeCanvas(c);
  for (const c of Object.values(chartCanvases)) sizeCanvas(c);
  draw();
}
window.addEventListener("resize", resizeAll);

// ── map image loading (only when revision changes) ───────────────────────────
function maybeReloadMap(meta, actualRevision) {
  if (!meta || meta.width <= 0) return;
  const visibleRevision = Number(meta.revision);
  const revision = Number(actualRevision);
  if (!Number.isFinite(visibleRevision) || !Number.isFinite(revision)) return;
  if (visibleRevision !== revision) return;
  if (revision === requestedRevision || revision === loadedRevision) return;
  const token = ++mapRequestToken;
  requestedRevision = revision;
  const requestedMeta = { ...meta };
  const img = new Image();
  img.onload = () => {
    if (token !== mapRequestToken || requestedRevision !== revision) return;
    mapImg = img;
    mapMeta = requestedMeta;
    loadedRevision = revision;
    drawMap();
  };
  img.onerror = () => {
    if (token === mapRequestToken && requestedRevision === revision) requestedRevision = -1;
  };
  img.src = "/map.png?rev=" + revision;
}

// ── map panel (uses the map's own world AABB) ────────────────────────────────
function drawMap() {
  const canvas = canvases.map;
  mapCtx.fillStyle = "#0b0f14";
  mapCtx.fillRect(0, 0, canvas.width, canvas.height);
  if (!mapMeta || mapMeta.width <= 0 || !mapImg) return;
  const res = mapMeta.resolution;
  const worldW = mapMeta.width * res;
  const worldH = mapMeta.height * res;
  const scale = Math.min(canvas.width / worldW, canvas.height / worldH) * 0.95;
  const drawW = worldW * scale, drawH = worldH * scale;
  const offX = (canvas.width - drawW) / 2, offY = (canvas.height - drawH) / 2;
  const toS = (x, y) => [offX + (x - mapMeta.origin_x) * scale,
                         offY + (worldH - (y - mapMeta.origin_y)) * scale];
  mapCtx.imageSmoothingEnabled = false;
  mapCtx.drawImage(mapImg, offX, offY, drawW, drawH);
  const st = lastState;
  if (!st) return;
  const stroke = (pts, color) => {
    if (!pts || pts.length < 2) return;
    mapCtx.strokeStyle = color; mapCtx.lineWidth = Math.max(1.5, 0.03 * scale);
    mapCtx.beginPath();
    pts.forEach((point, i) => {
      const p = Array.isArray(point) ? { x: point[0], y: point[1] } : point;
      const [sx, sy] = toS(p.x, p.y);
      i === 0 ? mapCtx.moveTo(sx, sy) : mapCtx.lineTo(sx, sy);
    });
    mapCtx.stroke();
  };
  stroke((st.paths && st.paths.corrected) || [], "#38bdf8");
  stroke((st.fallback && st.fallback.trail) || [], "#facc15");
  const prov = (st.paths && st.paths.provisional) || [];
  mapCtx.fillStyle = "#facc15";
  prov.forEach((pt) => {
    const [sx, sy] = toS(pt[0], pt[1]);
    mapCtx.beginPath(); mapCtx.arc(sx, sy, Math.max(2, 0.06 * scale), 0, Math.PI * 2); mapCtx.fill();
  });
  const p = st.tracking && st.tracking.pose;
  if (p) drawPoseMarker(mapCtx, toS(p.x, p.y), p.yaw, "#22c55e", Math.max(4, 0.12 * scale));
}

function drawPoseMarker(ctx, [sx, sy], yaw, color, r) {
  ctx.fillStyle = color;
  ctx.beginPath(); ctx.arc(sx, sy, r, 0, Math.PI * 2); ctx.fill();
  if (typeof yaw === "number") {
    ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(sx, sy);
    ctx.lineTo(sx + Math.cos(yaw) * r * 2, sy - Math.sin(yaw) * r * 2); ctx.stroke();
  }
}

// ── generic self-scaling trajectory / graph panel ────────────────────────────
function asXY(p) { return Array.isArray(p) ? { x: p[0], y: p[1] } : p; }

function fitBounds(groups) {
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const g of groups) {
    for (const raw of g || []) {
      if (raw == null) continue;
      const p = asXY(raw);
      if (!p || !Number.isFinite(p.x) || !Number.isFinite(p.y)) continue;
      if (p.x < minX) minX = p.x; if (p.x > maxX) maxX = p.x;
      if (p.y < minY) minY = p.y; if (p.y > maxY) maxY = p.y;
    }
  }
  if (!Number.isFinite(minX)) return null;
  return { minX, minY, maxX, maxY };
}

// Draws into `canvas` fitting the union of points/nodes/pose to the view.
function drawPanel(canvas, opts) {
  const ctx = canvas.getContext("2d");
  ctx.fillStyle = "#0b0f14";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  const pts = (opts.points || []).map(asXY);
  const nodes = opts.nodes || [];
  const poseArr = opts.pose ? [opts.pose] : [];
  const b = fitBounds([pts, nodes, poseArr]);
  if (!b) {
    ctx.fillStyle = "#334155"; ctx.font = "13px system-ui";
    ctx.fillText("waiting…", 10, 20);
    return;
  }
  const cx = (b.minX + b.maxX) / 2, cy = (b.minY + b.maxY) / 2;
  const half = Math.max((b.maxX - b.minX), (b.maxY - b.minY), 0.5) * 0.6;
  const W = canvas.width, H = canvas.height;
  const scale = Math.min(W, H) / (2 * half);
  const toS = (x, y) => [W / 2 + (x - cx) * scale, H / 2 - (y - cy) * scale];

  // origin crosshair
  const [ox, oy] = toS(0, 0);
  ctx.strokeStyle = "#1e293b"; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(ox - 8, oy); ctx.lineTo(ox + 8, oy);
  ctx.moveTo(ox, oy - 8); ctx.lineTo(ox, oy + 8); ctx.stroke();

  if (opts.color && pts.length > 1) {
    ctx.strokeStyle = opts.color; ctx.lineWidth = 2;
    ctx.beginPath();
    pts.forEach((p, i) => { const [sx, sy] = toS(p.x, p.y); i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy); });
    ctx.stroke();
  }
  if (opts.edges) {
    ctx.strokeStyle = "#f43f5e"; ctx.lineWidth = 1.5;
    for (const e of opts.edges) {
      const [ax, ay] = toS(e.from.x, e.from.y), [bx, by] = toS(e.to.x, e.to.y);
      ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
    }
  }
  if (nodes.length) {
    ctx.fillStyle = opts.nodeColor || "#22d3ee";
    for (const n of nodes) { const [sx, sy] = toS(n.x, n.y); ctx.beginPath(); ctx.arc(sx, sy, 1.6, 0, Math.PI * 2); ctx.fill(); }
  }
  if (opts.pose) drawPoseMarker(ctx, toS(opts.pose.x, opts.pose.y), opts.pose.yaw, opts.poseColor || "#22c55e", 5);
  // scale bar (1 m)
  ctx.strokeStyle = "#475569"; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(10, H - 12); ctx.lineTo(10 + scale, H - 12); ctx.stroke();
  ctx.fillStyle = "#64748b"; ctx.font = "11px system-ui"; ctx.fillText("1 m", 12, H - 16);
}

// ── line chart panel (time-series, self-scaling to data range) ──────────────
function drawLineChart(canvas, values, opts) {
  opts = opts || {};
  const ctx = canvas.getContext("2d");
  const W = canvas.width, H = canvas.height;
  ctx.fillStyle = "#0b0f14";
  ctx.fillRect(0, 0, W, H);
  if (!values || values.length < 2) {
    ctx.fillStyle = "#334155"; ctx.font = "13px system-ui";
    ctx.fillText("waiting…", 10, 20);
    return;
  }
  const pad = { l: 34, r: 8, t: 8, b: 8 };
  const plotW = Math.max(1, W - pad.l - pad.r);
  const plotH = Math.max(1, H - pad.t - pad.b);
  let minV = Math.min(...values), maxV = Math.max(...values);
  if (opts.minZero) minV = Math.min(0, minV);
  if (maxV - minV < 1e-6) { maxV = minV + 1; }
  const yFor = (v) => pad.t + plotH - ((v - minV) / (maxV - minV)) * plotH;
  const xFor = (i) => pad.l + (i / (values.length - 1)) * plotW;

  // gridlines + labels (min/mid/max)
  ctx.strokeStyle = "#1e293b"; ctx.lineWidth = 1; ctx.font = "10px system-ui"; ctx.fillStyle = "#64748b";
  for (const frac of [0, 0.5, 1]) {
    const v = minV + frac * (maxV - minV);
    const y = yFor(v);
    ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(W - pad.r, y); ctx.stroke();
    ctx.fillText(v.toFixed(opts.decimals ?? 0), 2, y + 3);
  }

  // optional target line (e.g. 30 Hz)
  if (typeof opts.target === "number" && opts.target >= minV && opts.target <= maxV) {
    ctx.strokeStyle = "#334155"; ctx.setLineDash([4, 3]);
    const y = yFor(opts.target);
    ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(W - pad.r, y); ctx.stroke();
    ctx.setLineDash([]);
  }

  ctx.strokeStyle = opts.color || "#38bdf8"; ctx.lineWidth = 1.75;
  ctx.beginPath();
  values.forEach((v, i) => { const x = xFor(i), y = yFor(v); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
  ctx.stroke();

  // fill under last point marker
  const lastX = xFor(values.length - 1), lastY = yFor(values[values.length - 1]);
  ctx.fillStyle = opts.color || "#38bdf8";
  ctx.beginPath(); ctx.arc(lastX, lastY, 2.5, 0, Math.PI * 2); ctx.fill();
}

function drawCharts() {
  drawLineChart(chartCanvases.trackHz, history.trackHz, { color: "#38bdf8", minZero: true, decimals: 1, target: 30 });
  drawLineChart(chartCanvases.odomHz, history.odomHz, { color: "#facc15", minZero: true, decimals: 1 });
  drawLineChart(chartCanvases.kp, history.kp, { color: "#a78bfa", minZero: true, decimals: 0 });
  drawLineChart(chartCanvases.loops, history.loops, { color: "#22d3ee", minZero: true, decimals: 0 });
  const last = (arr) => (arr.length ? arr[arr.length - 1] : 0);
  el("chartTrackHzNow").textContent = last(history.trackHz).toFixed(1);
  el("chartOdomHzNow").textContent = last(history.odomHz).toFixed(1);
  el("chartKpNow").textContent = last(history.kp);
  el("chartLoopsNow").textContent = last(history.loops);
}

function draw() {
  if (activeTab === "map") {
    drawMap();
    drawCharts();
  } else {
    const st = lastState || {};
    const odom = st.odom || {};
    const orb = st.orb || {};
    const graph = st.graph || {};
    drawPanel(canvases.odom, { points: odom.trail || [], pose: odom.pose, color: "#facc15", poseColor: "#facc15" });
    drawPanel(canvases.orb, { points: orb.trail || [], pose: orb.pose, color: "#a78bfa", poseColor: "#a78bfa" });
    drawPanel(canvases.corrected, {
      points: (st.paths && st.paths.corrected) || [],
      pose: (st.paths && st.paths.corrected && st.paths.corrected.length) ? st.paths.corrected[st.paths.corrected.length - 1] : null,
      color: "#38bdf8", poseColor: "#38bdf8",
    });
    drawPanel(canvases.loopgraph, { nodes: graph.keyframes || [], edges: graph.loops || [], nodeColor: "#22d3ee" });
  }
}

// ── tab switching ────────────────────────────────────────────────────────────
function setActiveTab(tab) {
  activeTab = tab;
  const mapTab = tab === "map";
  el("tab-map").classList.toggle("active", mapTab);
  el("tab-map").setAttribute("aria-selected", String(mapTab));
  el("tab-charts").classList.toggle("active", !mapTab);
  el("tab-charts").setAttribute("aria-selected", String(!mapTab));
  el("view-map").hidden = !mapTab;
  el("view-charts").hidden = mapTab;
  resizeAll();
}
el("tab-map").addEventListener("click", () => setActiveTab("map"));
el("tab-charts").addEventListener("click", () => setActiveTab("charts"));

// ── status / events UI ────────────────────────────────────────────────────────
function trackClass(state) {
  if (state === "OK") return "status-ok";
  if (state === "RECENTLY_LOST") return "status-warn";
  if (state === "LOST") return "status-error";
  if (state === "recovery_pending") return "status-warn";
  return "";
}

function renderEvents(events) {
  const ul = el("events");
  ul.innerHTML = "";
  for (const e of events || []) {
    const li = document.createElement("li");
    const type = document.createElement("span");
    type.className = "type"; type.textContent = e.type; li.appendChild(type);
    if (e.detail) {
      const d = document.createElement("span");
      d.className = "detail"; d.textContent = " — " + e.detail; li.appendChild(d);
    }
    ul.appendChild(li);
  }
}

function applyState(st) {
  lastState = st;
  const conn = el("conn");
  conn.textContent = "connected"; conn.className = "badge badge-connected";
  const t = st.tracking || {};
  const trackBadge = el("track");
  trackBadge.textContent = t.state || "--";
  trackBadge.className = "badge " + trackClass(t.state);
  const trackHz = (t.hz ?? 0).toFixed(1);
  const odomHz = (st.odom && st.odom.hz != null ? st.odom.hz : 0).toFixed(1);
  el("trackHz").textContent = trackHz;
  el("odomHz").textContent = odomHz;
  el("orbHz").textContent = trackHz + " Hz";
  el("odomHz2").textContent = odomHz + " Hz";
  el("kp").textContent = t.keypoints || 0;
  const rev = st.map_revision || {};
  el("graphRev").textContent = rev.graph_revision ?? (st.graph && st.graph.revision) ?? 0;
  el("mapRev").textContent = rev.map_revision ?? (st.map && st.map.revision) ?? 0;
  const loopN = (st.graph && st.graph.loops) ? st.graph.loops.length : 0;
  el("loopCount").textContent = loopN;
  el("loopInfo").textContent = loopN + " edges";
  const recovery = el("recovery");
  recovery.hidden = st.state !== "recovery_pending";
  recovery.textContent = "RECOVERY"; recovery.className = "badge status-warn";
  el("rebuild").textContent = rev.state === "BUILDING" ? "BUILDING" : (rev.state === "FAILED" ? "FAILED" : "");
  el("waiting").style.display = st.map && st.map.width > 0 ? "none" : "flex";
  maybeReloadMap(st.map, rev.map_revision);
  pushHistory(st);
  draw();
  renderEvents(st.events);
}

function setDisconnected() {
  const conn = el("conn");
  conn.textContent = "disconnected"; conn.className = "badge badge-disconnected";
}

// ── poll loop ──────────────────────────────────────────────────────────────────
async function poll() {
  try {
    const res = await fetch("/state", { cache: "no-store" });
    if (!res.ok) throw new Error("bad status " + res.status);
    applyState(await res.json());
  } catch (e) {
    setDisconnected();
  }
}

resizeAll();
poll();
setInterval(poll, 66); // ~15 Hz for smooth realtime pose windows
