// Minimal custom dashboard: polls /state (10 Hz) and reloads /map.png on new
// map revision. No foxglove, no build step. Read-only.
"use strict";

const canvas = document.getElementById("map");
const ctx = canvas.getContext("2d");
const el = (id) => document.getElementById(id);

let mapImg = null;          // loaded <img> of /map.png
let mapMeta = null;         // {resolution, origin_x, origin_y, width, height, revision}
let requestedRevision = -1;
let loadedRevision = -1;
let mapRequestToken = 0;
let lastState = null;

// ── canvas sizing ────────────────────────────────────────────────────────────
function resizeCanvas() {
  const r = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.round(r.width * dpr));
  canvas.height = Math.max(1, Math.round(r.height * dpr));
  draw();
}
window.addEventListener("resize", resizeCanvas);

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
    draw();
  };
  img.onerror = () => {
    if (token === mapRequestToken && requestedRevision === revision) {
      requestedRevision = -1;
    }
  };
  img.src = "/map.png?rev=" + revision;
}

// ── world <-> screen transform ───────────────────────────────────────────────
// The PNG is already flipped so its row 0 is world +y (top). We fit the map's
// world AABB into the canvas with letterboxing; overlays share the transform.
function computeView() {
  if (!mapMeta || mapMeta.width <= 0) return null;
  const res = mapMeta.resolution;
  const worldW = mapMeta.width * res;
  const worldH = mapMeta.height * res;
  const pad = 0.95;
  const scale = Math.min(canvas.width / worldW, canvas.height / worldH) * pad;
  const drawW = worldW * scale;
  const drawH = worldH * scale;
  const offX = (canvas.width - drawW) / 2;
  const offY = (canvas.height - drawH) / 2;
  return { res, worldW, worldH, scale, drawW, drawH, offX, offY };
}

function worldToScreen(v, x, y) {
  // x grows right; y grows up in world -> down on screen.
  const sx = v.offX + (x - mapMeta.origin_x) * v.scale;
  const sy = v.offY + (v.worldH - (y - mapMeta.origin_y)) * v.scale;
  return [sx, sy];
}

// ── draw ──────────────────────────────────────────────────────────────────────
function draw() {
  ctx.fillStyle = "#0b0f14";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  const v = computeView();
  if (!v || !mapImg) return;

  // Map raster (PNG already y-up oriented) into the letterboxed rect.
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(mapImg, v.offX, v.offY, v.drawW, v.drawH);

  const st = lastState;
  if (!st) return;

  function drawLine(points, color) {
    if (!points || points.length < 2) return;
    ctx.strokeStyle = color; ctx.lineWidth = Math.max(2, 0.03 * v.scale);
    ctx.beginPath();
    points.forEach((point, i) => {
      const p = Array.isArray(point) ? { x: point[0], y: point[1] } : point;
      const [sx, sy] = worldToScreen(v, p.x, p.y);
      i === 0 ? ctx.moveTo(sx, sy) : ctx.lineTo(sx, sy);
    });
    ctx.stroke();
  }

  // Layer order is part of the direct dashboard contract.
  drawLine((st.paths && st.paths.corrected) || [], "#38bdf8");
  drawLine((st.fallback && st.fallback.trail) || [], "#facc15");
  const provisional = (st.paths && st.paths.provisional) || [];
  ctx.fillStyle = "#facc15";
  provisional.forEach((point) => {
    const [sx, sy] = worldToScreen(v, point[0], point[1]);
    ctx.beginPath(); ctx.arc(sx, sy, Math.max(2, 0.06 * v.scale), 0, Math.PI * 2); ctx.fill();
  });

  // The legacy path is retained as a compatibility fallback for old state fixtures.
  const path = st.path || [];
  if (path.length > 1 && !(st.paths && st.paths.corrected)) {
    ctx.strokeStyle = "#f59e0b";
    ctx.lineWidth = Math.max(1, 0.03 * v.scale);
    ctx.beginPath();
    for (let i = 0; i < path.length; i++) {
      const [sx, sy] = worldToScreen(v, path[i][0], path[i][1]);
      i === 0 ? ctx.moveTo(sx, sy) : ctx.lineTo(sx, sy);
    }
    ctx.stroke();
  }

  // Robot pose marker.
  const p = st.tracking && st.tracking.pose;
  if (p) {
    const [sx, sy] = worldToScreen(v, p.x, p.y);
    const r = Math.max(4, 0.12 * v.scale);
    ctx.fillStyle = "#22c55e";
    ctx.beginPath();
    ctx.arc(sx, sy, r, 0, Math.PI * 2);
    ctx.fill();
    // heading (yaw): world y-up -> screen y-down, so negate sin.
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = Math.max(1, 0.04 * v.scale);
    ctx.beginPath();
    ctx.moveTo(sx, sy);
    ctx.lineTo(sx + Math.cos(p.yaw) * r * 2, sy - Math.sin(p.yaw) * r * 2);
    ctx.stroke();
  }
}

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
    type.className = "type";
    type.textContent = e.type;
    li.appendChild(type);
    if (e.detail) {
      const d = document.createElement("span");
      d.className = "detail";
      d.textContent = " — " + e.detail;
      li.appendChild(d);
    }
    ul.appendChild(li);
  }
}

function applyState(st) {
  lastState = st;
  const conn = el("conn");
  conn.textContent = "connected";
  conn.className = "badge badge-connected";

  const t = st.tracking || {};
  const trackBadge = el("track");
  trackBadge.textContent = t.state || "--";
  trackBadge.className = "badge " + trackClass(t.state);
  el("trackHz").textContent = (t.hz ?? 0).toFixed(1);
  el("odomHz").textContent = (st.odom && st.odom.hz != null ? st.odom.hz : 0).toFixed(1);
  el("kp").textContent = t.keypoints || 0;
  const rev = st.map_revision || {};
  el("graphRev").textContent = rev.graph_revision ?? 0;
  el("mapRev").textContent = rev.map_revision ?? (st.map && st.map.revision) ?? 0;
  const recovery = el("recovery");
  recovery.hidden = st.state !== "recovery_pending";
  recovery.textContent = "RECOVERY";
  recovery.className = "badge status-warn";
  el("rebuild").textContent = rev.state === "BUILDING" ? "BUILDING" : (rev.state === "FAILED" ? "FAILED" : "");

  el("waiting").style.display = st.map && st.map.width > 0 ? "none" : "flex";

  maybeReloadMap(st.map, rev.map_revision);
  draw();
  renderEvents(st.events);
}

function setDisconnected() {
  const conn = el("conn");
  conn.textContent = "disconnected";
  conn.className = "badge badge-disconnected";
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

resizeCanvas();
poll();
setInterval(poll, 100); // 10 Hz
