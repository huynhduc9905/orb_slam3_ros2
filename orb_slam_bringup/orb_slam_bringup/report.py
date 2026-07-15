"""Static HTML report, acceptance-gate checker, and run comparator."""

from __future__ import annotations

import base64
import io
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import numpy as np
from PIL import Image, ImageDraw

# ---------------------------------------------------------------------------
# Acceptance thresholds (Task 6 relies on these constants)
# ---------------------------------------------------------------------------

ACCEPTANCE_THRESHOLDS: Dict[str, Any] = {
    "stereo_paired_min_ratio": 0.99,
    "expected_stereo_pairs": 6633,
    "camera_width": 848,
    "camera_height": 480,
    "baseline_m": 0.0501881428,
    "ok_ratio_min": 0.70,
    "min_loop_closures": 1,
    "traj_pos_tol_m": 0.02,  # 2 cm
    "traj_yaw_tol_rad": math.radians(1.0),  # 1 degree
    "cell_count_rel_tol": 0.01,  # 1%
    "baseline_tol": 1e-6,
}


def check_acceptance(metrics: Dict[str, Any]) -> Dict[str, Any]:
    """Evaluate metrics against acceptance gates. Returns pass + gate list."""
    T = ACCEPTANCE_THRESHOLDS
    gates: List[Dict[str, Any]] = []

    stereo = metrics.get("stereo") or {}
    tracking = metrics.get("tracking") or {}
    fallback = metrics.get("fallback") or {}
    loops = metrics.get("loops") or []
    revisions = metrics.get("map_revisions") or []
    final_map = metrics.get("final_map") or {}

    # Stereo paired ratio
    expected = int(stereo.get("expected_pairs", T["expected_stereo_pairs"]))
    paired = int(stereo.get("paired_count", 0))
    ratio = float(stereo.get("paired_ratio", paired / expected if expected else 0.0))
    gates.append(
        _gate(
            "stereo_paired_ratio",
            ratio >= T["stereo_paired_min_ratio"],
            f"paired {paired}/{expected} ({ratio:.4f})",
            f">= {T['stereo_paired_min_ratio']:.0%} of {T['expected_stereo_pairs']}",
        )
    )

    # Camera validation
    w = int(stereo.get("width", 0))
    h = int(stereo.get("height", 0))
    baseline = float(stereo.get("baseline_m", 0.0))
    cam_ok = (
        w == T["camera_width"]
        and h == T["camera_height"]
        and abs(baseline - T["baseline_m"]) <= T["baseline_tol"]
        and bool(stereo.get("camera_validated", False))
    )
    gates.append(
        _gate(
            "camera_validated",
            cam_ok,
            f"{w}x{h} baseline={baseline}",
            f"{T['camera_width']}x{T['camera_height']} baseline={T['baseline_m']}",
        )
    )

    # ORB initialized & no invalid poses / deadlock
    init_ok = bool(tracking.get("initialized", False))
    invalid_poses = int(tracking.get("invalid_poses", 0))
    deadlock = bool(tracking.get("deadlock", False))
    gates.append(
        _gate(
            "orb_initialized",
            init_ok and invalid_poses == 0 and not deadlock,
            f"init={init_ok} invalid_poses={invalid_poses} deadlock={deadlock}",
            "initialized, invalid_poses==0, no deadlock",
        )
    )

    # Tracking OK ratio
    ok_ratio = float(tracking.get("ok_ratio_after_init", 0.0))
    gates.append(
        _gate(
            "tracking_ok_ratio",
            ok_ratio >= T["ok_ratio_min"],
            f"{ok_ratio:.4f}",
            f">= {T['ok_ratio_min']}",
        )
    )

    # Loop closures
    loop_count = int(tracking.get("loop_count", len(loops)))
    gates.append(
        _gate(
            "loop_closures",
            loop_count >= T["min_loop_closures"],
            f"{loop_count}",
            f">= {T['min_loop_closures']}",
        )
    )

    # Every loop revision has a completed atomic map rebuild
    published_graph_revs = {
        int(r.get("graph_revision", -1))
        for r in revisions
        if str(r.get("state", "")).upper() == "PUBLISHED"
    }
    loops_missing = []
    for lp in loops:
        gr = int(lp.get("graph_revision", -1))
        if gr not in published_graph_revs:
            loops_missing.append(gr)
    gates.append(
        _gate(
            "loop_rebuild_complete",
            len(loops_missing) == 0,
            f"missing_graph_revs={loops_missing}",
            "every loop graph_revision has PUBLISHED map rebuild",
        )
    )

    # Invalid TF / trajectory committed (None / missing = unmeasured → FAIL closed)
    inv_tf_ok, inv_tf_actual = _zero_count_gate(
        fallback, "invalid_tf_committed"
    )
    gates.append(
        _gate(
            "invalid_tf_committed",
            inv_tf_ok,
            inv_tf_actual,
            "== 0 (measured)",
        )
    )

    # Wheel-only before recovery (None / missing = unmeasured → FAIL closed)
    wheel_ok, wheel_actual = _zero_count_gate(
        fallback, "wheel_only_before_recovery"
    )
    gates.append(
        _gate(
            "wheel_only_before_recovery",
            wheel_ok,
            wheel_actual,
            "== 0 (measured)",
        )
    )

    # Final free/occupied cells
    free = int(final_map.get("free_cells", 0))
    occ = int(final_map.get("occupied_cells", 0))
    gates.append(
        _gate(
            "final_free_cells",
            free > 0,
            f"{free}",
            "> 0",
        )
    )
    gates.append(
        _gate(
            "final_occupied_cells",
            occ > 0,
            f"{occ}",
            "> 0",
        )
    )

    # PGM/YAML match
    pgm_ok = bool(final_map.get("pgm_yaml_match", False))
    gates.append(
        _gate(
            "pgm_yaml_match",
            pgm_ok,
            str(pgm_ok),
            "final PGM/YAML reload matches OccupancyGrid",
        )
    )

    failed = [g for g in gates if not g["pass"]]
    return {
        "pass": len(failed) == 0,
        "gates": gates,
        "failed": failed,
    }


def _gate(name: str, passed: bool, actual: str, expected: str) -> Dict[str, Any]:
    return {
        "name": name,
        "pass": bool(passed),
        "actual": actual,
        "expected": expected,
    }


def _zero_count_gate(
    section: Dict[str, Any], key: str
) -> tuple[bool, str]:
    """Pass only when key is present, measured, and equal to 0.

    Missing key or explicit None / "unmeasured" fails closed (never invents 0).
    """
    if key not in section:
        return False, "unmeasured"
    raw = section[key]
    if raw is None:
        return False, "unmeasured"
    if isinstance(raw, str) and raw.strip().lower() in ("unmeasured", "unknown", ""):
        return False, "unmeasured"
    try:
        value = int(raw)
    except (TypeError, ValueError):
        return False, f"unmeasured ({raw!r})"
    return value == 0, str(value)


def compare_runs(a: Dict[str, Any], b: Dict[str, Any]) -> Dict[str, Any]:
    """Repeatability check between two metrics.json dicts."""
    T = ACCEPTANCE_THRESHOLDS
    mismatches: List[str] = []

    fa = a.get("final_map") or {}
    fb = b.get("final_map") or {}
    for key in ("width", "height"):
        if int(fa.get(key, -1)) != int(fb.get(key, -2)):
            mismatches.append(f"map dimension {key}: {fa.get(key)} vs {fb.get(key)}")
    if abs(float(fa.get("resolution", 0)) - float(fb.get("resolution", 0))) > 1e-9:
        mismatches.append(
            f"map resolution: {fa.get('resolution')} vs {fb.get('resolution')}"
        )
    oa = fa.get("origin") or [0, 0, 0]
    ob = fb.get("origin") or [0, 0, 0]
    for i, label in enumerate(("origin_x", "origin_y", "origin_yaw")):
        if abs(float(oa[i]) - float(ob[i])) > 1e-6:
            mismatches.append(f"map {label}: {oa[i]} vs {ob[i]}")

    # Ordered event types
    ea = a.get("event_types_ordered") or []
    eb = b.get("event_types_ordered") or []
    if ea != eb:
        mismatches.append(f"event_types_ordered mismatch: {ea} vs {eb}")

    # Map revision count (published)
    ra = [
        r
        for r in (a.get("map_revisions") or [])
        if str(r.get("state", "")).upper() == "PUBLISHED"
    ]
    rb = [
        r
        for r in (b.get("map_revisions") or [])
        if str(r.get("state", "")).upper() == "PUBLISHED"
    ]
    if len(ra) != len(rb):
        mismatches.append(f"map revision count: {len(ra)} vs {len(rb)}")

    # Cell counts within 1%
    for key in ("free_cells", "occupied_cells"):
        va = float(fa.get(key, 0))
        vb = float(fb.get(key, 0))
        base = max(abs(va), abs(vb), 1.0)
        if abs(va - vb) / base > T["cell_count_rel_tol"]:
            mismatches.append(f"{key} outside 1%: {va} vs {vb}")

    # Trajectories within 2 cm / 1 deg at matched timestamps
    for name in ("orb", "wheel", "corrected"):
        ta = ((a.get("trajectories") or {}).get(name)) or []
        tb = ((b.get("trajectories") or {}).get(name)) or []
        err = _trajectory_mismatch(ta, tb, T["traj_pos_tol_m"], T["traj_yaw_tol_rad"])
        if err:
            mismatches.append(f"trajectory {name}: {err}")

    return {"pass": len(mismatches) == 0, "mismatches": mismatches}


def _trajectory_mismatch(
    a: Sequence[Dict[str, float]],
    b: Sequence[Dict[str, float]],
    pos_tol: float,
    yaw_tol: float,
    *,
    time_tol_s: float = 0.05,
    min_match_fraction: float = 0.5,
) -> Optional[str]:
    """Compare trajectories at nearest timestamps.

    Fail-closed: if too few samples share a timestamp within ``time_tol_s``
    (non-overlapping time bases), return a mismatch instead of silent pass.
    """
    if not a and not b:
        return None
    if not a or not b:
        return f"length {len(a)} vs {len(b)}"
    smaller = min(len(a), len(b))
    min_matched = max(1, int(math.ceil(min_match_fraction * smaller)))
    matched = 0
    # Match by nearest timestamp
    for pa in a:
        t = float(pa.get("t", 0.0))
        # nearest in b
        pb = min(b, key=lambda q: abs(float(q.get("t", 0.0)) - t))
        if abs(float(pb.get("t", 0.0)) - t) > time_tol_s:
            continue  # no close timestamp
        matched += 1
        dx = float(pa.get("x", 0.0)) - float(pb.get("x", 0.0))
        dy = float(pa.get("y", 0.0)) - float(pb.get("y", 0.0))
        dist = math.hypot(dx, dy)
        if dist > pos_tol:
            return f"pos delta {dist:.4f}m at t={t}"
        dyaw = _angle_diff(float(pa.get("yaw", 0.0)), float(pb.get("yaw", 0.0)))
        if abs(dyaw) > yaw_tol:
            return f"yaw delta {math.degrees(dyaw):.3f}deg at t={t}"
    if matched < min_matched:
        return (
            f"too few matched samples: {matched}/{smaller} "
            f"(need >= {min_matched}, fraction {min_match_fraction:.0%})"
        )
    return None


def _angle_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def render_trajectory_overlay_png(
    *,
    orb: Sequence[Dict[str, float]],
    wheel: Sequence[Dict[str, float]],
    corrected: Sequence[Dict[str, float]],
    width: int = 640,
    height: int = 480,
) -> bytes:
    """Render trajectory overlay to PNG bytes using numpy+PIL (no matplotlib)."""
    img = Image.new("RGB", (width, height), (24, 24, 28))
    draw = ImageDraw.Draw(img)

    all_pts = list(orb) + list(wheel) + list(corrected)
    if not all_pts:
        draw.text((10, 10), "no trajectory", fill=(200, 200, 200))
        return _png_bytes(img)

    xs = [float(p.get("x", 0.0)) for p in all_pts]
    ys = [float(p.get("y", 0.0)) for p in all_pts]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max(max_x - min_x, 1e-3)
    span_y = max(max_y - min_y, 1e-3)
    pad = 20

    def to_px(x: float, y: float) -> tuple:
        px = pad + (x - min_x) / span_x * (width - 2 * pad)
        # y up
        py = height - pad - (y - min_y) / span_y * (height - 2 * pad)
        return (px, py)

    def draw_path(pts: Sequence[Dict[str, float]], color: tuple) -> None:
        if len(pts) < 2:
            if pts:
                p = to_px(float(pts[0].get("x", 0)), float(pts[0].get("y", 0)))
                r = 2
                draw.ellipse((p[0] - r, p[1] - r, p[0] + r, p[1] + r), fill=color)
            return
        coords = [
            to_px(float(p.get("x", 0)), float(p.get("y", 0))) for p in pts
        ]
        draw.line(coords, fill=color, width=2)

    draw_path(wheel, (120, 160, 255))  # blue-ish
    draw_path(orb, (255, 180, 80))  # orange
    draw_path(corrected, (80, 220, 120))  # green

    # Legend
    draw.text((10, 8), "wheel", fill=(120, 160, 255))
    draw.text((70, 8), "orb", fill=(255, 180, 80))
    draw.text((110, 8), "corrected", fill=(80, 220, 120))

    return _png_bytes(img)


def _png_bytes(img: Image.Image) -> bytes:
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return buf.getvalue()


def generate_report_html(
    metrics: Dict[str, Any],
    *,
    artifact_dir: Path | str | None = None,
) -> str:
    """Self-contained static HTML report (no network, embedded CSS/JS)."""
    artifact_dir = Path(artifact_dir) if artifact_dir else Path(".")
    acceptance = check_acceptance(metrics)
    tracking = metrics.get("tracking") or {}
    loops = metrics.get("loops") or []
    revisions = metrics.get("map_revisions") or []
    final_map = metrics.get("final_map") or {}
    stereo = metrics.get("stereo") or {}
    bag = metrics.get("bag") or {}
    git = metrics.get("git") or {}
    diagnostics = metrics.get("diagnostics") or []
    trajs = metrics.get("trajectories") or {}

    traj_png = render_trajectory_overlay_png(
        orb=trajs.get("orb") or [],
        wheel=trajs.get("wheel") or [],
        corrected=trajs.get("corrected") or [],
    )
    traj_b64 = base64.b64encode(traj_png).decode("ascii")

    # Embed available map-revision PNGs as base64 if present
    map_imgs_html = []
    for p in sorted(artifact_dir.glob("map-revision-*.png")):
        try:
            b64 = base64.b64encode(p.read_bytes()).decode("ascii")
            map_imgs_html.append(
                f'<figure><figcaption>{p.name}</figcaption>'
                f'<img alt="{p.name}" src="data:image/png;base64,{b64}"/></figure>'
            )
        except OSError:
            map_imgs_html.append(f"<p>missing {p.name}</p>")

    # Final map preview from PGM if present
    final_map_html = ""
    pgm = artifact_dir / "final-map.pgm"
    if pgm.is_file():
        try:
            final_png = _pgm_to_png_bytes(pgm)
            b64 = base64.b64encode(final_png).decode("ascii")
            final_map_html = (
                f'<img alt="final-map" src="data:image/png;base64,{b64}"/>'
            )
        except Exception:  # noqa: BLE001
            final_map_html = f"<p>final-map.pgm present ({pgm.stat().st_size} bytes)</p>"

    overall = "PASS" if acceptance["pass"] else "FAIL"
    gates_rows = "".join(
        f"<tr class=\"{'ok' if g['pass'] else 'bad'}\">"
        f"<td>{_esc(g['name'])}</td>"
        f"<td>{'PASS' if g['pass'] else 'FAIL'}</td>"
        f"<td>{_esc(str(g['actual']))}</td>"
        f"<td>{_esc(str(g['expected']))}</td></tr>"
        for g in acceptance["gates"]
    )
    loop_rows = "".join(
        f"<tr><td>{lp.get('t')}</td><td>{lp.get('graph_revision')}</td>"
        f"<td>{_esc(str(lp.get('detail', '')))}</td></tr>"
        for lp in loops
    ) or "<tr><td colspan='3'>none</td></tr>"
    rev_rows = "".join(
        f"<tr><td>{r.get('t')}</td><td>{_esc(str(r.get('state')))}</td>"
        f"<td>{r.get('graph_revision')}</td><td>{r.get('map_revision')}</td>"
        f"<td>{r.get('duration_ms')}</td>"
        f"<td>{r.get('committed_scan_count')}/{r.get('input_scan_count')}</td></tr>"
        for r in revisions
    ) or "<tr><td colspan='6'>none</td></tr>"

    loss = tracking.get("loss_intervals") or []
    loss_rows = "".join(
        f"<tr><td>{li.get('start_s')}</td><td>{li.get('end_s')}</td>"
        f"<td>{li.get('duration_s')}</td></tr>"
        for li in loss
    ) or "<tr><td colspan='3'>none</td></tr>"

    diag_rows = "".join(
        f"<tr><td>{_esc(str(d.get('name', '')))}</td>"
        f"<td>{d.get('level')}</td>"
        f"<td>{_esc(str(d.get('message', '')))}</td></tr>"
        for d in diagnostics[:50]
    ) or "<tr><td colspan='3'>none</td></tr>"

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<title>ORB-SLAM3 Lidar Run Report — {overall}</title>
<style>
:root {{ --bg:#0f1115; --fg:#e6e6e6; --muted:#9aa0a6; --ok:#3ddc97; --bad:#ff6b6b;
  --card:#1a1d24; --border:#2a2f3a; --accent:#6cb6ff; }}
* {{ box-sizing:border-box; }}
body {{ margin:0; font-family: ui-sans-serif, system-ui, sans-serif; background:var(--bg);
  color:var(--fg); line-height:1.45; padding:1.5rem; }}
h1,h2,h3 {{ margin:1.2rem 0 0.5rem; }}
h1 {{ font-size:1.5rem; }}
.badge {{ display:inline-block; padding:0.2rem 0.6rem; border-radius:4px; font-weight:700; }}
.badge.pass {{ background:var(--ok); color:#042; }}
.badge.fail {{ background:var(--bad); color:#200; }}
.card {{ background:var(--card); border:1px solid var(--border); border-radius:8px;
  padding:1rem; margin:0.8rem 0; }}
table {{ width:100%; border-collapse:collapse; font-size:0.9rem; }}
th,td {{ text-align:left; padding:0.35rem 0.5rem; border-bottom:1px solid var(--border); }}
th {{ color:var(--muted); font-weight:600; }}
tr.ok td:nth-child(2) {{ color:var(--ok); }}
tr.bad td:nth-child(2) {{ color:var(--bad); }}
img {{ max-width:100%; height:auto; border:1px solid var(--border); border-radius:4px; }}
figure {{ display:inline-block; margin:0.5rem; max-width:320px; }}
figcaption {{ color:var(--muted); font-size:0.8rem; }}
a {{ color:var(--accent); }}
.meta {{ color:var(--muted); font-size:0.85rem; }}
.grid {{ display:grid; grid-template-columns:1fr 1fr; gap:1rem; }}
@media (max-width:800px) {{ .grid {{ grid-template-columns:1fr; }} }}
ul.links {{ list-style:none; padding:0; }}
ul.links li {{ margin:0.25rem 0; }}
code {{ background:#0a0c10; padding:0.1rem 0.3rem; border-radius:3px; }}
</style>
</head>
<body>
<h1>ORB-SLAM3 Lidar Run Report
  <span class="badge {'pass' if acceptance['pass'] else 'fail'}">{overall}</span>
</h1>
<p class="meta">Generated offline — no network requests. Open this file directly in a browser.</p>

<div class="card">
<h2>Run configuration</h2>
<table>
<tr><th>Bag path</th><td>{_esc(str(bag.get('path', '')))}</td></tr>
<tr><th>Bag duration (s)</th><td>{bag.get('duration_s', '')}</td></tr>
<tr><th>Git commit</th><td><code>{_esc(str(git.get('commit', '')))}</code>
  {'(dirty)' if git.get('dirty') else ''}</td></tr>
<tr><th>Configuration SHA-256</th><td><code>{_esc(str(metrics.get('configuration_sha256', '')))}</code></td></tr>
<tr><th>Camera</th><td>{stereo.get('width')}x{stereo.get('height')}
  baseline={stereo.get('baseline_m')}</td></tr>
<tr><th>Stereo pairs</th><td>{stereo.get('paired_count')}/{stereo.get('expected_pairs')}
  ({float(stereo.get('paired_ratio', 0)):.4f})</td></tr>
</table>
</div>

<div class="card">
<h2>Acceptance gates</h2>
<table>
<thead><tr><th>Gate</th><th>Result</th><th>Actual</th><th>Expected</th></tr></thead>
<tbody>{gates_rows}</tbody>
</table>
</div>

<div class="grid">
<div class="card">
<h2>Tracking timeline</h2>
<table>
<tr><th>Initialized</th><td>{tracking.get('initialized')}</td></tr>
<tr><th>Init time (s)</th><td>{tracking.get('init_time_s')}</td></tr>
<tr><th>OK ratio after init</th><td>{tracking.get('ok_ratio_after_init')}</td></tr>
<tr><th>OK duration (s)</th><td>{tracking.get('ok_duration_s')}</td></tr>
<tr><th>Post-init duration (s)</th><td>{tracking.get('post_init_duration_s')}</td></tr>
<tr><th>Loop count</th><td>{tracking.get('loop_count')}</td></tr>
<tr><th>Mean rebuild (ms)</th><td>{tracking.get('mean_rebuild_ms')}</td></tr>
<tr><th>Max rebuild (ms)</th><td>{tracking.get('max_rebuild_ms')}</td></tr>
</table>
<h3>Loss intervals</h3>
<table>
<thead><tr><th>Start</th><th>End</th><th>Duration</th></tr></thead>
<tbody>{loss_rows}</tbody>
</table>
</div>

<div class="card">
<h2>Trajectory overlay</h2>
<img alt="trajectory overlay" src="data:image/png;base64,{traj_b64}"/>
</div>
</div>

<div class="card">
<h2>Loops</h2>
<table>
<thead><tr><th>t</th><th>graph_revision</th><th>detail</th></tr></thead>
<tbody>{loop_rows}</tbody>
</table>
</div>

<div class="card">
<h2>Map rebuilds</h2>
<table>
<thead><tr><th>t</th><th>state</th><th>graph_rev</th><th>map_rev</th>
<th>duration_ms</th><th>committed/input</th></tr></thead>
<tbody>{rev_rows}</tbody>
</table>
</div>

<div class="card">
<h2>Map revisions (before/after previews)</h2>
{''.join(map_imgs_html) if map_imgs_html else '<p class="meta">No map-revision-N.png files found.</p>'}
</div>

<div class="card">
<h2>Final map</h2>
<table>
<tr><th>Size</th><td>{final_map.get('width')} x {final_map.get('height')}</td></tr>
<tr><th>Resolution</th><td>{final_map.get('resolution')}</td></tr>
<tr><th>Origin</th><td>{final_map.get('origin')}</td></tr>
<tr><th>Free / Occupied / Unknown</th>
<td>{final_map.get('free_cells')} / {final_map.get('occupied_cells')} / {final_map.get('unknown_cells')}</td></tr>
<tr><th>PGM/YAML match</th><td>{final_map.get('pgm_yaml_match')}</td></tr>
</table>
{final_map_html}
</div>

<div class="card">
<h2>Diagnostics</h2>
<table>
<thead><tr><th>Name</th><th>Level</th><th>Message</th></tr></thead>
<tbody>{diag_rows}</tbody>
</table>
</div>

<div class="card">
<h2>Raw artifacts</h2>
<ul class="links">
<li><a href="metrics.json">metrics.json</a></li>
<li><a href="events.jsonl">events.jsonl</a></li>
<li><a href="orb_trajectory.csv">orb_trajectory.csv</a></li>
<li><a href="wheel_trajectory.csv">wheel_trajectory.csv</a></li>
<li><a href="corrected_trajectory.csv">corrected_trajectory.csv</a></li>
<li><a href="final-map.pgm">final-map.pgm</a></li>
<li><a href="final-map.yaml">final-map.yaml</a></li>
</ul>
</div>

<script>
/* Local-only helper: toggle FAIL rows emphasis (no network). */
(function() {{
  document.querySelectorAll('tr.bad').forEach(function(tr) {{
    tr.title = 'Acceptance gate failed';
  }});
}})();
</script>
</body>
</html>
"""


def _esc(s: str) -> str:
    return (
        s.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def _pgm_to_png_bytes(pgm_path: Path) -> bytes:
    raw = pgm_path.read_bytes()
    if not raw.startswith(b"P5"):
        raise ValueError("not a P5 PGM")
    idx = raw.index(b"\n") + 1
    while raw[idx : idx + 1] == b"#":
        idx = raw.index(b"\n", idx) + 1
    end = raw.index(b"\n", idx)
    parts = raw[idx:end].decode("ascii").split()
    if len(parts) >= 2:
        w, h = int(parts[0]), int(parts[1])
        idx = end + 1
    else:
        raise ValueError("bad PGM header")
    # maxval
    end = raw.index(b"\n", idx)
    maxval = int(raw[idx:end].decode("ascii").strip())
    idx = end + 1
    pixels = np.frombuffer(raw[idx : idx + w * h], dtype=np.uint8).reshape((h, w))
    # Scale if needed
    if maxval != 255:
        pixels = (pixels.astype(np.float32) * (255.0 / maxval)).astype(np.uint8)
    return _png_bytes(Image.fromarray(pixels, mode="L"))


def check_main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI: orb_slam_report_check <metrics.json> — exit 0 iff all gates pass."""
    args = list(argv if argv is not None else sys.argv)
    # Drop program name if present
    if args and (args[0].endswith("orb_slam_report_check") or args[0].endswith("check")
                 or args[0] == "check" or "report" in Path(args[0]).name):
        # keep if second is path; strip first when it looks like prog name
        if len(args) >= 2 and not args[0].endswith(".json"):
            args = args[1:]
    if not args:
        print("usage: orb_slam_report_check <metrics.json>", file=sys.stderr)
        return 2
    path = Path(args[0])
    if not path.is_file():
        print(f"metrics file not found: {path}", file=sys.stderr)
        return 2
    try:
        metrics = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"failed to load metrics: {exc}", file=sys.stderr)
        return 2
    result = check_acceptance(metrics)
    for g in result["gates"]:
        status = "PASS" if g["pass"] else "FAIL"
        print(f"[{status}] {g['name']}: {g['actual']} (expected {g['expected']})")
    if result["pass"]:
        print("ALL GATES PASSED")
        return 0
    print(f"FAILED {len(result['failed'])} gate(s)", file=sys.stderr)
    return 1


def compare_main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI: orb_slam_compare_runs <a.json> <b.json> — exit 0 iff repeatable."""
    args = list(argv if argv is not None else sys.argv)
    if args and not args[0].endswith(".json"):
        # strip program name / subcommand
        if args[0] in ("compare",) or "compare" in Path(args[0]).name or args[0].endswith(
            "orb_slam_compare_runs"
        ):
            args = args[1:]
    if len(args) < 2:
        print("usage: orb_slam_compare_runs <metrics_a.json> <metrics_b.json>", file=sys.stderr)
        return 2
    paths = [Path(args[0]), Path(args[1])]
    metrics_list = []
    for p in paths:
        if not p.is_file():
            print(f"metrics file not found: {p}", file=sys.stderr)
            return 2
        try:
            metrics_list.append(json.loads(p.read_text(encoding="utf-8")))
        except (OSError, json.JSONDecodeError) as exc:
            print(f"failed to load {p}: {exc}", file=sys.stderr)
            return 2
    result = compare_runs(metrics_list[0], metrics_list[1])
    if result["pass"]:
        print("RUNS MATCH within tolerances")
        return 0
    for m in result["mismatches"]:
        print(f"MISMATCH: {m}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    # Default to check_main when invoked as module with a path.
    if len(sys.argv) >= 2 and sys.argv[1] in ("check", "compare"):
        cmd = sys.argv[1]
        rest = sys.argv[2:]
        if cmd == "check":
            sys.exit(check_main(["check"] + rest))
        sys.exit(compare_main(["compare"] + rest))
    sys.exit(check_main(sys.argv))
