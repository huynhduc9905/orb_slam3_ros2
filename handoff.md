# ORB-SLAM3 ROS 2 Handoff

## Current State

- Workspace: `/home/duc/orb_slam3_ros2`
- Branch: `feature/orb-lidar-thin-walls-p0-p1`
- Latest main commit: `2162803 feat: add read-only keyframe drift diagnostic`
- Vendor submodule `orb_slam3_vendor/vendor/ORB_SLAM3` HEAD (detached, by design):
  `d8fbdd9 fix: allow distant covisible keyframes as loop candidates via id gap`
- The multi-lap loop-closure "double wall" problem is **fixed and validated**
  on the circle bag. See "Loop Revisit Fix" below.

### Uncommitted / untracked (intentional, do not blindly clean)

- `git status --short` shows `M orb_slam3_vendor/vendor/ORB_SLAM3` — the
  **submodule pointer bump is NOT yet committed in the main repo**. The fork
  commits (`848b3de`, `d8fbdd9`) live inside the submodule, but the parent repo
  still records the old pointer `601bec2`. A fresh clone would not pick up the
  fix until this pointer is committed (and the fork pushed to its remote
  `huynhduc9905/ORB_SLAM3`). This is the one remaining finalize step — see
  "Next Steps".
- Untracked plan docs from earlier finished work, left as-is:
  `docs/superpowers/plans/2026-07-20-loop-evidence-map-revisions.md`,
  `docs/superpowers/plans/2026-07-20-wrapper-log-teardown.md`.
- Generated `artifacts/` directories are not to be committed.

## Loop Revisit Fix (primary recent work)

### Problem

On multi-lap bags (the robot re-drives the same physical loop), ORB-SLAM3 fired
loop closure exactly once near the end of lap 1, then never again. Lap 2's
accumulated drift was never corrected, so the lidar mapper drew every wall
twice (~0.3 m apart): the "double wall". Measured lap-1 vs lap-2 ORB pose
difference at the same physical location: mean 0.29 m, max 0.61 m.

### Confirmed root cause (verified in source)

Loop-candidate selection excluded **any** covisibility-connected keyframe, with
no temporal component. On lap 2 the robot re-observes lap-1 features, creating
covisibility edges to the lap-1 keyframes, so they were excluded from ever
becoming loop candidates. Two gates enforced this:

- `orb_slam3_vendor/vendor/ORB_SLAM3/src/KeyFrameDatabase.cc`
  `DetectNBestCandidates`: `if(!spConnectedKF.count(pKFi))`
- `orb_slam3_vendor/vendor/ORB_SLAM3/src/LoopClosing.cc`
  `DetectCommonRegionsFromBoW`: `bAbortByNearKF` loop

### Fix

A pure, header-only predicate gates both exclusions on a keyframe-ID gap, so
only *recent* trajectory neighbors are excluded; distant connected keyframes
(genuine revisits) flow through to the unchanged Sim3 / `>=3` coincidence
verification.

- `orb_slam3_vendor/vendor/ORB_SLAM3/include/LoopRevisitPolicy.h`
  - `namespace orb_slam3_wrapper_fork`
  - `constexpr unsigned long kLoopRevisitMinKFGap = 20;`
  - `bool shouldExcludeConnectedLoopCandidate(current_id, candidate_id)` —
    returns true (exclude) only when
    `current_id >= candidate_id && current_id - candidate_id < 20`; false
    otherwise (guards unsigned underflow for future/equal ids).
- Both gates call the predicate: exclude iff (connected AND recent).
- Merge detection, the `>=3` coincidence gate, mapper, metrics, replay runner,
  and camera/ORB params are all unchanged.
- The `20` constant is hardcoded (user decision) and tuned to the circle bag's
  keyframe density; may need adjustment for very different robots/rates.

Fork commits:

- `848b3de feat: add keyframe-id-gap loop revisit policy helper`
- `d8fbdd9 fix: allow distant covisible keyframes as loop candidates via id gap`

Main-repo commits (tests + wiring + diagnostic):

- `bbaaf64 test: cover loop revisit id-gap policy`
- `2162803 feat: add read-only keyframe drift diagnostic`

Design + plan:

- `docs/superpowers/specs/2026-07-20-loop-revisit-id-gap-design.md`
- `docs/superpowers/plans/2026-07-20-loop-revisit-id-gap.md`

### Validation evidence (circle bag, rate 1, benchmark_mode off)

Artifacts: `artifacts/circle-loop-revisit-idgap-20260720/`

- Graph revision advanced 1 -> **33** (baseline stopped at 3).
- **25 loop edges**, all `same_map_loop`, firing throughout lap 2.
- **21 mapper full rebuilds** during lap 2.
- Corrected trajectory now spans the **full run (bag+0..85s)** vs baseline's
  bag+42s.
- ORB lap-1-vs-lap-2 pose difference mean dropped **0.290 -> 0.212 m** (max
  0.607 -> 0.601; a single worst-point sample, expected — global drift is
  redistributed as soft constraints).
- No `MAP_MERGED` / `MAP_RESET` / `cross_map_loop`. Corrected-path "jumps >1 m"
  were all across 4-19 s sparse-sampling gaps at 0.16-0.36 m/s (below the
  robot's 0.38 m/s median), i.e. artifacts, not teleports.

### Reviews

Implemented via subagent-driven development. Each task independently reviewed
(all clean). Final whole-branch review: **Ready to merge = Yes**, only Minor
item was the then-uncommitted drift diagnostic (now committed as `2162803`).
Progress ledger: `.superpowers/sdd/progress.md`.

## Read-Only Keyframe Drift Diagnostic (`2162803`)

Param-gated (`log_keyframe_drift`, default **false**), read-only. When enabled,
a 500 ms wall timer logs how far ORB-SLAM3 moves keyframe camera poses between
graph polls, independent of `MapChanged()`. This is a **console/ROS log line
only** (`RCLCPP_INFO`, `keyframe_drift ...`); it is NOT rendered in the web
dashboard. Inert when disabled. Files:

- `orb_slam3_wrapper/include/orb_slam3_wrapper/wrapper_node.hpp`
- `orb_slam3_wrapper/src/wrapper_node.cpp` (`logKeyframeDrift`, `drift_timer_`)

To enable for a run, add `-p log_keyframe_drift:=true` to the wrapper launch
args (or temporarily to `bag_replay.launch.py`'s `wrapper_cmd_parts`, then
revert). The launch file is currently clean of this flag.

## Running the Stack

All builds/tests/replays run inside the nix devshell:

```bash
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; <cmd>'
```

Full stack + LAN dashboard (detach so it survives the shell):

```bash
nohup setsid nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; \
  exec ros2 launch orb_slam_bringup bag_replay.launch.py \
  bag_path:=/home/duc/bag/full-run \
  artifact_dir:=artifacts/<name> rate:=1 ros_domain_id:=<id> \
  start_dashboard:=true dashboard_host:=0.0.0.0 benchmark_mode:=off' \
  > artifacts/<name>.launch.log 2>&1 < /dev/null & disown
```

- Live dashboard: `http://<LAN-IP>:51871/` (bind `0.0.0.0` for LAN). This
  machine's LAN IP is `192.168.100.112`. The launch log prints
  `[dashboard] open http://127.0.0.1:51871/` regardless; LAN binding is
  confirmed via `ss -ltnp | grep 51871` showing `0.0.0.0:51871`.
- `benchmark_mode:=full_stack` **disables** the dashboard (logs
  "benchmark modes do not start the dashboard"). Use `benchmark_mode:=off`
  when you want the dashboard.
- The dashboard web UI (React/PixiJS) shows: map, trajectories/paths, tracking
  image, Events panel (loop closures, tracking events), Health panel
  (`/diagnostics`). It is read-only and in-graph (no Foxglove bridge).
- The stack shuts down cleanly when the bag finishes.

### Static per-run report (map versions, odometry, trajectory, etc.)

Separate from the live dashboard, each run writes a self-contained
`report.html` to its artifact dir, generated by
`orb_slam_bringup/orb_slam_bringup/report.py`. Sections: Run configuration,
Acceptance gates, Tracking timeline, Trajectory overlay (orb/wheel/corrected),
Loops, Map rebuilds, Map revisions (before/after previews), Final map,
Diagnostics, Raw artifacts. Finalized at run shutdown.

To view reports over LAN, serve the artifacts dir:

```bash
cd artifacts && nohup setsid python3 -m http.server 8090 --bind 0.0.0.0 \
  > /tmp/opencode/report_http.log 2>&1 < /dev/null & disown
# then browse http://192.168.100.112:8090/<run>/report.html
```

NOTE: a plain `python3 -m http.server` on `:8090` was left running to serve
`artifacts/` (no auth, read-only, LAN). Stop it when done
(`pkill -f 'http.server 8090'`).

## Bags

Under `/home/duc/bag/` (NOT `/home/duc/robot/bag/`):

- `circle-run` (~90 s) — two laps of the same loop; the double-wall test case.
- `full-run` (~187 s) — larger trajectory.
- Others: `20260713_152907`, `forward-and-back-origin`, `inplace-rotate*`.

Recent replay artifacts:

- `artifacts/circle-loop-revisit-idgap-20260720/` — primary validation run
  (complete report).
- `artifacts/full-run-loop-revisit-dashboard-20260721/` — full-run with the fix
  (12 loop closures, graph revision to 14; report at
  `.../report.html`).

## Prior Completed Work (context)

Earlier on this branch, before the loop-revisit fix, several wrapper/bringup
issues were resolved and reviewed clean (see `.superpowers/sdd/progress.md`):

- Graph baseline capture, graph session reset on reconfiguration, and graph
  observation diagnostics (`2675fd8`..`cec5aee`). These made loop-edge deltas
  observable to the wrapper — the earlier "circle loop-closure acceptance"
  failure described in older handoffs is resolved.
- Loop-closure evidence evaluator reads `metrics["map_revisions"]` and requires
  a `PUBLISHED` rebuild at/after the loop graph revision
  (`2bc0399`, `c8043f7`); `orb_slam_bringup/orb_slam_bringup/loop_closure_evidence.py`,
  runner `tools/run_circle_loop_closure_evaluation.sh`.
- Wrapper logging teardown: bag replay now uses
  `exec ros2 run ... > >(tee -a <log>) 2>&1` so shutdown signals reach the
  wrapper and no `ros2 run`/`tee`/wrapper processes leak (`a073d2d`, `1706a71`).

## Next Steps

1. **Commit the submodule pointer bump** in the main repo so the fork fix is
   actually referenced by the parent, ideally after pushing the fork:
   ```bash
   git -C orb_slam3_vendor/vendor/ORB_SLAM3 push origin HEAD   # push 848b3de/d8fbdd9 to the fork remote
   git add orb_slam3_vendor/vendor/ORB_SLAM3
   git commit -m "chore: bump ORB_SLAM3 submodule to loop-revisit id-gap fix"
   ```
   (Not done automatically — it publishes the fork pointer and depends on the
   fork remote having the commits.)
2. Optionally validate the fix on `full-run` via the evaluator / report and on
   any other multi-lap bags.
3. If deploying to robots with very different keyframe density, revisit
   `kLoopRevisitMinKFGap` (currently hardcoded 20).

## Important Files

- `orb_slam3_vendor/vendor/ORB_SLAM3/include/LoopRevisitPolicy.h`
- `orb_slam3_vendor/vendor/ORB_SLAM3/src/KeyFrameDatabase.cc`
- `orb_slam3_vendor/vendor/ORB_SLAM3/src/LoopClosing.cc`
- `orb_slam3_vendor/test/loop_revisit_policy_test.cpp`
- `orb_slam3_wrapper/src/wrapper_node.cpp` (`logKeyframeDrift`, `pollGraphChanges`)
- `orb_slam3_wrapper/src/graph_semantics.cpp`
- `orb_slam_bringup/launch/bag_replay.launch.py`
- `orb_slam_bringup/orb_slam_bringup/report.py`
- `orb_slam_bringup/orb_slam_bringup/loop_closure_evidence.py`
- `tools/run_circle_loop_closure_evaluation.sh`
- `docs/superpowers/specs/2026-07-20-loop-revisit-id-gap-design.md`
- `docs/superpowers/plans/2026-07-20-loop-revisit-id-gap.md`
- `.superpowers/sdd/progress.md`

## Build / Vendor Notes

- The vendor package builds ORB_SLAM3 by `git clone`-ing the submodule's
  **committed** state at CMake configure time, then applying
  `orb_slam3_vendor/patches/*.patch`. Consequences:
  - Edits to the submodule source only take effect after committing **inside
    the submodule**.
  - A clean vendor rebuild requires
    `rm -rf build/orb_slam3_vendor install/orb_slam3_vendor` first; otherwise
    CMake reuses the cached clone. A sub-second vendor "build" means nothing
    recompiled — an ORB_SLAM3 recompile takes minutes.
  - Verify a source change reached the build via
    `grep -n <token> build/orb_slam3_vendor/ORB_SLAM3/src/<file>`.
- The submodule is intentionally on a detached HEAD; commit there without
  creating/switching branches unless asked.

## Workspace Hygiene

Do not delete/modify/stage unrelated untracked scratch files or generated
`artifacts/` directories. The intended uncommitted items are the submodule
pointer (pending step 1 above) and the two untracked plan docs listed under
"Current State".
