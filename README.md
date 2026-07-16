# orb_slam3_ros2

Headless official ORB-SLAM3 integration, corrected 2D lidar mapping, and a
read-only ROS 2 dashboard.

The initial robot profile targets tasteRobot2. Core packages remain topic- and
frame-parameterized.

## Packages

| Package | Role |
|---|---|
| `orb_slam3_vendor` | Pinned upstream ORB-SLAM3 (headless) |
| `orb_slam3_msgs` | Custom messages (TrackedFrame, GraphSnapshot, MapRevision, …) |
| `orb_slam3_wrapper` | ROS 2 wrapper around ORB-SLAM3 stereo tracking |
| `orb_lidar_mapper` | ORB-corrected 2D lidar occupancy mapper |
| `orb_slam_bringup` | Bag replay, trusted TF audit, metrics, report, direct dashboard launch |
| `orb_slam_dashboard` | Read-only React/PixiJS map-first dashboard |

## Build (Nix + colcon)

From the robot meta-repo, enter the Nix shell then build the independent
workspace:

```bash
cd ~/robot
nix develop
cd src/orb_slam3_ros2

# Cap parallelism on machines with limited RAM (OpenVDB/PCL are heavy)
export MAKEFLAGS="-j4"
export CMAKE_BUILD_PARALLEL_LEVEL=4

COLCON_DEFAULTS_FILE=/dev/null colcon build \
  --packages-select orb_slam3_vendor orb_slam3_msgs orb_slam3_wrapper \
                    orb_lidar_mapper orb_slam_dashboard orb_slam_bringup \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
  --parallel-workers 1
source install/setup.bash
```

Run the package suites:

```bash
COLCON_DEFAULTS_FILE=/dev/null colcon test \
  --packages-select orb_slam3_vendor orb_slam3_msgs orb_slam3_wrapper \
                    orb_lidar_mapper orb_slam_dashboard orb_slam_bringup
colcon test-result --verbose
```

Current full-suite result: **321 tests, 0 errors, 0 failures, 0 skipped**.

## Offline lidar rotation-center check

The read-only calibration tool compares odometry-deskewed `/scan_origin`,
IMU-deskewed `/scan_origin`, and the existing `/scan` output without playing
the bag or changing runtime state:

```bash
source install/setup.bash
ros2 run orb_lidar_mapper lidar_rotation_center_check \
  --bag /home/duc/robot/bag/inplace-rotate \
  --output "$PWD/artifacts/inplace-rotate-calibration-20260716-retry1"
```

The immutable MCAP fixture is 91.353939991 seconds long and contains 915 raw
scans, 915 existing undistorted scans, 1,827 odometry messages, and 18,289 raw
IMU messages. The first attempt at
`artifacts/inplace-rotate-calibration-20260716` failed operationally because
the bag contains contiguous duplicate IMU header stamps. The approved reader
behavior now validates every finite angular-rate sample, averages only values
sharing an equal contiguous stamp, and continues to reject decreasing stamps.
That produced 8,569 unique IMU timestamps on the single fresh retry above.

The retry used no threshold overrides. The predeclared defaults remained
unchanged, including `0.15 <= |omega| <= 0.45 rad/s`, maximum linear speed
`0.02 m/s`, 10–30 degree pair separation, minimum overlap 0.40, maximum ICP
RMSE 0.05 m, and maximum ICP/odom yaw disagreement 2 degrees. It returned
scientific exit 3 and classification `INCONCLUSIVE`:

| Method | Center x/y (m) | 95% CI x (m) | Accepted/attempted | RMSE (m) | Overlap | Rejections |
|---|---|---|---:|---:|---:|---|
| Odom | 0.000 / 0.000 | [0.000, 0.000] | 0/512 | 0.000 | 0.000 | yaw disagreement 301; insufficient overlap 211; no accepted pairs 1 |
| IMU | 0.000 / 0.000 | [0.000, 0.000] | 0/512 | 0.000 | 0.000 | yaw disagreement 302; insufficient overlap 210; no accepted pairs 1 |
| Existing `/scan` | 0.000 / 0.000 | [0.000, 0.000] | 0/512 | 0.000 | 0.000 | yaw disagreement 281; insufficient overlap 231; no accepted pairs 1 |

Those zero method fields and the aggregate 0.000 m consensus are the raw
fail-closed values emitted when no pair is accepted; they are not usable
physical estimates. Each method reports forward offset 0.000 m, delta
-0.260 m from the recorded 0.260 m center, zero covered yaw sectors, and is
unreliable. The aggregate reason is `insufficient_reliable_methods`.

The independent sharpness sweep has a raw minimum at 0.23575000000000004 m
(score 0.0011207191715077018), 0.02425 m below the recorded offset, but it is
also unreliable because it did not meet the predeclared 3% prominence gate
(`sharpness_not_three_percent_sharper`). Treat 0.23575 m only as a candidate
for further measurement that requires explicit user approval. No TF, URDF,
bag data, or source configuration was changed, and this result does not
authorize changing any mount transform.

## Full bag replay

Immutable bag fixture: `/home/duc/robot/20260713_152907` (~221 s).

```bash
source install/setup.bash
ROS_DOMAIN_ID=79 ros2 launch orb_slam_bringup bag_replay.launch.py \
  bag_path:=/home/duc/robot/20260713_152907 \
  artifact_dir:=$PWD/artifacts/run-1 \
  rate:=1.0 start_dashboard:=true dashboard_host:=0.0.0.0
```

Launch arguments: `bag_path`, `artifact_dir`, `rate`, `ros_domain_id`,
`publish_odom_tf` (default true for bag replay; set false on the live robot),
`start_dashboard`, `dashboard_host`.

The launch:

- Publishes the two missing supplemental static mounts
  (`base_link→camera_link`, `base_link→base_scan`) after auditing bag
  `/tf_static` so they are never duplicated.
- Publishes `odom→base_link` from `/odom_wheel` (replay only).
- Starts the ORB-SLAM3 wrapper, lidar mapper, TF audit, and metrics recorder
  with `use_sim_time=true`.
- Plays the bag with `--clock` and shuts the graph down cleanly when playback
  ends so metrics flush.

### Dashboard URL

With `start_dashboard:=true` the launch prints:

```text
http://<dashboard_host>:51871/
```

Bind `dashboard_host:=0.0.0.0` for LAN/Tailscale access and open
`http://100.102.92.45:51871/` from a Tailscale-connected machine. There is no
`?ws=` query. Foxglove and `foxglove_bridge` are intentionally ignored/disabled
for now: measured visual tracking was approximately 5 Hz through the bridge
versus approximately 30 Hz through the direct dashboard. The supported
dashboard is the read-only in-graph HTTP server; do not source or launch a
Foxglove bridge for this path.

### Artifacts and report

Each run writes under `artifact_dir/`:

```text
metrics.json                 # bag / git / stereo / tracking / loops / map / diagnostics
events.jsonl                 # append-only event log
orb_trajectory.csv
wheel_trajectory.csv
corrected_trajectory.csv
map-revision-N.png           # occupancy preview per published map revision
final-map.pgm / final-map.yaml   # Nav2-compatible map export
report.html                  # self-contained static report (no network)
tf_audit.json
```

Inspect metrics and compare two runs:

```bash
orb_slam_report_check artifacts/run-1/metrics.json
orb_slam_report_check artifacts/run-2/metrics.json
orb_slam_compare_runs artifacts/run-1/metrics.json artifacts/run-2/metrics.json
```

`orb_slam_report_check` exits nonzero unless all acceptance gates pass
(stereo pairing, camera validation, ORB init, tracking OK ratio, loop
closures, atomic rebuild after each loop, zero wrongly-committed scans,
populated free/occupied cells, PGM/YAML match).

`orb_slam_compare_runs` checks structural repeatability: same ordered event
types, both closed a loop, same resolution, free/occupied cell counts within
15%, and trajectories within 10 cm / 3° at matched timestamps. Exact map
dimensions and revision counts are not required — multi-threaded ORB-SLAM3
is non-deterministic and produces different but equally-valid maps run to
run.

## Scan and dashboard lifecycle

- **Committed** scans enter the occupancy grid. They are placed while ORB
  tracking is healthy (OK) and the active atlas map is connected.
- **Provisional** scans are shown as yellow markers during visual loss and are
  excluded from the committed map. LOST scans remain provisional while the
  interval is unresolved. After successful recovery, the mapper rebuilds from
  corrected anchors and commits the interval only after that rebuild publishes.
- The dashboard receives a revisioned corrected path, not an append-only raw
  path. Its yellow fallback is only the current loss interval and is cleared
  after coherent successful recovery publication.
- RPLidar NaN and infinity sectors are intentionally ignored: they provide
  neither clearing evidence nor occupied evidence because filtered angles use
  invalid values. Adjacent finite beams retain normal behavior.
- Recorded `/tf_static` is trusted. The package is independent of the
  `tasteRobot2` package and the immutable bag fixture.

## Phase 1 limitation

Candidate outputs are namespaced under `orb_map` and `/orb_lidar/*` /
`/orb_slam3/*`. Phase 1 does **not** publish canonical `/map` or `map→odom`,
and does **not** control Nav2. The dashboard and map are observational
candidates for offline evaluation. The package is independent of `tasteRobot2`;
the immutable bag is `/home/duc/robot/20260713_152907`, and recorded
`/tf_static` is trusted while replay adds only verified missing mounts.

## Known residual

Mild wall "ghosting" (~5–15 cm) can remain on this bag due to residual
ORB-SLAM3 drift before the single loop closure. It does not prevent gate
passage; reducing it further is open-ended SLAM tuning outside Phase 1.

## License

GPL-3.0-or-later. The pinned upstream ORB-SLAM3 source and local adapter
patches retain their original notices.
