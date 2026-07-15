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
| `orb_slam_bringup` | Bag replay, TF audit, metrics, report, read-only bridge launch |
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

## Full bag replay

Immutable bag fixture: `/home/duc/robot/20260713_152907` (~221 s).

```bash
source install/setup.bash
ROS_DOMAIN_ID=79 ros2 launch orb_slam_bringup bag_replay.launch.py \
  bag_path:=/home/duc/robot/20260713_152907 \
  artifact_dir:=$PWD/artifacts/run-1 \
  rate:=1.0 start_dashboard:=true dashboard_host:=100.102.92.45
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
http://<dashboard_host>:51871/?ws=ws://<dashboard_host>:8765
```

Phase-1 default Tailscale host: `100.102.92.45`. The HTTP server and
foxglove_bridge bind to the configured host; capabilities are locked down
(empty) so the dashboard is observational only — no publish, service, or
parameter write.

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

## Committed vs provisional scans

- **Committed** scans enter the occupancy grid. They are placed while ORB
  tracking is healthy (OK) and the active atlas map is connected.
- **Provisional** scans are shown as yellow markers during visual loss and are
  excluded from the committed map. On relocalization / loop correction the
  mapper rebuilds the grid atomically from archived scans and corrected graph
  poses, then deletes the provisional markers.

## Phase 1 limitation

Candidate outputs are namespaced under `orb_map` and `/orb_lidar/*` /
`/orb_slam3/*`. Phase 1 does **not** publish canonical `/map` or `map→odom`,
and does **not** control Nav2. The dashboard and map are observational
candidates for offline evaluation.

## Known residual

Mild wall "ghosting" (~5–15 cm) can remain on this bag due to residual
ORB-SLAM3 drift before the single loop closure. It does not prevent gate
passage; reducing it further is open-ended SLAM tuning outside Phase 1.

## License

GPL-3.0-or-later. The pinned upstream ORB-SLAM3 source and local adapter
patches retain their original notices.
