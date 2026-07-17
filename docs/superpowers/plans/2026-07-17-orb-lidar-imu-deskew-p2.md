# ORB Lidar IMU Deskew P2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fuse Handsfree ~200 Hz `/imu` yaw-rate into live scan deskew (wheel xy + IMU Δyaw) under existing ORB residual^α, without changing ORB global authority.

**Architecture:** Add `ImuYawBuffer` (timestamped ω_z, trapezoidal integrate, 20 ms max gap). Extend `ScanDeskewer` to optionally take an IMU buffer and build fused SE(2) poses: translation from wheel relative, yaw from IMU integral from scan start. `MapperNode` subscribes to `/imu`, feeds the buffer, and passes it into deskew; if IMU coverage fails, fall back to wheel-only deskew (do not drop the scan solely for missing IMU).

**Tech Stack:** C++17, GTest, ROS 2 `sensor_msgs/Imu`, existing `orb_lidar_mapper`.

## Global Constraints

- ORB remains sole global pose authority; no pose-moving ICP.
- IMU is **relative motion only** inside a scan / between ray stamps.
- Max IMU sample gap: **20_000_000 ns** (20 ms), same spirit as offline calib.
- Default IMU topic: **`/imu`** (Handsfree); param `imu_topic`.
- Param `enable_imu_deskew` default **true**; when false, behavior identical to wheel-only.
- On IMU gap / no coverage for a scan: **fall back to wheel-only** (log/count `imu_deskew_fallbacks_`); do not reject the scan for IMU alone.
- Contiguous duplicate IMU stamps: average ω_z (like bag reader).
- Keep turn gate `max_scan_yaw_change_rad` unchanged.
- Archive still stores per-ray **wheel** poses as today (rebuild model unchanged this phase); fused motion is live deskew only (P3 archive IMU is out of scope).
- Branch: `feature/orb-lidar-thin-walls-p0-p1`
- TDD for new buffer + deskew fusion tests.

## File map

| File | Role |
|---|---|
| `include/orb_lidar_mapper/imu_yaw_buffer.hpp` | API |
| `src/imu_yaw_buffer.cpp` | push / integrate / coverage |
| `test/imu_yaw_buffer_test.cpp` | unit tests |
| `include/.../scan_deskewer.hpp` + `src/scan_deskewer.cpp` | optional IMU fusion in deskew + deskewBracketed |
| `test/scan_deskewer_test.cpp` | fusion tests |
| `mapper_node.hpp` + `mapper_node.cpp` | subscribe `/imu`, wire buffer |
| `CMakeLists.txt` | new sources + test |
| `handoff-kiro.md` | short note |

---

### Task 1: ImuYawBuffer

**Files:**
- Create: `orb_lidar_mapper/include/orb_lidar_mapper/imu_yaw_buffer.hpp`
- Create: `orb_lidar_mapper/src/imu_yaw_buffer.cpp`
- Create: `orb_lidar_mapper/test/imu_yaw_buffer_test.cpp`
- Modify: `orb_lidar_mapper/CMakeLists.txt`

**Interfaces:**
```cpp
struct TimedYawRate {
  std::int64_t stamp_ns{};
  double omega_rad_s{};
};

class ImuYawBuffer {
 public:
  explicit ImuYawBuffer(std::int64_t retention_ns,
                        std::int64_t max_gap_ns = 20'000'000LL);
  // Returns false if non-finite omega or non-monotonic stamp (strictly decreasing).
  // Equal stamp: average omega into last sample (duplicate stamp policy).
  bool push(TimedYawRate sample);
  // True if rates cover [start,end] with no gap > max_gap_ns.
  bool covers(std::int64_t start_ns, std::int64_t end_ns) const;
  // Trapezoidal integrate ω from start to target; nullopt if !covers(start,target).
  std::optional<double> integratedYaw(std::int64_t start_ns, std::int64_t target_ns) const;
  std::size_t size() const noexcept;
};
```

- [ ] **Step 1: Failing tests** — constant ω=1.0 for 0.1 s → yaw≈0.1; gap >20ms → covers false; duplicate stamp averages.

- [ ] **Step 2: Implement + CMake** — add to `_core` library.

- [ ] **Step 3: Tests pass + commit**

```bash
git commit -m "feat: add ImuYawBuffer for live deskew yaw integration"
```

---

### Task 2: Fuse IMU yaw into ScanDeskewer

**Files:**
- Modify: `scan_deskewer.hpp`, `scan_deskewer.cpp`, `scan_deskewer_test.cpp`

**Interfaces:**
Add optional const `ImuYawBuffer*` (default nullptr) to both:
```cpp
static std::optional<std::vector<Ray2>> deskew(...,
  const TimedPoseBuffer& wheels,
  const ImuYawBuffer* imu = nullptr);

static std::optional<BracketedDeskewResult> deskewBracketed(...,
  const TimedPoseBuffer& wheels,
  const ScanMotionBracket& bracket,
  const ImuYawBuffer* imu = nullptr);
```

**Fusion rule** (when `imu != nullptr` and `imu->covers(scan.stamp_ns, last_ray_stamp)`):
For each ray stamp `t`:
1. `wheel_t = wheels.interpolate(t)` (still required)
2. `wheel_0 = wheels.interpolate(scan.stamp_ns)`
3. `rel = inv(wheel_0) * wheel_t`
4. `dyaw = *imu->integratedYaw(scan.stamp_ns, t)`
5. `fused = wheel_0 * Pose2{rel.x, rel.y, dyaw}`
6. Use `fused` wherever `wheel_pose` was used in bracketedBasePose / relative paths

If `imu == nullptr` or coverage fails for the full sweep: use pure wheel (existing behavior). For bracketed path, if imu non-null but coverage incomplete → treat as coverage fail → wheel-only for whole scan (do not mix mid-scan).

- [ ] **Step 1: Test** — synthetic constant ω and zero wheel yaw change: fused rays must rotate with IMU; wheel-only path unchanged when imu=nullptr.

- [ ] **Step 2: Implement**

- [ ] **Step 3: Commit**

```bash
git commit -m "feat: fuse IMU yaw into scan deskew (wheel xy + gyro)"
```

---

### Task 3: MapperNode IMU subscription + diagnostics

**Files:**
- Modify: `mapper_node.hpp`, `mapper_node.cpp`
- Modify: `mapper_node_test.cpp` if easy (param defaults)
- Modify: `bag_replay.launch.py` optional — no change required if defaults work (`/imu`)

**Params:**
- `imu_topic` default `"/imu"`
- `enable_imu_deskew` default `true`
- `imu_retention_s` default same as `wheel_retention_s` or 30.0
- `imu_max_gap_ms` default `20.0`

**Behavior:**
- Subscribe `sensor_msgs/Imu` on `imu_topic`
- `onImu`: lock mutex, push `{stamp, angular_velocity.z}` to `imu_buf_`
- Pass `enable_imu_deskew_ ? imu_buf_.get() : nullptr` into deskew calls
- Counter `imu_deskew_fallbacks_` when enable true but deskew used wheel-only due to coverage (detect by checking covers before call, or compare — simplest: check `imu_buf_->covers(start,end)` before deskew; if false ++fallback)
- Diagnostics keys: `imu_samples`, `imu_deskew_fallbacks`, `enable_imu_deskew`

- [ ] **Step 1: Implement + param default test**

- [ ] **Step 2: Full `orb_lidar_mapper` tests pass**

- [ ] **Step 3: Commit**

```bash
git commit -m "feat: subscribe /imu and enable live IMU deskew in mapper"
```

---

### Task 4: Docs + bag replay verification

**Files:**
- Modify: `handoff-kiro.md` (IMU deskew note)

- [ ] **Step 1: Document params and fallback**

- [ ] **Step 2: Rebuild and run**

```bash
cd /home/duc/robot/src/orb_slam3_ros2
colcon build --packages-select orb_lidar_mapper orb_slam_bringup
source install/setup.bash
tools/run_full_stack_dashboard.sh \
  --bag /home/duc/robot/bag/forward-and-back-origin \
  --domain 93 \
  --rate 2.0 \
  --dashboard-host 127.0.0.1 \
  --output tools/full-stack-report/forward-and-back-origin-imu-deskew
```

- [ ] **Step 3: Confirm metrics/report written; note imu_deskew_fallbacks if visible in diagnostics**

- [ ] **Step 4: Commit docs**

```bash
git commit -m "docs: note live IMU deskew params and bag replay"
```

---

## Spec coverage

| Design P2 item | Task |
|---|---|
| Handsfree /imu yaw integrate | 1–3 |
| Wheel xy + IMU yaw | 2 |
| residual^α unchanged | 2 (bracket path) |
| 20 ms gap | 1 |
| Fall back / no hard reject | 2–3 |
| No ICP / no ORB replace | all |
| Bag forward-and-back-origin | 4 |

## Out of scope

- Archive IMU for rebuild (P3)
- Bridge ESKF filter
- Raising yaw gate
- Consistency gate
