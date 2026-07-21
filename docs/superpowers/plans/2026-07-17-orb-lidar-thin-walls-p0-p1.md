# ORB Lidar Thin Walls P0+P1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce forward wall thickening by capping hit range and softening occupancy log-odds (P0), and expose insert diagnostics plus a unit-testable thickness helper (P1), while keeping pure ORB pose projection (no online ICP).

**Architecture:** Extend `GridConfig` / `TiledOccupancyGrid::insert` so hits only mark occupied within `hit_range_max_m`, free-space miss still uses `usable_range_m` (or clears only to hit cap when a finite hit is beyond the hit cap). Wire softer defaults and new ROS params through `MapperNode` → `MapRebuilder`. Track per-insert counters and publish them on `/diagnostics`. Add a pure helper to measure occupied thickness of a synthetic vertical wall strip for tests.

**Tech Stack:** C++17, GTest, ROS 2 (`rclcpp`, `diagnostic_msgs`), existing `orb_lidar_mapper` package.

## Global Constraints

- ORB owns global pose; **no** pose-moving online ICP.
- Live and rebuild share the **same** `GridConfig` / insert rules.
- No-return / NaN / ±inf free rays stay disabled (deskew layer; do not reintroduce).
- Default numbers (locked for this plan):
  - `hit_log_odds = 0.55F`
  - `miss_log_odds = -0.50F`
  - `hit_range_max_m = 10.0`
  - `usable_range_m` default in `GridConfig` remains `12.0`; mapper node still defaults `usable_range_m` param to `20.0`
- Miss range policy (locked): free-space clearing uses `usable_range_m` for normal rays; when `ray.has_hit && length > hit_range_max_m`, clear only up to `hit_range_max_m` and **do not** paint a hit (same idea as today's beyond-usable finite ray).
- Valid config requires `0 < hit_range_max_m <= usable_range_m` (both finite).
- Work on branch `feature/orb-lidar-thin-walls-p0-p1` from current `main`.
- TDD: failing test first for each behavior change.
- Do not implement IMU deskew, consistency gate, or bridge filter (P2+).

## File map

| File | Role |
|---|---|
| `orb_lidar_mapper/include/orb_lidar_mapper/tiled_occupancy_grid.hpp` | `GridConfig` fields, insert stats API |
| `orb_lidar_mapper/src/tiled_occupancy_grid.cpp` | Hit-range insert + counters |
| `orb_lidar_mapper/include/orb_lidar_mapper/wall_thickness.hpp` | Pure thickness helper (P1) |
| `orb_lidar_mapper/src/wall_thickness.cpp` | Implementation |
| `orb_lidar_mapper/test/tiled_occupancy_grid_test.cpp` | Grid default + hit-range tests |
| `orb_lidar_mapper/test/wall_thickness_test.cpp` | Thickness helper tests |
| `orb_lidar_mapper/src/mapper_node.cpp` / `.hpp` | Params + diagnostics keys |
| `orb_lidar_mapper/test/mapper_node_test.cpp` | Param default assertions if present |
| `orb_lidar_mapper/CMakeLists.txt` | New source + test target |
| `handoff-kiro.md` | Short note on new params (Task 4) |

---

### Task 1: GridConfig hit range + softer log-odds + insert behavior

**Files:**
- Modify: `orb_lidar_mapper/include/orb_lidar_mapper/tiled_occupancy_grid.hpp`
- Modify: `orb_lidar_mapper/src/tiled_occupancy_grid.cpp`
- Modify: `orb_lidar_mapper/test/tiled_occupancy_grid_test.cpp`

**Interfaces:**
- Produces: `GridConfig` with `hit_range_max_m` default `10.0`, `hit_log_odds` default `0.55F`, `miss_log_odds` default `-0.50F`
- Produces: insert semantics as Global Constraints
- Produces: `struct InsertStats { std::uint64_t hits_applied; std::uint64_t hits_range_skipped; };` and methods:
  - `InsertStats lastInsertStats() const`
  - `InsertStats cumulativeInsertStats() const`
  - Counters update inside `insert()`

- [ ] **Step 1: Write failing tests** in `tiled_occupancy_grid_test.cpp`

Update `HasSpecifiedDefaultConfiguration` expectations to new defaults and `hit_range_max_m == 10.0`.

Add tests:

```cpp
TEST(TiledOccupancyGrid, FiniteHitBeyondHitRangeClearsWithoutOccupiedEndpoint) {
  GridConfig config;
  config.resolution_m = 1.0;
  config.hit_range_max_m = 3.0;
  config.usable_range_m = 10.0;
  TiledOccupancyGrid grid(config);
  grid.insert({ray(10.0), ray(10.0)});  // has_hit true, length 10 > hit cap 3
  const GridSnapshot snapshot = grid.snapshot();
  EXPECT_EQ(snapshot.cellAt(0, 0), 0);
  EXPECT_EQ(snapshot.cellAt(1, 0), 0);
  EXPECT_EQ(snapshot.cellAt(2, 0), 0);
  EXPECT_EQ(snapshot.cellAt(3, 0), 0);
  EXPECT_EQ(snapshot.cellAt(4, 0), -1);  // no hit painted at 10
  const auto stats = grid.lastInsertStats();
  EXPECT_EQ(stats.hits_applied, 0U);
  EXPECT_EQ(stats.hits_range_skipped, 2U);
}

TEST(TiledOccupancyGrid, FiniteHitWithinHitRangeMarksOccupied) {
  GridConfig config;
  config.resolution_m = 1.0;
  config.hit_range_max_m = 5.0;
  config.usable_range_m = 10.0;
  TiledOccupancyGrid grid(config);
  grid.insert({ray(2.0)});
  EXPECT_EQ(grid.snapshot().cellAt(2, 0), 100);
  EXPECT_EQ(grid.lastInsertStats().hits_applied, 1U);
  EXPECT_EQ(grid.lastInsertStats().hits_range_skipped, 0U);
}

TEST(TiledOccupancyGrid, RejectsHitRangeAboveUsableRange) {
  GridConfig config;
  config.hit_range_max_m = 15.0;
  config.usable_range_m = 10.0;
  EXPECT_THROW(TiledOccupancyGrid grid(config), std::invalid_argument);
}
```

Keep existing `FiniteRayBeyondUsableRangeClearsWithoutOccupiedEndpoint` valid: set `hit_range_max_m` equal to `usable_range_m` (3.0) in that test so behavior stays “clear without hit beyond usable”.

- [ ] **Step 2: Run tests — expect FAIL** (missing field / old defaults)

```bash
cd /home/duc/robot/src/orb_slam3_ros2
source /opt/ros/*/setup.bash 2>/dev/null || true
colcon build --packages-select orb_lidar_mapper --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
# or build test target only if package already configured
```

- [ ] **Step 3: Implement**

In `tiled_occupancy_grid.hpp`:

```cpp
struct InsertStats {
  std::uint64_t hits_applied{0};
  std::uint64_t hits_range_skipped{0};
};

struct GridConfig {
  double resolution_m{0.05};
  int tile_size{64};
  float hit_log_odds{0.55F};
  float miss_log_odds{-0.50F};
  float min_log_odds{-4.0F};
  float max_log_odds{4.0F};
  float occupied_threshold{0.619F};
  float free_threshold{-0.619F};
  double usable_range_m{12.0};
  double hit_range_max_m{10.0};
};
```

Add to class:

```cpp
  InsertStats lastInsertStats() const;
  InsertStats cumulativeInsertStats() const;
```

Private members: `InsertStats last_insert_{}; InsertStats cumulative_insert_{};`

In `validConfig`: require finite `hit_range_max_m > 0` and `hit_range_max_m <= usable_range_m`.

In `insert()`:

```cpp
last_insert_ = {};
// per ray after length known:
const bool beyond_hit_cap = ray.has_hit && std::isfinite(length) && length > config_.hit_range_max_m;
const bool has_hit = ray.has_hit && length <= config_.hit_range_max_m;
double used_length;
if (infinite_clear) {
  used_length = config_.usable_range_m;
} else if (beyond_hit_cap) {
  used_length = config_.hit_range_max_m;
  ++last_insert_.hits_range_skipped;
} else {
  used_length = std::min(length, config_.usable_range_m);
}
// ... bresenham as today ...
if (has_hit) {
  update(end, config_.hit_log_odds);
  ++last_insert_.hits_applied;
}
// after loop:
cumulative_insert_.hits_applied += last_insert_.hits_applied;
cumulative_insert_.hits_range_skipped += last_insert_.hits_range_skipped;
```

Zero-length hit path: still counts as `hits_applied` when painted.

- [ ] **Step 4: Run `tiled_occupancy_grid_test` — all PASS**

- [ ] **Step 5: Commit**

```bash
git add orb_lidar_mapper/include/orb_lidar_mapper/tiled_occupancy_grid.hpp \
        orb_lidar_mapper/src/tiled_occupancy_grid.cpp \
        orb_lidar_mapper/test/tiled_occupancy_grid_test.cpp
git commit -m "feat: cap occupancy hits and soften log-odds for thinner walls"
```

---

### Task 2: Wire GridConfig params through MapperNode

**Files:**
- Modify: `orb_lidar_mapper/include/orb_lidar_mapper/mapper_node.hpp`
- Modify: `orb_lidar_mapper/src/mapper_node.cpp`
- Modify: `orb_lidar_mapper/test/mapper_node_test.cpp` (default param checks)

**Interfaces:**
- Consumes: Task 1 `GridConfig` fields
- Produces: ROS params `hit_range_max_m` (default 10.0), `hit_log_odds` (0.55), `miss_log_odds` (-0.50) applied to rebuilder grid config with `resolution_m` and `usable_range_m`

- [ ] **Step 1: Write/update failing param test**

In `mapper_node_test.cpp` near existing usable_range default check (~1034):

```cpp
EXPECT_NEAR(mapper_->get_parameter("hit_range_max_m").as_double(), 10.0, 1e-9);
EXPECT_NEAR(mapper_->get_parameter("hit_log_odds").as_double(), 0.55, 1e-6);
EXPECT_NEAR(mapper_->get_parameter("miss_log_odds").as_double(), -0.50, 1e-6);
```

- [ ] **Step 2: Run test — FAIL** (params missing)

- [ ] **Step 3: Implement**

Declare parameters in constructor init list / body:

```cpp
hit_range_max_m_(declare_parameter("hit_range_max_m", 10.0)),
hit_log_odds_(declare_parameter("hit_log_odds", 0.55)),
miss_log_odds_(declare_parameter("miss_log_odds", -0.50)),
```

Validate finite and `0 < hit_range_max_m_ <= usable_range_m_` (throw `invalid_argument` like other gates).

```cpp
grid_cfg.hit_range_max_m = hit_range_max_m_;
grid_cfg.hit_log_odds = static_cast<float>(hit_log_odds_);
grid_cfg.miss_log_odds = static_cast<float>(miss_log_odds_);
```

Add members on `MapperNode`.

- [ ] **Step 4: Run mapper_node_test + tiled tests — PASS**

- [ ] **Step 5: Commit**

```bash
git commit -m "feat: expose hit range and log-odds params on orb_lidar_mapper"
```

---

### Task 3: Diagnostics counters + wall thickness helper (P1)

**Files:**
- Create: `orb_lidar_mapper/include/orb_lidar_mapper/wall_thickness.hpp`
- Create: `orb_lidar_mapper/src/wall_thickness.cpp`
- Create: `orb_lidar_mapper/test/wall_thickness_test.cpp`
- Modify: `orb_lidar_mapper/src/mapper_node.cpp` (diagnostics keys)
- Modify: `orb_lidar_mapper/include/orb_lidar_mapper/map_rebuilder.hpp` + `.cpp` to expose cumulative insert stats from live grid
- Modify: `orb_lidar_mapper/CMakeLists.txt`

**Interfaces:**
- Produces:

```cpp
// wall_thickness.hpp
namespace orb_lidar_mapper {
// Count consecutive occupied (100) cells along +X from (start_cell_x, cell_y)
// in a GridSnapshot. Returns 0 if start is not occupied.
std::size_t occupiedRunLengthX(const GridSnapshot& snap,
                               std::int64_t start_cell_x,
                               std::int64_t cell_y);
}
```

- Produces: diagnostics key values `hits_applied`, `hits_range_skipped` from rebuilder cumulative stats (or node-local counters updated when map publishes — prefer reading from rebuilder).

Minimal rebuilder API:

```cpp
InsertStats cumulativeInsertStats() const;  // thread-safe snapshot of grid cumulative stats
```

Implementation: store stats on `Impl` updated whenever grid insert runs (appendCommitted path and full rebuild). On full rebuild, reset cumulative to the rebuilt grid’s cumulative after all inserts (or re-sum). Simplest correct approach: after each `grid->insert`, copy `grid->cumulativeInsertStats()` into `Impl::insert_stats_`. On full rebuild replace grid then set stats from new grid after rebuild completes.

- [ ] **Step 1: Failing tests for thickness helper**

```cpp
TEST(WallThickness, CountsOccupiedRunAlongX) {
  GridConfig config;
  config.resolution_m = 1.0;
  config.hit_range_max_m = 10.0;
  TiledOccupancyGrid grid(config);
  // Paint a thick wall at x=5 by inserting hits with small lateral offsets via multiple origins
  for (int i = 0; i < 3; ++i) {
    grid.insert({{{{0.0, 0.0}, {5.0 + 0.1 * i, 0.0}, true}}});
  }
  // Or manually: insert rays that hit cells 5,6,7
  // Prefer deterministic: three hits at x=5,6,7
  grid.insert({{{{0,0},{5,0},true}, {{0,0},{6,0},true}, {{0,0},{7,0},true}}});
  const auto snap = grid.snapshot();
  EXPECT_GE(occupiedRunLengthX(snap, 5, 0), 1U);
}
```

Also a pure synthetic snapshot test without insert if easier: build `GridSnapshot` with width/height/cells manually and assert run length 3.

- [ ] **Step 2: Implement helper + CMake + rebuilder stats + diagnostics**

In `publishDiagnostics`, add:

```cpp
add_kv("hits_applied", rebuilder_->cumulativeInsertStats().hits_applied);
add_kv("hits_range_skipped", rebuilder_->cumulativeInsertStats().hits_range_skipped);
```

- [ ] **Step 3: Run full `orb_lidar_mapper` tests — PASS**

- [ ] **Step 4: Commit**

```bash
git commit -m "feat: insert hit diagnostics and wall thickness helper"
```

---

### Task 4: Operator note + full package verification

**Files:**
- Modify: `handoff-kiro.md` (short subsection under parameters)

- [ ] **Step 1: Document** new params and that thickness fix is occupancy-first (no IMU yet).

- [ ] **Step 2: Full test**

```bash
cd /home/duc/robot/src/orb_slam3_ros2
colcon test --packages-select orb_lidar_mapper --event-handlers console_direct+
colcon test-result --verbose
```

Expected: all tests pass (0 failures).

- [ ] **Step 3: Commit docs**

```bash
git commit -m "docs: note thin-wall occupancy params in handoff"
```

---

## Spec coverage (self-check)

| Spec P0/P1 item | Task |
|---|---|
| hit_range_max_m | 1, 2 |
| softer hit/miss log-odds | 1, 2 |
| live+rebuild same config | 2 (single GridConfig into MapRebuilder) |
| miss vs hit range policy | 1 (locked in Global Constraints) |
| hits_applied / hits_range_skipped diagnostics | 3 |
| thickness metric helper | 3 |
| no ICP / no IMU | all tasks (out of scope) |
| operator note | 4 |

## Out of scope (do not implement)

- IMU deskew, archive IMU, bridge filter, consistency gate, yaw gate changes
