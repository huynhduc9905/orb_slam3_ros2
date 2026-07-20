# Loop-Closure Revisit via Keyframe-ID Gap — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Allow ORB-SLAM3 to loop-close when the robot re-drives a previously mapped area (later laps), by gating the covisibility-based loop-candidate exclusion on a keyframe-ID gap.

**Architecture:** Add one pure, header-only predicate `orb_slam3_wrapper_fork::shouldExcludeConnectedLoopCandidate(current_id, candidate_id)` in the vendored ORB-SLAM3 fork. Call it at the two covisibility exclusion gates (`KeyFrameDatabase::DetectNBestCandidates` and `LoopClosing::DetectCommonRegionsFromBoW`) so connected keyframes are only excluded when they are recent trajectory neighbors, not distant revisits. All downstream geometric verification is unchanged.

**Tech Stack:** C++ (vendored ORB-SLAM3), CMake, ament_cmake_gtest, ROS 2 (build via `nix develop --command bash -lc '...'`), colcon.

## Global Constraints

- Scope is the fork only: `orb_slam3_vendor/vendor/ORB_SLAM3` (a nested git submodule, remote `huynhduc9905/ORB_SLAM3`). Vendor changes MUST be committed inside the submodule; the vendor package builds from committed submodule state via `git clone`.
- Do NOT change merge-detection logic, the `>=3` loop coincidence gate, mapper code, metrics schema, replay runner, acceptance thresholds, or camera/ORB parameters.
- Hardcoded constant, no ROS/YAML parameter plumbing: `kLoopRevisitMinKFGap = 20`.
- All builds/tests run inside the nix devshell: `nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; <cmd>'`.
- A clean vendor rebuild requires removing `build/orb_slam3_vendor` and `install/orb_slam3_vendor` first (CMake caches the submodule clone).
- `mnId` is `long unsigned int` (unsigned); guard against unsigned underflow with `current_id >= candidate_id` before subtracting.

---

### Task 1: Pure id-gap predicate + unit tests

**Files:**
- Create: `orb_slam3_vendor/vendor/ORB_SLAM3/include/LoopRevisitPolicy.h` (in submodule)
- Create: `orb_slam3_vendor/test/loop_revisit_policy_test.cpp` (main repo)
- Modify: `orb_slam3_vendor/CMakeLists.txt` (register the gtest)

**Interfaces:**
- Produces: header `LoopRevisitPolicy.h` in namespace `orb_slam3_wrapper_fork` with:
  - `constexpr unsigned long kLoopRevisitMinKFGap = 20;`
  - `inline bool shouldExcludeConnectedLoopCandidate(unsigned long current_id, unsigned long candidate_id);`
  - Contract: returns `true` (exclude, original behavior) when the candidate is a recent trajectory neighbor — i.e. `current_id >= candidate_id && current_id - candidate_id < kLoopRevisitMinKFGap`. Returns `false` (allow as loop candidate) otherwise, including when `candidate_id > current_id` (future ids never treated as recent-neighbor exclusions).
  - Task 2 consumes this from both exclusion gates.

- [ ] **Step 1: Write the failing test**

Create `orb_slam3_vendor/test/loop_revisit_policy_test.cpp`:

```cpp
#include <gtest/gtest.h>

#include <LoopRevisitPolicy.h>

using orb_slam3_wrapper_fork::kLoopRevisitMinKFGap;
using orb_slam3_wrapper_fork::shouldExcludeConnectedLoopCandidate;

TEST(LoopRevisitPolicy, ExcludesImmediateNeighbor) {
  // gap 1 -> recent neighbor -> exclude (original behavior)
  EXPECT_TRUE(shouldExcludeConnectedLoopCandidate(101, 100));
}

TEST(LoopRevisitPolicy, ExcludesJustBelowThreshold) {
  // gap 19 (< 20) -> still a neighbor -> exclude
  EXPECT_TRUE(shouldExcludeConnectedLoopCandidate(100, 81));
}

TEST(LoopRevisitPolicy, AllowsAtThreshold) {
  // gap 20 (>= 20) -> revisit -> allow as candidate
  EXPECT_FALSE(shouldExcludeConnectedLoopCandidate(100, 80));
}

TEST(LoopRevisitPolicy, AllowsDistantRevisit) {
  // lap-2 current id vs lap-1 revisited id, gap ~40+ -> allow
  EXPECT_FALSE(shouldExcludeConnectedLoopCandidate(120, 78));
}

TEST(LoopRevisitPolicy, SameIdIsRecentNeighbor) {
  // gap 0 -> exclude
  EXPECT_TRUE(shouldExcludeConnectedLoopCandidate(50, 50));
}

TEST(LoopRevisitPolicy, FutureCandidateIsAllowedNotUnderflow) {
  // candidate_id > current_id must NOT be treated as a huge unsigned gap
  // nor as a recent neighbor; policy returns false (allow), no underflow.
  EXPECT_FALSE(shouldExcludeConnectedLoopCandidate(80, 120));
}
```

- [ ] **Step 2: Register the test in CMake**

In `orb_slam3_vendor/CMakeLists.txt`, inside the existing `if(BUILD_TESTING)` block (alongside the other `ament_add_gtest` calls, e.g. after the `vendor_link_test` lines), add:

```cmake
  ament_add_gtest(loop_revisit_policy_test test/loop_revisit_policy_test.cpp)
  target_link_libraries(loop_revisit_policy_test ORB_SLAM3::ORB_SLAM3)
  set_tests_properties(loop_revisit_policy_test PROPERTIES TIMEOUT 10)
```

- [ ] **Step 3: Run the test to verify it fails**

Run:
```bash
rm -rf build/orb_slam3_vendor install/orb_slam3_vendor
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; colcon build --packages-select orb_slam3_vendor'
```
Expected: FAIL — configure/compile error because `LoopRevisitPolicy.h` does not exist yet (`fatal error: LoopRevisitPolicy.h: No such file or directory`).

- [ ] **Step 4: Create the header**

Create `orb_slam3_vendor/vendor/ORB_SLAM3/include/LoopRevisitPolicy.h`:

```cpp
#ifndef LOOP_REVISIT_POLICY_H
#define LOOP_REVISIT_POLICY_H

// Local fork addition. Decides whether a covisibility-connected keyframe should
// be excluded as a loop-closure candidate. Original ORB-SLAM3 excludes every
// connected keyframe, which prevents loop closure when the robot re-drives a
// previously mapped area on a later lap (the revisited keyframes become
// covisibility-connected). We exclude a connected keyframe only when it is a
// recent trajectory neighbor, measured by keyframe-id gap; distant connected
// keyframes are genuine revisits and remain eligible for the normal geometric
// (Sim3 + coincidence) verification downstream.

namespace orb_slam3_wrapper_fork {

// Minimum keyframe-id gap for a covisibility-connected keyframe to still be
// eligible as a loop-closure candidate. Below this gap the connected keyframe
// is treated as an immediate trajectory neighbor and excluded (original
// behavior). At or above it, the keyframe is treated as a genuine revisit.
constexpr unsigned long kLoopRevisitMinKFGap = 20;

// Returns true when the connected candidate should be EXCLUDED as a loop
// candidate (recent trajectory neighbor); false when it should be ALLOWED
// (distant revisit, or a future/equal id that must not underflow).
inline bool shouldExcludeConnectedLoopCandidate(unsigned long current_id,
                                                unsigned long candidate_id) {
  if (current_id < candidate_id) {
    return false;
  }
  return (current_id - candidate_id) < kLoopRevisitMinKFGap;
}

}  // namespace orb_slam3_wrapper_fork

#endif  // LOOP_REVISIT_POLICY_H
```

- [ ] **Step 5: Run the test to verify it passes**

Run:
```bash
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; colcon build --packages-select orb_slam3_vendor && colcon test --packages-select orb_slam3_vendor --ctest-args -R loop_revisit_policy_test && colcon test-result --verbose'
```
Expected: `loop_revisit_policy_test` PASSES (6/6 assertions), 0 failures.

- [ ] **Step 6: Commit the header inside the submodule**

```bash
cd orb_slam3_vendor/vendor/ORB_SLAM3
git add include/LoopRevisitPolicy.h
git commit -m "feat: add keyframe-id-gap loop revisit policy helper"
cd -
```

- [ ] **Step 7: Commit the test + CMake wiring in the main repo**

```bash
git add orb_slam3_vendor/test/loop_revisit_policy_test.cpp orb_slam3_vendor/CMakeLists.txt
git commit -m "test: cover loop revisit id-gap policy"
```

---

### Task 2: Apply the predicate at both covisibility exclusion gates

**Files:**
- Modify: `orb_slam3_vendor/vendor/ORB_SLAM3/src/KeyFrameDatabase.cc` (~line 626, in `DetectNBestCandidates`)
- Modify: `orb_slam3_vendor/vendor/ORB_SLAM3/src/LoopClosing.cc` (~lines 628-641, in `DetectCommonRegionsFromBoW`)

**Interfaces:**
- Consumes: `orb_slam3_wrapper_fork::shouldExcludeConnectedLoopCandidate` and `LoopRevisitPolicy.h` from Task 1.
- Produces: no new symbols; changes runtime candidate-selection behavior only.

- [ ] **Step 1: Add the include to KeyFrameDatabase.cc**

At the top of `orb_slam3_vendor/vendor/ORB_SLAM3/src/KeyFrameDatabase.cc`, with the other project includes (e.g. after `#include "KeyFrameDatabase.h"`), add:

```cpp
#include "LoopRevisitPolicy.h"
```

- [ ] **Step 2: Replace the Gate 1 exclusion in DetectNBestCandidates**

In `KeyFrameDatabase::DetectNBestCandidates`, the current block (around line 623-632) reads:

```cpp
                if(pKFi->mnPlaceRecognitionQuery!=pKF->mnId)
                {
                    pKFi->mnPlaceRecognitionWords=0;
                    if(!spConnectedKF.count(pKFi))
                    {

                        pKFi->mnPlaceRecognitionQuery=pKF->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
```

Replace the inner `if(!spConnectedKF.count(pKFi))` condition so a connected
keyframe is only excluded when it is a recent neighbor:

```cpp
                if(pKFi->mnPlaceRecognitionQuery!=pKF->mnId)
                {
                    pKFi->mnPlaceRecognitionWords=0;
                    // Fork change: exclude a covisibility-connected keyframe only
                    // when it is a recent trajectory neighbor (small id gap).
                    // Distant connected keyframes are genuine revisits and remain
                    // eligible loop candidates.
                    const bool connected = spConnectedKF.count(pKFi) != 0;
                    const bool exclude = connected &&
                        orb_slam3_wrapper_fork::shouldExcludeConnectedLoopCandidate(
                            pKF->mnId, pKFi->mnId);
                    if(!exclude)
                    {

                        pKFi->mnPlaceRecognitionQuery=pKF->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
```

- [ ] **Step 3: Add the include to LoopClosing.cc**

At the top of `orb_slam3_vendor/vendor/ORB_SLAM3/src/LoopClosing.cc`, with the other project includes, add:

```cpp
#include "LoopRevisitPolicy.h"
```

- [ ] **Step 4: Replace the Gate 2 abort in DetectCommonRegionsFromBoW**

In `LoopClosing::DetectCommonRegionsFromBoW`, the current block (around line 628-641) reads:

```cpp
        bool bAbortByNearKF = false;
        for(int j=0; j<vpCovKFi.size(); ++j)
        {
            if(spConnectedKeyFrames.find(vpCovKFi[j]) != spConnectedKeyFrames.end())
            {
                bAbortByNearKF = true;
                break;
            }
        }
```

Replace it so the abort only triggers for a recent connected neighbor:

```cpp
        bool bAbortByNearKF = false;
        for(int j=0; j<vpCovKFi.size(); ++j)
        {
            KeyFrame* pCov = vpCovKFi[j];
            // Fork change: abort on a near covisibility neighbor only when it is
            // a recent trajectory neighbor by id gap. Distant connected keyframes
            // are genuine revisits and must be allowed through to Sim3 checks.
            if(pCov && spConnectedKeyFrames.find(pCov) != spConnectedKeyFrames.end() &&
               orb_slam3_wrapper_fork::shouldExcludeConnectedLoopCandidate(
                   mpCurrentKF->mnId, pCov->mnId))
            {
                bAbortByNearKF = true;
                break;
            }
        }
```

- [ ] **Step 5: Clean rebuild the vendor + dependents**

Run:
```bash
rm -rf build/orb_slam3_vendor install/orb_slam3_vendor
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; colcon build --packages-select orb_slam3_vendor orb_slam3_wrapper orb_slam_bringup'
```
Expected: all three packages build. (ORB_SLAM3 recompiles — build takes minutes, not seconds; a sub-second vendor build means the change was NOT picked up.)

- [ ] **Step 6: Verify the change reached the compiled build source**

Run:
```bash
grep -n 'shouldExcludeConnectedLoopCandidate' build/orb_slam3_vendor/ORB_SLAM3/src/KeyFrameDatabase.cc build/orb_slam3_vendor/ORB_SLAM3/src/LoopClosing.cc
```
Expected: one match in each file.

- [ ] **Step 7: Run existing vendor tests to confirm no regression**

Run:
```bash
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; colcon test --packages-select orb_slam3_vendor && colcon test-result --verbose'
```
Expected: all vendor tests pass, including `loop_closing_notification_test`, `snapshot_value_test`, `system_shutdown_test`, and `loop_revisit_policy_test`. 0 failures.

- [ ] **Step 8: Commit the source changes inside the submodule**

```bash
cd orb_slam3_vendor/vendor/ORB_SLAM3
git add src/KeyFrameDatabase.cc src/LoopClosing.cc
git commit -m "fix: allow distant covisible keyframes as loop candidates via id gap"
cd -
```

---

### Task 3: Validate on the circle bag (lap-2 closes, double wall resolved, no false closures)

**Files:**
- Modify (temporary, reverted at end): `orb_slam_bringup/launch/bag_replay.launch.py` — add the read-only `log_keyframe_drift` diagnostic flag for this run only.
- Produces: `artifacts/circle-loop-revisit-idgap-20260720/` and its `.launch.log`.

**Interfaces:**
- Consumes: the rebuilt vendor from Task 2.
- Baseline for comparison (th=3, no fix): lap-1 vs lap-2 ORB pose diff mean 0.29 m / max 0.61 m; exactly one `LOOP_CLOSED`; corrected trajectory ends at bag+42s; 1 mapper full rebuild.

- [ ] **Step 1: Temporarily enable the drift diagnostic in the launch**

In `orb_slam_bringup/launch/bag_replay.launch.py`, in the `wrapper_cmd_parts` list, after the `settings_file` `-p` entry, add:

```python
        "-p", "log_keyframe_drift:=true",
```

Then rebuild bringup:
```bash
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; colcon build --packages-select orb_slam_bringup'
```

- [ ] **Step 2: Run the circle bag**

Run:
```bash
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; ros2 launch orb_slam_bringup bag_replay.launch.py bag_path:=/home/duc/bag/circle-run artifact_dir:=artifacts/circle-loop-revisit-idgap-20260720 rate:=1 ros_domain_id:=75 start_dashboard:=false benchmark_mode:=off' > 'artifacts/circle-loop-revisit-idgap-20260720.launch.log' 2>&1; echo "EXIT=$?"
```
Expected: `EXIT=0`.

- [ ] **Step 3: Check loop closures and graph revisions**

Run:
```bash
grep -aE 'Loop detected|graph_observation|classification' 'artifacts/circle-loop-revisit-idgap-20260720.launch.log' | sed -E 's/^\[orb_slam3_wrapper-3\] //'
```
Expected (success criteria): more than one `*Loop detected`, at least one additional `LOOP_CLOSED`/`graph_observation stage=changed` occurring during lap 2 (bag+~50s onward), and graph revision advancing past 3. If only one closure at bag+~47s and revision stops at 3, the fix did not work — STOP and report (do not tune further without discussion).

- [ ] **Step 4: Measure lap-1 vs lap-2 ORB pose difference and mapper rebuilds**

Run:
```bash
python3 - <<'EOF'
import csv, math, bisect, json
d="artifacts/circle-loop-revisit-idgap-20260720"
def load(p):
    r=list(csv.DictReader(open(f"{d}/{p}")))
    return [(float(x["t"]),float(x["x"]),float(x["y"])) for x in r]
wheel=load("wheel_trajectory.csv"); orb=load("orb_trajectory.csv")
t0=wheel[0][0]
ots=[o[0] for o in orb]
def orb_at(t):
    i=bisect.bisect_left(ots,t); i=min(max(i,0),len(orb)-1); return orb[i]
lap1=[w for w in wheel if 8<w[0]-t0<47]
lap2=[w for w in wheel if 50<w[0]-t0<89]
pairs=[]
for w1 in lap1[::20]:
    best=min(lap2,key=lambda w2: math.hypot(w2[1]-w1[1],w2[2]-w1[2]))
    if math.hypot(best[1]-w1[1],best[2]-w1[2])<0.10:
        o1=orb_at(w1[0]); o2=orb_at(best[0])
        pairs.append(math.hypot(o2[1]-o1[1],o2[2]-o1[2]))
print(f"same-location pairs={len(pairs)}")
if pairs:
    print(f"ORB lap1-vs-lap2 diff: mean={sum(pairs)/len(pairs):.3f}m max={max(pairs):.3f}m (baseline mean=0.290 max=0.607)")
m=json.load(open(f"{d}/metrics.json"))
mr=m["map_revisions"]; pub=[r for r in mr if r["state"]=="PUBLISHED"]
full=[r for r in pub if r["input_scan_count"]>1]
print(f"loops recorded: {len(m.get('loops',[]))}")
for l in m.get("loops",[]): print("  ", l["detail"])
print(f"mapper full rebuilds: {len(full)}", [f"gr{r['graph_revision']}/in{r['input_scan_count']}" for r in full])
cor=list(csv.DictReader(open(f"{d}/corrected_trajectory.csv")))
print(f"corrected trajectory spans bag+0..{float(cor[-1]['t'])-float(cor[0]['t']):.1f}s (baseline ended ~42s)")
EOF
```
Expected (success criteria): mean/max ORB lap-diff substantially below the 0.290/0.607 baseline; >=1 mapper full rebuild triggered by the new revision; corrected trajectory now extends well past bag+42s into lap 2.

- [ ] **Step 5: Sanity-check for false closures / map corruption**

Run:
```bash
grep -aE 'MAP_MERGED|MAP_RESET|cross_map_loop' 'artifacts/circle-loop-revisit-idgap-20260720.launch.log' | sed -E 's/^\[orb_slam3_wrapper-3\] //' | head
python3 - <<'EOF'
import json, csv, math
d="artifacts/circle-loop-revisit-idgap-20260720"
cor=list(csv.DictReader(open(f"{d}/corrected_trajectory.csv")))
jumps=0
for a,b in zip(cor, cor[1:]):
    if math.hypot(float(b["x"])-float(a["x"]), float(b["y"])-float(a["y"]))>1.0:
        jumps+=1
print(f"corrected-trajectory consecutive jumps >1.0m: {jumps} (expect 0; a loop-corrected path should be continuous)")
EOF
```
Expected: no `MAP_MERGED`/`MAP_RESET`/`cross_map_loop` (all closures should stay `same_map_loop`), and 0 large trajectory jumps. If present, the relaxation introduced spurious closures — STOP and report.

- [ ] **Step 6: Revert the temporary diagnostic flag**

In `orb_slam_bringup/launch/bag_replay.launch.py`, remove the `"-p", "log_keyframe_drift:=true",` line added in Step 1, then rebuild:
```bash
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; colcon build --packages-select orb_slam_bringup'
```

- [ ] **Step 7: Confirm launch file is back to committed state**

Run:
```bash
git diff --stat orb_slam_bringup/launch/bag_replay.launch.py
```
Expected: no output (file unchanged vs committed).

- [ ] **Step 8: Record the validation result**

Summarize in the final report: loop closures during lap 2 (count + ids), lap-diff before/after, mapper rebuild count, corrected-trajectory span, and the false-closure sanity checks. No commit in this task (validation only; artifacts are generated output).

---

## Notes for the executor

- The submodule is on a detached HEAD by design (it was at `601bec2` when this plan was written). Committing on detached HEAD is expected here; do not create/switch branches unless asked. Record the new submodule commit hashes in the task reports.
- If Task 3 shows the fix did not resolve the double wall, or introduced false closures, STOP and report with evidence — do not iterate on the constant or add more relaxations without discussion (avoid a tuning-thrash loop).
- The read-only `log_keyframe_drift` diagnostic already exists in the wrapper (default off); Task 3 only toggles it via launch for the validation run.
