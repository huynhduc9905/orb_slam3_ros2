# Loop Evidence Map Revisions Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the loop-closure evaluator recognize qualifying published map rebuilds recorded in `metrics["map_revisions"]`.

**Architecture:** The metrics recorder's established schema is authoritative. The evaluator reads only `map_revisions` when matching each observed loop graph revision to a published map graph revision. The fixture helper and focused unit tests use that same schema, including a non-published later revision that must not qualify.

**Tech Stack:** Python 3, pytest, ROS 2 ament_python package, Nix development shell.

## Global Constraints

- Modify only `orb_slam_bringup/orb_slam_bringup/loop_closure_evidence.py` and `orb_slam_bringup/test/test_loop_closure_evidence.py`.
- Do not change the metrics recorder schema, ROS messages, wrapper, mapper, vendor code, camera or ORB parameters, replay launcher, or acceptance threshold.
- Read map rebuild records with `metrics.get("map_revisions", [])`; do not support the stale `revisions` key.
- A qualifying rebuild has `state == "PUBLISHED"` and `graph_revision >= loop["graph_revision"]`.
- `BUILDING`, `IDLE`, and `FAILED` records do not qualify.
- Preserve all existing diagnosis ordering and non-rebuild behavior.
- Run all build and test commands through `nix develop --command bash -lc` with `CMAKE_BUILD_PARALLEL_LEVEL=32` exported.
- Preserve unrelated worktree changes and generated `artifacts/`.
- Before acceptance replay, obtain an independent subagent review of the implementation and test evidence.

---

### Task 1: Correct Evaluator Schema Contract

**Files:**
- Modify: `orb_slam_bringup/orb_slam_bringup/loop_closure_evidence.py:52-66`
- Modify: `orb_slam_bringup/test/test_loop_closure_evidence.py:4-99`

**Interfaces:**
- Consumes: artifact `metrics.json` with `loops: list[dict]` and `map_revisions: list[dict]`.
- Produces: `evaluate_artifact(artifact_dir: Path) -> dict`, where an observed loop is rebuilt only if a qualifying `map_revisions` entry exists.

- [ ] **Step 1: Change the test fixture to the recorder's schema and add the non-published regression**

Replace the helper's `revisions` argument and output with `map_revisions`:

```python
def write_metrics(path: Path, loops=None, map_revisions=None,
                  initialized=True, ok_ratio=0.9, deadlock=False):
    metrics = {
        "is_initialized": initialized,
        "is_deadlocked": deadlock,
        "tracking_ok_ratio": ok_ratio,
        "loops": loops if loops is not None else [],
        "map_revisions": map_revisions if map_revisions is not None else [],
    }
    with open(path / "metrics.json", "w") as f:
        json.dump(metrics, f)
```

Update every existing `write_metrics(..., revisions=...)` call to use
`map_revisions=...`. Keep the successful observed-loop case with a
`PUBLISHED` map revision at the loop revision. Keep the missing-rebuild case
with a prior `PUBLISHED` map revision. Add this test directly after it:

```python
def test_observed_loop_with_only_building_rebuild_is_missing(tmp_path: Path):
    from orb_slam_bringup.loop_closure_evidence import evaluate_artifact

    write_metrics(
        tmp_path,
        loops=[{"graph_revision": 4}],
        map_revisions=[{"state": "BUILDING", "graph_revision": 5}],
    )
    (tmp_path / "orb_slam3_wrapper.log").write_text(
        "*Loop detected\nLocal Mapping STOP\nLocal Mapping RELEASE\n"
    )

    evidence = evaluate_artifact(tmp_path)

    assert evidence["diagnoses"] == ["loop_rebuild_missing"]
    assert evidence["passed"] is False
```

- [ ] **Step 2: Run the focused test set to verify the fixture schema change is RED**

Run:

```bash
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; python3 -m pytest orb_slam_bringup/test/test_loop_closure_evidence.py -q'
```

Expected: tests that previously used a qualifying `revisions` entry now fail
with `loop_rebuild_missing`, demonstrating that the evaluator still reads the
stale key. The new `BUILDING` test may already pass; it is the qualifying
published-rebuild test that must be RED.

- [ ] **Step 3: Read published rebuild records from `map_revisions`**

In `evaluate_artifact`, replace the stale key in the published-revision
comprehension with the recorder's key:

```python
published_revisions = [
    rev["graph_revision"]
    for rev in metrics.get("map_revisions", [])
    if rev.get("state") == "PUBLISHED"
]
```

Leave the loop iteration, `pub_rev >= loop_rev` comparison, diagnosis order,
and all other evaluator logic unchanged.

- [ ] **Step 4: Run focused tests to verify GREEN**

Run:

```bash
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; python3 -m pytest orb_slam_bringup/test/test_loop_closure_evidence.py -q'
```

Expected: all evaluator tests pass. In particular, the qualifying published
case returns `observed_and_rebuilt`, while the prior-published and
later-`BUILDING` cases return `loop_rebuild_missing`.

- [ ] **Step 5: Run package tests and static diff validation**

Run:

```bash
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; colcon test --packages-select orb_slam_bringup --event-handlers console_direct+ --output-on-failure'
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; colcon test-result --all --verbose'
git diff --check
```

Expected: `orb_slam_bringup` tests pass, `colcon test-result` reports no test
failures, and `git diff --check` has no output.

- [ ] **Step 6: Commit the task**

```bash
git add orb_slam_bringup/orb_slam_bringup/loop_closure_evidence.py \
  orb_slam_bringup/test/test_loop_closure_evidence.py
git commit -m "fix: evaluate published map revisions"
```

### Task 2: Independent Review Gate

**Files:**
- Review: commit created by Task 1
- Review: `docs/superpowers/specs/2026-07-20-loop-evidence-map-revisions-design.md`
- Review: `docs/superpowers/plans/2026-07-20-loop-evidence-map-revisions.md`

**Interfaces:**
- Consumes: Task 1 commit, RED/GREEN command output, and source/test diff.
- Produces: a written review verdict before any acceptance replay.

- [ ] **Step 1: Dispatch a fresh independent review subagent**

Require the reviewer to confirm all of the following from the actual commit:

```text
1. The evaluator reads only metrics.get("map_revisions", []).
2. PUBLISHED is required and graph_revision >= loop revision is unchanged.
3. BUILDING, IDLE, and FAILED cannot satisfy rebuild evidence.
4. Missing map_revisions retains loop_rebuild_missing for observed loops.
5. All fixture calls use map_revisions and no stale revisions contract remains.
6. Scope is limited to the two approved Python files.
7. RED/GREEN and package test evidence is credible.
```

Save the detailed report to:

```text
.superpowers/sdd/task-1-loop-evidence-review.md
```

- [ ] **Step 2: Address review findings before replay**

If the review is rejected or reports an actionable defect, stop acceptance
replay, create a new TDD task for the defect, and obtain a fresh independent
review after the corrective commit. Do not modify the replay runner or alter
the acceptance rule to make a run pass.

### Task 3: Acceptance Replay Verification

**Files:**
- Generate: `artifacts/circle-loop-evidence-map-revisions-20260720/run-1/`
- Generate: `artifacts/circle-loop-evidence-map-revisions-20260720/run-2/`
- Generate: `artifacts/circle-loop-evidence-map-revisions-20260720/run-3/`
- Generate: `artifacts/circle-loop-evidence-map-revisions-20260720/summary.json`

**Interfaces:**
- Consumes: reviewed Task 1 implementation, `/home/duc/bag/circle-run`, and unchanged `tools/run_circle_loop_closure_evaluation.sh`.
- Produces: `summary.json` with `passed_runs >= 2` and per-run
`loop_closure_evidence.json` results.

- [ ] **Step 1: Run the unchanged three-run rate-one evaluator**

Run from the workspace root:

```bash
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; tools/run_circle_loop_closure_evaluation.sh --bag /home/duc/bag/circle-run --output artifacts/circle-loop-evidence-map-revisions-20260720 --domain 40'
```

Expected: the command exits 0 only if at least two of three runs pass. Each
run remains rate one with `benchmark_mode:=off` because the script is
unchanged.

- [ ] **Step 2: Inspect acceptance evidence**

Read:

```text
artifacts/circle-loop-evidence-map-revisions-20260720/summary.json
artifacts/circle-loop-evidence-map-revisions-20260720/run-1/loop_closure_evidence.json
artifacts/circle-loop-evidence-map-revisions-20260720/run-2/loop_closure_evidence.json
artifacts/circle-loop-evidence-map-revisions-20260720/run-3/loop_closure_evidence.json
```

Expected: at least two per-run evidence files contain:

```json
{"diagnoses": ["observed_and_rebuilt"], "passed": true}
```

and `summary.json` contains:

```json
{"passed_runs": 2, "required_passes": 2, "passed": true}
```

or a higher `passed_runs` value. If the replay fails, report the persisted
artifacts and diagnose from evidence before making another code change.
