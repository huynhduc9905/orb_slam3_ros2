# Wrapper Log Teardown Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make bag-replay shutdown terminate the ORB-SLAM3 wrapper and log writer while retaining append-only wrapper logs.

**Architecture:** Replace the foreground Bash pipeline with an `exec`-ed `ros2 run` command and a `tee` process substitution. Launch then signals `ros2 run` directly; it forwards signals to the wrapper node, while `tee` exits after EOF. The existing launch test validates the command shape and shell-safe log-path quoting.

**Tech Stack:** Python 3, ROS 2 launch, pytest, Bash process substitution, Nix development shell.

## Global Constraints

- Modify only `orb_slam_bringup/launch/bag_replay.launch.py` and `orb_slam_bringup/test/test_bag_profile.py`.
- Preserve existing quoted wrapper arguments, merged stdout/stderr, append-only `orb_slam3_wrapper.log`, and `ExecuteProcess(shell=False)`.
- Use `exec` before the quoted `ros2 run` command and `> >(tee -a <quoted-log-path>) 2>&1` for logging.
- Do not use `-o pipefail` or a foreground `| tee` pipeline.
- Do not change the wrapper process, ROS messages, mapper, metrics schema, evaluator, replay runner, camera or ORB parameters, or acceptance threshold.
- Run all test and replay commands through `nix develop --command bash -lc` with `CMAKE_BUILD_PARALLEL_LEVEL=32` exported.
- Preserve unrelated worktree changes and generated `artifacts/`.
- Before the three-run replay, obtain independent subagent review of the code and test evidence.

---

### Task 1: Make Wrapper Logging Teardown-Safe

**Files:**
- Modify: `orb_slam_bringup/launch/bag_replay.launch.py:286-301`
- Modify: `orb_slam_bringup/test/test_bag_profile.py:108-198`

**Interfaces:**
- Consumes: `wrapper_cmd_quoted: str` and `wrapper_log_path: str` constructed in `_setup()`.
- Produces: an `ExecuteProcess` whose `cmd` is `['bash', '-c', script_content]`; `script_content` runs the wrapper through `exec` and appends combined output through `tee` process substitution.

- [ ] **Step 1: Update the command-shape test before production code**

In `test_wrapper_log_capture_configuration`, replace the current Bash-command
and pipeline assertions with assertions that require the new contract:

```python
assert resolved_cmd[:2] == ["bash", "-c"]
script_content = resolved_cmd[2]

assert script_content.startswith("exec ros2 run orb_slam3_wrapper orb_slam3_wrapper_node ")
assert " > >(tee -a " in script_content
assert " 2>&1" in script_content
assert "| tee" not in script_content
assert "pipefail" not in resolved_cmd
assert "'/tmp/path with spaces and $meta/orb_slam3_wrapper.log'" in script_content

parsed_command = shlex.split(script_content.split(" > >(", 1)[0])
assert parsed_command[:5] == [
    "exec", "ros2", "run", "orb_slam3_wrapper", "orb_slam3_wrapper_node"
]
```

Keep the existing checks for `__node:=orb_slam3_wrapper`,
`use_sim_time:=true`, the quoted settings path, and `shell=False`. Remove the
old `pipefail` subprocess probe because the new command intentionally has no
foreground pipeline.

- [ ] **Step 2: Run the focused test to verify RED**

Run:

```bash
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; python3 -m pytest orb_slam_bringup/test/test_bag_profile.py::test_wrapper_log_capture_configuration -q'
```

Expected: FAIL because the current command begins `ros2 run`, uses `-o
pipefail`, and contains `| tee -a` rather than `exec` and output process
substitution.

- [ ] **Step 3: Replace the foreground pipeline with exec and process substitution**

In `_setup()`, replace the existing `script_content` and `ExecuteProcess.cmd`
construction with:

```python
script_content = (
    f"exec {wrapper_cmd_quoted} > >(tee -a {shlex.quote(wrapper_log_path)}) 2>&1"
)

actions.append(
    ExecuteProcess(
        cmd=["bash", "-c", script_content],
        name="orb_slam3_wrapper",
        output="screen",
        shell=False,
    )
)
```

Do not change `wrapper_cmd_parts`, `wrapper_cmd_quoted`, `wrapper_log_path`,
or any other launch action.

- [ ] **Step 4: Run focused and package tests to verify GREEN**

Run:

```bash
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; python3 -m pytest orb_slam_bringup/test/test_bag_profile.py::test_wrapper_log_capture_configuration -q'
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; colcon test --packages-select orb_slam_bringup --event-handlers console_direct+ --output-on-failure'
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; colcon test-result --all --verbose'
git diff --check
```

Expected: focused test passes, package tests have no failures, and
`git diff --check` has no output.

- [ ] **Step 5: Commit the implementation**

```bash
git add orb_slam_bringup/launch/bag_replay.launch.py \
  orb_slam_bringup/test/test_bag_profile.py
git commit -m "fix: terminate wrapper logging on replay shutdown"
```

### Task 2: Independent Review Gate

**Files:**
- Review: Task 1 commit
- Review: `docs/superpowers/specs/2026-07-20-wrapper-log-teardown-design.md`
- Review: `docs/superpowers/plans/2026-07-20-wrapper-log-teardown.md`

**Interfaces:**
- Consumes: Task 1 source/test diff and its RED/GREEN/package-test evidence.
- Produces: written review approval before runtime replay verification.

- [ ] **Step 1: Dispatch a fresh independent review subagent**

Require the reviewer to verify:

```text
1. The wrapper command is exec-ed and Launch directly manages the ros2 run process.
2. Combined stdout/stderr remains appended through tee process substitution.
3. The log path remains shlex-quoted and handles spaces and shell metacharacters.
4. No pipefail or foreground | tee pipeline remains.
5. shell=False remains set.
6. The two-file scope is respected.
7. The revised launch test would fail on the old command and credibly validates the new command.
8. Focused and package test evidence is complete.
```

Save the detailed review to:

```text
.superpowers/sdd/task-1-wrapper-log-teardown-review.md
```

- [ ] **Step 2: Resolve review findings before runtime verification**

If the review reports a Critical or Important finding, dispatch a fresh fix
subagent with the complete findings, rerun the focused test and
`orb_slam_bringup` package tests, then obtain a fresh independent review. Do
not run replay verification until the review approves both spec compliance and
task quality.

### Task 3: Runtime Leak And Acceptance Verification

**Files:**
- Generate: `artifacts/circle-wrapper-log-teardown-smoke-20260720/`
- Generate: `artifacts/circle-wrapper-log-teardown-20260720/run-1/`
- Generate: `artifacts/circle-wrapper-log-teardown-20260720/run-2/`
- Generate: `artifacts/circle-wrapper-log-teardown-20260720/run-3/`
- Generate: `artifacts/circle-wrapper-log-teardown-20260720/summary.json`

**Interfaces:**
- Consumes: approved Task 1 implementation, `/home/duc/bag/circle-run`, and unchanged `tools/run_circle_loop_closure_evaluation.sh`.
- Produces: a clean post-shutdown process check and acceptance `summary.json`
with at least two passing runs.

- [ ] **Step 1: Run a one-replay teardown smoke test**

Run:

```bash
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; ros2 launch orb_slam_bringup bag_replay.launch.py bag_path:=/home/duc/bag/circle-run artifact_dir:=artifacts/circle-wrapper-log-teardown-smoke-20260720 rate:=1 ros_domain_id:=50 start_dashboard:=false benchmark_mode:=off'
```

Expected: bag playback completes and launch shuts down.

- [ ] **Step 2: Prove that the smoke replay left no wrapper or tee process**

After the launch command has returned, run:

```bash
pgrep -af 'orb_slam3_wrapper_node|tee -a artifacts/circle-wrapper-log-teardown-smoke-20260720' || true
```

Expected: no matching output. If a matching process remains, stop and
investigate it before running the acceptance suite; do not kill it and claim
the teardown fix is verified.

- [ ] **Step 3: Run the unchanged three-run rate-one acceptance evaluator**

Run:

```bash
nix develop --command bash -lc 'export CMAKE_BUILD_PARALLEL_LEVEL=32; tools/run_circle_loop_closure_evaluation.sh --bag /home/duc/bag/circle-run --output artifacts/circle-wrapper-log-teardown-20260720 --domain 60'
```

Expected: the script exits zero only when at least two of three runs pass.

- [ ] **Step 4: Inspect persisted acceptance evidence**

Read:

```text
artifacts/circle-wrapper-log-teardown-20260720/summary.json
artifacts/circle-wrapper-log-teardown-20260720/run-1/loop_closure_evidence.json
artifacts/circle-wrapper-log-teardown-20260720/run-2/loop_closure_evidence.json
artifacts/circle-wrapper-log-teardown-20260720/run-3/loop_closure_evidence.json
```

Expected: `summary.json` has `passed: true` and `passed_runs >= 2`; at least
two run evidence files contain `diagnoses: ["observed_and_rebuilt"]` and
`passed: true`. If not, report the saved artifacts and investigate before
making another change.
