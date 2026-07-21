# Wrapper Log Teardown Design

## Goal

Ensure a bag replay shuts down the ORB-SLAM3 wrapper and its log writer cleanly
when playback exits, while preserving append-only wrapper logs in the artifact
directory.

## Root Cause

`bag_replay.launch.py` starts the wrapper through Bash with a foreground
pipeline:

```bash
ros2 run ... 2>&1 | tee -a <wrapper-log>
```

Launch signals the Bash process during shutdown. Bash exits without forwarding
the signal to the pipeline children, leaving the `ros2 run` launcher, wrapper
node, and `tee` process alive. Three completed circle replays each leaked this
three-process group.

`ros2 run` forwards SIGINT and SIGTERM to its child executable. It only needs
to be the process directly managed by launch.

## Design

Keep the existing quoted wrapper argument construction and append-only log path.
Build the Bash command as:

```bash
exec ros2 run ... > >(tee -a '<artifact>/orb_slam3_wrapper.log') 2>&1
```

`exec` replaces Bash with `ros2 run`, so launch signals the `ros2 run` process
directly. `ros2 run` forwards the signal to `orb_slam3_wrapper_node`. The
process-substitution `tee` receives EOF after wrapper output closes and exits.

Do not use `pipefail`: there is no foreground pipeline whose exit status must
be propagated. Keep `shell=False` on `ExecuteProcess`.

## Scope

Change only:

- `orb_slam_bringup/launch/bag_replay.launch.py`
- `orb_slam_bringup/test/test_bag_profile.py`

Do not change the wrapper process, ROS messages, mapper, metrics schema,
evaluator, replay runner, camera or ORB parameters, or acceptance threshold.

## Required Behavior

1. Wrapper stdout and stderr remain merged and appended to
   `orb_slam3_wrapper.log` under the selected artifact directory.
2. Artifact paths containing spaces and shell metacharacters remain correctly
   quoted.
3. The ExecuteProcess command begins `bash -c`; its script begins `exec ros2
   run orb_slam3_wrapper orb_slam3_wrapper_node`.
4. The script uses output process substitution with `tee -a`; it does not use
   `| tee` or `-o pipefail`.
5. `ExecuteProcess.shell` remains false.
6. After a bag replay shuts down, no process remains whose command belongs to
   that replay's wrapper log artifact path or its wrapper node invocation.

## Tests And Verification

Update the launch-configuration unit test to verify the new command shape,
merged output, append-mode tee, secure quoting, no `pipefail`, no pipeline,
and `shell=False`.

Run focused `test_bag_profile.py`, then the `orb_slam_bringup` package test
suite. Run one rate-one circle replay to a new artifact directory and inspect
the process table after launch exits for wrapper and `tee` commands associated
with that artifact. Only after that check passes, run the unchanged three-run
rate-one acceptance evaluator and confirm at least two runs are
`observed_and_rebuilt`.

Require independent subagent review of the implementation and evidence before
the three-run replay, and a final branch review before completion.
