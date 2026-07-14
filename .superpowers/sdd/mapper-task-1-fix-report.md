# Mapper Task 1 Fix Report

## Changed files

- `orb_lidar_mapper/src/timed_pose_buffer.cpp`: reject negative duration configuration, replace equal newest timestamps, and compare ordered timestamp durations in `uint64_t`.
- `orb_lidar_mapper/test/timed_pose_buffer_test.cpp`: cover invalid configuration, duplicate replacement, exact max-gap acceptance, endpoints/out-of-range requests, and full-`int64_t` retention/gap cases.
- `orb_lidar_mapper/test/pose2_test.cpp`: cover regular and small-angle `exp`/`log` round trips and `pow` alpha clamping.

## RED evidence

Before production changes, the focused test command failed as expected:

- `TimedPoseBuffer.ReplacesNewestSampleWithEqualTimestamp`: size was `2`, expected `1`; lookup returned the original pose.
- `TimedPoseBuffer.RejectsNegativeConfiguration`: neither negative argument threw `std::invalid_argument`.
- `Pose2.ExpAndLogRoundTripThroughSmallAngleBranch`: the initial `1e-20` yaw tolerance was unrealistically strict for double precision (observed error `8.2740370962658176e-18`); it was corrected to `1e-16` while retaining small-angle coverage.

The full-range timestamp tests document the overflow boundary. The pre-fix signed subtractions were undefined when timestamps spanned the full domain, even though this particular non-sanitized run happened to produce the expected values.

## GREEN command and output

Command:

```bash
nix develop /home/duc/robot -c bash -lc 'cd /home/duc/robot/src/orb_slam3_ros2 && COLCON_DEFAULTS_FILE=/dev/null colcon build --packages-select orb_lidar_mapper --cmake-args -DBUILD_TESTING=ON && source install/setup.bash && COLCON_DEFAULTS_FILE=/dev/null colcon test --packages-select orb_lidar_mapper --event-handlers console_direct+ && COLCON_DEFAULTS_FILE=/dev/null colcon test-result --verbose'
```

Output:

```text
Summary: 1 package finished [0.53s]
100% tests passed, 0 tests failed out of 2
Summary: 95 tests, 0 errors, 0 failures, 0 skipped
```

## Commit hash

Implementation commit: `e48b3acdd762553da13e8f1f5b26963fa95fb0c2` (`fix: harden timed pose buffer boundaries`).

## Self-review

- All retention and interpolation duration comparisons use unsigned differences only after the timestamp ordering checks establish a non-negative mathematical interval.
- Negative retention and maximum-gap values fail at construction with `std::invalid_argument`.
- Equal newest timestamps replace the existing sample and return without growing the deque.
- Exact sample lookup remains accepted before gap validation; out-of-range requests remain rejected.
- `git diff --check` reported no whitespace errors before the implementation commit.

## Concerns

The Nix/colcon environment emits pre-existing `AMENT_PREFIX_PATH` warnings for store paths without `local_setup.*` files. They did not affect the focused build or test result.
