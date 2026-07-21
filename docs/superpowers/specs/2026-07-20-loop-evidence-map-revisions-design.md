# Loop Evidence Map Revisions Design

## Goal

Correct the loop-closure acceptance evaluator so it recognizes a map rebuild
recorded by the metrics recorder, without weakening the requirement that a
published map exists at or after each observed loop event.

## Root Cause

`loop_closure_evidence.evaluate_artifact()` reads published rebuild records
from `metrics["revisions"]`. The metrics recorder writes those records under
`metrics["map_revisions"]`. Consequently, an artifact can contain a
`LOOP_CLOSED` event and a qualifying `PUBLISHED` map revision but is always
classified as `loop_rebuild_missing`.

The rate-one `circle-run` replay reproduced this in all three runs. Each run
recorded a loop at graph revision 2 and a later published map at graph revision
3.

## Scope

Change only:

- `orb_slam_bringup/orb_slam_bringup/loop_closure_evidence.py`
- `orb_slam_bringup/test/test_loop_closure_evidence.py`

Do not change the metrics recorder schema, ROS messages, wrapper, mapper,
vendor code, camera or ORB parameters, replay launcher, or acceptance
threshold.

## Behavior

1. The evaluator reads rebuild records from `metrics.get("map_revisions", [])`.
2. A loop with `graph_revision = N` is rebuilt only if at least one map revision
   has `state == "PUBLISHED"` and `graph_revision >= N`.
3. A `BUILDING`, `IDLE`, or `FAILED` record does not qualify.
4. Missing `map_revisions` is treated as an empty collection and preserves the
   existing `loop_rebuild_missing` diagnosis for an observed loop.
5. Existing diagnosis ordering and all non-rebuild evaluator behavior remain
   unchanged.

## Tests

Update the test metrics fixture to write `map_revisions`.

Cover these cases:

- An observed loop with a same-or-later `PUBLISHED` map revision produces
  `observed_and_rebuilt`.
- An observed loop with only a prior `PUBLISHED` map revision produces
  `loop_rebuild_missing`.
- A later `BUILDING` record without a qualifying `PUBLISHED` record produces
  `loop_rebuild_missing`.
- Existing core-loop, missing-log, diagnosis-order, runner-contract, and
  summary cases retain their behavior.

## Verification

Run the focused Python evaluator tests in the Nix development shell. Have an
independent subagent review the implementation and test evidence before
executing the unchanged three-run, rate-one `circle-run` evaluator. The final
acceptance result is the generated `summary.json`, which requires at least two
passing runs.
