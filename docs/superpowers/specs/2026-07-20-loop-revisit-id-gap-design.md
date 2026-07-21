# Loop-Closure Revisit via Keyframe-ID Gap (ORB-SLAM3 fork)

Date: 2026-07-20
Status: Approved for planning
Scope: vendored fork `orb_slam3_vendor/vendor/ORB_SLAM3` only

## Problem

On the circle bag the robot drives the same physical loop **twice**. ORB-SLAM3
fires loop closure exactly once, near the end of lap 1 (bag+~47s). Lap 2 (a full
re-drive of the same path) never loop-closes, so its accumulated drift is never
corrected. Measured lap-1 vs lap-2 ORB pose difference at the same physical
location: **mean 0.29 m, max 0.61 m**. The lidar mapper faithfully places lap-2
scans at those drifted poses, drawing every wall twice (~0.3 m apart): the
"double wall".

## Confirmed root cause (verified in source)

Loop-candidate selection excludes any keyframe that is **covisibility-connected**
to the current keyframe. On lap 2, re-observing lap-1 features creates
covisibility edges to the lap-1 keyframes, so they are excluded from ever
becoming loop candidates. Two gates enforce this, both purely covisibility-based
with **no temporal / keyframe-id component**:

1. `src/KeyFrameDatabase.cc:626` in `DetectNBestCandidates`:
   ```cpp
   spConnectedKF = pKF->GetConnectedKeyFrames();
   ...
   if(!spConnectedKF.count(pKFi))   // covisible candidates dropped here
   ```
2. `src/LoopClosing.cc:628-641` in `DetectCommonRegionsFromBoW`:
   ```cpp
   set<KeyFrame*> spConnectedKeyFrames = mpCurrentKF->GetConnectedKeyFrames();
   ...
   if(spConnectedKeyFrames.find(vpCovKFi[j]) != spConnectedKeyFrames.end()) {
       bAbortByNearKF = true;   // candidate aborted if it neighbors a connected KF
       break;
   }
   ```

Because the exclusion is purely covisibility-based, it cannot distinguish a
just-created forward-motion neighbor (should stay excluded) from an old area
being re-driven on a later lap (should be an eligible loop candidate).

## Design: keyframe-ID gap gate

Change both exclusion gates so a covisibility-connected keyframe is only
excluded when it is **temporally close** to the current keyframe, measured by
keyframe-ID gap. Distant covisible keyframes (genuine revisits) become eligible
loop candidates again. Everything downstream — BoW similarity ranking, the
`>=3` (temporal path) / `>=3` (BoW path) coincidence requirement, Sim3
estimation and inlier gates — is left unchanged, so geometric verification still
guards against false closures.

### Hardcoded constant

```cpp
// Minimum keyframe-id gap for a covisibility-connected keyframe to still be
// eligible as a loop-closure candidate. Below this gap the connected keyframe
// is treated as an immediate trajectory neighbor and excluded (original
// behavior). At or above it, the keyframe is treated as a genuine revisit of a
// previously-mapped area (e.g. a later lap of the same loop) and allowed.
static const unsigned long kLoopRevisitMinKFGap = 20;
```

Rationale for 20 (from circle-bag data): lap 1 ~80 keyframes; immediate
forward-motion covisibility neighbors span only the last ~10-15 keyframes; on
lap 2 the current KF id is ~100-127 while revisited lap-1 keyframes are id
~5-80 (gap 40-95). A gap of 20 sits comfortably between the neighbor band
(<~15) and the revisit band (>~40).

### Gate 1 — `KeyFrameDatabase::DetectNBestCandidates` (KeyFrameDatabase.cc:626)

Replace the unconditional connected-exclusion with an id-gap-qualified one:

```cpp
const bool connected = spConnectedKF.count(pKFi) != 0;
const bool recent =
    (pKF->mnId >= pKFi->mnId) &&
    (pKF->mnId - pKFi->mnId < kLoopRevisitMinKFGap);
if(!(connected && recent))
{
    pKFi->mnPlaceRecognitionQuery=pKF->mnId;
    lKFsSharingWords.push_back(pKFi);
}
```

Effect: a connected keyframe with a large id gap is no longer suppressed;
connected recent neighbors are still suppressed exactly as before.

### Gate 2 — `LoopClosing::DetectCommonRegionsFromBoW` (LoopClosing.cc:628-641)

Only abort on a near covisibility neighbor when that neighbor is also recent by
id gap:

```cpp
bool bAbortByNearKF = false;
for(int j=0; j<vpCovKFi.size(); ++j)
{
    KeyFrame* pCov = vpCovKFi[j];
    if(pCov && spConnectedKeyFrames.find(pCov) != spConnectedKeyFrames.end() &&
       mpCurrentKF->mnId >= pCov->mnId &&
       mpCurrentKF->mnId - pCov->mnId < kLoopRevisitMinKFGap)
    {
        bAbortByNearKF = true;
        break;
    }
}
```

Effect: consistent with Gate 1 — abort only for genuinely recent connected
neighbors, allow distant revisits through to Sim3 verification.

Both gates use the same `kLoopRevisitMinKFGap` so the two stages agree on what
"recent" means. `mnId` is monotonically increasing and `unsigned long`; the
`pKF->mnId >= pKFi->mnId` guard prevents unsigned underflow for the (normal)
case where candidates are older than the current keyframe.

## Non-goals / left unchanged

- Merge detection thresholds and merge exclusion logic (merges combine separate
  maps; out of scope and riskier).
- The `>=3` loop coincidence requirement (reverted earlier; not the lever).
- Any settings/YAML or ROS parameter plumbing (constant is hardcoded per user
  decision).
- Mapper (`orb_lidar_mapper`) code — no changes; it already rebuilds correctly
  when a higher graph revision arrives.
- Metrics schema, replay runner, acceptance thresholds.

## Risk

Relaxing the exclusion increases exposure to degenerate/false loop closures if a
robot loiters in place long enough to accumulate a >=20 id gap against a
still-covisible area. Mitigations already present and retained: BoW similarity
ranking, `>=3` consecutive geometric coincidences, Sim3 inlier thresholds
(`nBoWInliers=15`, `nSim3Inliers=20`, `nProjOptMatches=80`). Validation must
explicitly check for spurious closures / map corruption, not just "double wall
gone".

## Validation

Re-run the circle bag (rate 1, benchmark_mode off) with the read-only
`log_keyframe_drift` diagnostic enabled, and measure:

1. Loop closures now fire during lap 2 (>=1 additional `LOOP_CLOSED` with a
   large from/to keyframe-id gap; graph revision advances past 3).
2. Lap-1 vs lap-2 ORB pose difference at the same physical location drops
   substantially from the 0.29 m mean / 0.61 m max baseline.
3. A mapper full rebuild fires on the new revision and the corrected trajectory
   now extends across lap 2 (not just to bag+42s).
4. No spurious loop closures / no map fragmentation or teleport (sanity-check
   trajectory continuity and event stream).
5. Wrapper package tests still pass; existing vendor loop-closure notification
   test still passes.

Commit the fork change inside the submodule `orb_slam3_vendor/vendor/ORB_SLAM3`
(the vendor package builds from committed submodule state; a clean
`build/`+`install/` vendor rebuild is required to pick it up).
