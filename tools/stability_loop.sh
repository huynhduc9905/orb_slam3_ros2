#!/usr/bin/env bash
# Stability loop: replay the full stack 10x at rate 2.0 and 10x at rate 3.0.
# Records per-run stability signals (clean exit / hang / new coredump / deadlock)
# and tracking metrics, and compares against the previous baseline.
set -u

REPO=/home/duc/orb_slam3_ros2
BAG=/home/duc/bag/full-run-2307
STAMP="$(date +%Y%m%d-%H%M%S)"
OUTROOT="$REPO/tools/full-stack-report/stability-loop-$STAMP"
mkdir -p "$OUTROOT"
SUMMARY="$OUTROOT/summary.csv"
PROGRESS="$OUTROOT/progress.log"

# Per-run wall-clock ceiling. Bag is ~221s; 2x ~=110s, 3x ~=74s of playback plus
# startup/shutdown. A run exceeding this is treated as a HANG (the exact failure
# class H1 targeted). SIGINT first (graceful), SIGKILL 60s later.
PER_RUN_TIMEOUT=480

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$PROGRESS"; }

wrapper_cores() { coredumpctl list --no-pager 2>/dev/null | grep -c orb_slam3_wrapper; }

BASELINE_CORES="$(wrapper_cores)"
log "vendor=$(git -C "$REPO/orb_slam3_vendor/vendor/ORB_SLAM3" rev-parse --short HEAD) parent=$(git -C "$REPO" rev-parse --short HEAD)"
log "baseline_wrapper_coredumps=$BASELINE_CORES"
log "output=$OUTROOT"

echo "rate,iter,domain,exit,elapsed_s,timed_out,new_coredumps,initialized,ok_ratio,loop_count,map_rev_max,deadlock,invalid_poses,pgm_yaml_match,map_wxh,verdict" > "$SUMMARY"

DOMAIN=160
for RATE in 2.0 3.0; do
  for ITER in $(seq 1 10); do
    OUT="$OUTROOT/rate${RATE}-run${ITER}-domain${DOMAIN}"
    LOG="$OUT.log"
    mkdir -p "$OUT"
    log "START rate=$RATE iter=$ITER domain=$DOMAIN"
    START=$(date +%s)
    timeout --signal=SIGINT --kill-after=60 "$PER_RUN_TIMEOUT" \
      bash "$REPO/tools/run_full_stack_dashboard.sh" \
        --bag "$BAG" --output "$OUT" --rate "$RATE" --domain "$DOMAIN" \
        --no-dashboard --no-report-check > "$LOG" 2>&1
    STATUS=$?
    ELAPSED=$(( $(date +%s) - START ))
    TIMED_OUT=0
    if [ "$STATUS" -eq 124 ] || [ "$STATUS" -eq 137 ]; then TIMED_OUT=1; fi

    # Reap any stragglers so the next run starts clean.
    pkill -f "bag_replay.launch.py" 2>/dev/null
    pkill -f "orb_slam3_wrapper_node" 2>/dev/null
    pkill -f "orb_lidar_mapper" 2>/dev/null
    pkill -f "component_container" 2>/dev/null
    sleep 6

    CUR_CORES="$(wrapper_cores)"
    NEW_CORES=$(( CUR_CORES - BASELINE_CORES ))

    ROW="$(python3 - "$OUT/metrics.json" "$RATE" "$ITER" "$DOMAIN" "$STATUS" "$ELAPSED" "$TIMED_OUT" "$NEW_CORES" <<'PY'
import json,sys
mpath,rate,it,dom,status,elapsed,timed_out,new_cores=sys.argv[1:9]
initialized=ok=loops=maprev=dead=invalid=pgm=wxh="NA"
try:
    d=json.load(open(mpath))
    t=d.get("tracking",{})
    initialized=t.get("initialized")
    ok=t.get("ok_ratio_after_init")
    loops=t.get("loop_count")
    dead=t.get("deadlock")
    invalid=t.get("invalid_poses")
    mrs=d.get("map_revisions") or []
    maprev=max([m.get("map_revision",0) for m in mrs], default=0)
    fm=d.get("final_map") or {}
    pgm=fm.get("pgm_yaml_match")
    if fm.get("width") is not None:
        wxh=f"{fm.get('width')}x{fm.get('height')}"
except Exception as e:
    pass
# Stability verdict: clean exit, no hang, no new coredump, not deadlocked,
# initialized, and a final map produced.
verdict="STABLE"
reasons=[]
if str(timed_out)=="1": reasons.append("HANG")
if int(new_cores)>0: reasons.append("COREDUMP")
if status not in ("0",): reasons.append(f"exit{status}")
if dead is True: reasons.append("DEADLOCK")
if initialized is not True: reasons.append("no-init")
if pgm is not True: reasons.append("no-map")
if reasons: verdict="UNSTABLE:"+"|".join(reasons)
print(f"{rate},{it},{dom},{status},{elapsed},{timed_out},{new_cores},{initialized},{ok},{loops},{maprev},{dead},{invalid},{pgm},{wxh},{verdict}")
PY
)"
    echo "$ROW" >> "$SUMMARY"
    log "DONE  rate=$RATE iter=$ITER exit=$STATUS elapsed=${ELAPSED}s new_cores=$NEW_CORES -> ${ROW##*,}"
    DOMAIN=$((DOMAIN+1))
  done
done

log "ALL RUNS COMPLETE"
log "baseline_wrapper_coredumps=$BASELINE_CORES final_wrapper_coredumps=$(wrapper_cores)"
echo "=== SUMMARY ===" | tee -a "$PROGRESS"
column -s, -t "$SUMMARY" | tee -a "$PROGRESS"
