#!/usr/bin/env bash
# Post-regression-fix validation: 3 replays at rate 2.0 and 3 at rate 3.0.
# Confirms no crash/hang/deadlock and that map geometry stays sane after the
# merge-edge-frame (R1) and recovery-gating (R2) corrections.
set -u

REPO=/home/duc/orb_slam3_ros2
BAG=/home/duc/bag/full-run-2307
OUTROOT="$REPO/tools/full-stack-report/regfix-validate-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUTROOT"
SUMMARY="$OUTROOT/summary.csv"
PROGRESS="$OUTROOT/progress.log"
PER_RUN_TIMEOUT=480

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$PROGRESS"; }
wrapper_cores() { coredumpctl list --no-pager 2>/dev/null | grep -c orb_slam3_wrapper; }

BASELINE_CORES="$(wrapper_cores)"
PREV_CORES=0
log "vendor=$(git -C "$REPO/orb_slam3_vendor/vendor/ORB_SLAM3" rev-parse --short HEAD) parent=$(git -C "$REPO" rev-parse --short HEAD)"
log "baseline_wrapper_coredumps=$BASELINE_CORES"
log "output=$OUTROOT"

echo "rate,iter,domain,exit,elapsed_s,timed_out,new_coredumps,initialized,ok_ratio,loop_count,map_rev_max,deadlock,invalid_poses,pgm_yaml_match,map_wxh,verdict" > "$SUMMARY"

DOMAIN=190
for RATE in 2.0 3.0; do
  for ITER in 1 2 3; do
    OUT="$OUTROOT/rate${RATE}-run${ITER}-domain${DOMAIN}"
    mkdir -p "$OUT"
    log "START rate=$RATE iter=$ITER domain=$DOMAIN"
    START=$(date +%s)
    timeout --signal=SIGINT --kill-after=60 "$PER_RUN_TIMEOUT" \
      bash "$REPO/tools/run_full_stack_dashboard.sh" \
        --bag "$BAG" --output "$OUT" --rate "$RATE" --domain "$DOMAIN" \
        --no-dashboard --no-report-check > "$OUT.log" 2>&1
    STATUS=$?
    ELAPSED=$(( $(date +%s) - START ))
    TIMED_OUT=0
    if [ "$STATUS" -eq 124 ] || [ "$STATUS" -eq 137 ]; then TIMED_OUT=1; fi

    pkill -f "bag_replay.launch.py" 2>/dev/null
    pkill -f "orb_slam3_wrapper_node" 2>/dev/null
    pkill -f "orb_lidar_mapper" 2>/dev/null
    pkill -f "metrics_recorder" 2>/dev/null
    sleep 6

    NEW_CORES=$(( $(wrapper_cores) - BASELINE_CORES ))
    # Per-run delta, not cumulative: a crash in an earlier run must not mark
    # later clean runs as unstable.
    RUN_CORES=$(( NEW_CORES - PREV_CORES ))
    PREV_CORES=$NEW_CORES
    ROW="$(python3 - "$OUT/metrics.json" "$RATE" "$ITER" "$DOMAIN" "$STATUS" "$ELAPSED" "$TIMED_OUT" "$RUN_CORES" <<'PY'
import json,sys
mpath,rate,it,dom,status,elapsed,timed_out,new_cores=sys.argv[1:9]
initialized=ok=loops=maprev=dead=invalid=pgm=wxh="NA"
try:
    d=json.load(open(mpath)); t=d.get("tracking",{})
    initialized=t.get("initialized"); ok=t.get("ok_ratio_after_init")
    loops=t.get("loop_count"); dead=t.get("deadlock"); invalid=t.get("invalid_poses")
    mrs=d.get("map_revisions") or []
    maprev=max([m.get("map_revision",0) for m in mrs], default=0)
    fm=d.get("final_map") or {}
    pgm=fm.get("pgm_yaml_match")
    if fm.get("width") is not None: wxh=f"{fm.get('width')}x{fm.get('height')}"
except Exception:
    pass
verdict="STABLE"; reasons=[]
if str(timed_out)=="1": reasons.append("HANG")
if int(new_cores)>0: reasons.append("COREDUMP")
if status!="0": reasons.append(f"exit{status}")
if dead is True: reasons.append("DEADLOCK")
if initialized is not True: reasons.append("no-init")
if pgm is not True: reasons.append("no-map")
if reasons: verdict="UNSTABLE:"+"|".join(reasons)
print(f"{rate},{it},{dom},{status},{elapsed},{timed_out},{new_cores},{initialized},{ok},{loops},{maprev},{dead},{invalid},{pgm},{wxh},{verdict}")
PY
)"
    echo "$ROW" >> "$SUMMARY"
    log "DONE  rate=$RATE iter=$ITER exit=$STATUS elapsed=${ELAPSED}s run_cores=$RUN_CORES -> ${ROW##*,}"
    DOMAIN=$((DOMAIN+1))
  done
done

log "ALL RUNS COMPLETE"
echo "=== SUMMARY ===" | tee -a "$PROGRESS"
column -s, -t "$SUMMARY" | tee -a "$PROGRESS"
