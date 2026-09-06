#!/usr/bin/env bash
set -Eeuo pipefail

# Retry drone_demo's two genuinely pathological cells: 1FP (one-hot) and 2FP
# (enumerated), both Fair-Outcome-with-pending. These do not reliably resolve
# within the drone_demo_fp main-track task's 28800s (8h) budget -- escalating
# single-cell attempts during development went 900s -> 7200s -> 18000s ->
# 36000s -> 57600s -> 86400s before 1FP finally resolved at ~35,413s (~9.8h)
# realizability; 2FP has run past 24h without a verdict in at least one
# attempt. This script is the standing, canonical place for that retry --
# supersedes the one-off run_drone_demo_enumerated_fp.sh (single-cell,
# 2FP-only) and the specific run_drone_demo_long_retry.sh invocation that also
# folded in a since-obviated 1SP well-separation retry (that root cause is now
# fixed at the source: drone_demo_stats' main-track task sets
# --well-separation-timeout-s explicitly instead of relying on the harness's
# 1800s implicit default).
#
# Only 1FP/2FP are retried here, single fixed alphabetic ordering (not swept),
# matching their existing --fp treatment. If a run still doesn't resolve at
# this timeout, that is itself a legitimate finding to report (a firm lower
# bound), not a signal to keep escalating indefinitely -- check in before
# raising SYNTHESIS_TIMEOUT_S past a day or two.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "$REPO_ROOT/../.." && pwd)}"
SETUP_FILE="${SETUP_FILE:-$WORKSPACE_ROOT/install/setup.bash}"
SYNTHESIS_HOME="${FLEXBE_SYNTHESIS_HOME:-$HOME/.flexbe_synthesis}"
RUN_LOG="$SYNTHESIS_HOME/run_logs/drone_demo_outliers.log"
SYNTHESIS_TIMEOUT_S="${SYNTHESIS_TIMEOUT_S:-86400}"              # 24h
WELL_SEPARATION_TIMEOUT_S="${WELL_SEPARATION_TIMEOUT_S:-21600}"  # 6h

DRONE_PKG="$WORKSPACE_ROOT/src/flexbe_synthesis_demo/drone_demo/drone_demo_flexbe_synthesis"
DRONE_EXP="$SYNTHESIS_HOME/experiments/drone_demo"
MATRIX_FILE="$DRONE_EXP/matrices/drone_demo_outliers.yaml"
OUT_DIR="$DRONE_EXP/runs/drone_demo_fp"

timestamp() {
  date '+%Y-%m-%d %H:%M:%S %z'
}

log() {
  mkdir -p "$(dirname "$RUN_LOG")"
  printf '[%s] %s\n' "$(timestamp)" "$*" | tee -a "$RUN_LOG"
}

run_cmd() {
  printf '\n[%s] $' "$(timestamp)" | tee -a "$RUN_LOG"
  printf ' %q' "$@" | tee -a "$RUN_LOG"
  printf '\n' | tee -a "$RUN_LOG"
  "$@" 2>&1 | tee -a "$RUN_LOG"
}

set +u
source "$SETUP_FILE"
set -u
export FLEXBE_SYNTHESIS_HOME="$SYNTHESIS_HOME"

mkdir -p "$DRONE_EXP/matrices"

log "Building FP matrix (1FP + 2FP, single alphabetic ordering)"
cd "$DRONE_PKG"
python3 - "$MATRIX_FILE" <<'PYEOF'
import sys
import yaml
from drone_demo_flexbe_synthesis.drone_demo_matrix import build_matrix

out_path = sys.argv[1]
rows = build_matrix(fp=True)['rows']
if len(rows) != 2:
    raise SystemExit(f"Expected 2 rows (1FP, 2FP) from build_matrix(fp=True), got {len(rows)}.")
with open(out_path, 'w', encoding='utf-8') as f:
    yaml.safe_dump({'rows': rows}, f, sort_keys=False)
for r in rows:
    print(f"  {r['column_indicator']}: {r['system_name']} / {r['encoding']}")
print(f"Wrote {len(rows)} row(s) to '{out_path}'")
PYEOF

if [[ -f "$OUT_DIR/master.csv" ]]; then
  log "Stripping stale 1FP/2FP rows from $OUT_DIR/master.csv before appending"
  python3 - "$OUT_DIR/master.csv" 1FP 2FP <<'PYEOF'
import csv
import sys

master_path = sys.argv[1]
drop = set(sys.argv[2:])
with open(master_path, newline='', encoding='utf-8') as f:
    reader = csv.DictReader(f)
    fieldnames = reader.fieldnames
    all_rows = list(reader)
rows = [row for row in all_rows if row.get('column_indicator') not in drop]
dropped = len(all_rows) - len(rows)
with open(master_path, 'w', newline='', encoding='utf-8') as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows)
print(f"master.csv: dropped {dropped} stale row(s) matching {sorted(drop)}, {len(rows)} row(s) remain")
PYEOF
  APPEND_FLAG=(--append)
else
  log "No existing run at $OUT_DIR -- starting fresh (no --append)"
  APPEND_FLAG=()
fi

log "Running 1FP/2FP at synthesis-timeout-s=$SYNTHESIS_TIMEOUT_S well-separation-timeout-s=$WELL_SEPARATION_TIMEOUT_S"
run_cmd python3 -m flexbe_synthesis_slugs.helpers.batch_run_harness \
  --matrix "$MATRIX_FILE" \
  --out-dir "$OUT_DIR" \
  --state-mappings "$DRONE_PKG/mappings/batch_mappings.yaml" \
  --slugs-bin /usr/local/bin/slugs \
  --synthesis-timeout-s "$SYNTHESIS_TIMEOUT_S" \
  --well-separation-timeout-s "$WELL_SEPARATION_TIMEOUT_S" \
  "${APPEND_FLAG[@]}"

# drone_demo_fp's rows are alphabetic-ordering / threshold_250 -- experiment_postprocess table writer's
# default --table-ordering ('random') would match zero rows here.
run_cmd python3 -m flexbe_synthesis_slugs.helpers.experiment_postprocess \
  --run-dir "$OUT_DIR" \
  --refresh-summary \
  --table-policy threshold_250 \
  --table-ordering alphabetic

log "COMPLETE drone_demo outliers (1FP, 2FP)"
