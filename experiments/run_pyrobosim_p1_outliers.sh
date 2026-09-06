#!/usr/bin/env bash
set -Eeuo pipefail

# Retry the one PyRoboSim P1 cell that has never resolved: FullP1's
# well-separation check. Two prior attempts (7200s implicit default, then a
# dedicated 21600s/6h retry -- see git history for retry_p1_timeouts.sh, now
# retired in favor of this script) both hit ANALYSIS_INCOMPLETE ("TIME OUT").
# FullP1's realizability itself resolves fine (~3900s); it's specifically the
# well-separation analyzer that never finishes.
#
# This may legitimately never produce a verdict -- the point of a longer
# timeout here is to report a firmer bound ("exceeds 43200s") if it still
# doesn't finish, not to chase a result indefinitely. Do not keep escalating
# this script's timeout past a few attempts; if 12h doesn't resolve it,
# that's itself the finding.
#
# There is no way to retry only the well-separation step -- batch_run_harness
# reruns a matrix row end to end, so this redoes FullP1's ~3900s realizability
# too. The one-hot P1 row (1S) is NOT handled here: it already resolved
# (~23,307s, well within a 12h budget) during exploratory testing and is
# folded into the pyrobosim_p1 main-track task's own matrix -- no separate
# retry needed for it.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "$REPO_ROOT/../.." && pwd)}"
SETUP_FILE="${SETUP_FILE:-$WORKSPACE_ROOT/install/setup.bash}"
SYNTHESIS_HOME="${FLEXBE_SYNTHESIS_HOME:-$HOME/.flexbe_synthesis}"
RUN_LOG="$SYNTHESIS_HOME/run_logs/pyrobosim_p1_outliers.log"
SYNTHESIS_TIMEOUT_S="${SYNTHESIS_TIMEOUT_S:-43200}"              # 12h
WELL_SEPARATION_TIMEOUT_S="${WELL_SEPARATION_TIMEOUT_S:-43200}"  # 12h

PYROBOSIM_SYNTHESIS_PKG="$WORKSPACE_ROOT/src/flexbe_synthesis_demo/pyrobosim_demo/pyrobosim_flexbe_synthesis"
PYROBOSIM_EXP="$SYNTHESIS_HOME/experiments/pyrobosim"
MATRIX_FILE="$PYROBOSIM_EXP/matrices/pyrobosim_p1_outliers.yaml"
OUT_DIR="$PYROBOSIM_EXP/runs/pyrobosim_p1_default"
TARGET_INDICATOR="FullP1"

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

mkdir -p "$PYROBOSIM_EXP/matrices"

log "Building single-row matrix ($TARGET_INDICATOR only)"
cd "$PYROBOSIM_SYNTHESIS_PKG"
python3 - "$MATRIX_FILE" "$TARGET_INDICATOR" <<'PYEOF'
import sys
import yaml
from pyrobosim_flexbe_synthesis.pyrobosim_matrix import build_p1_matrix

out_path, indicator = sys.argv[1:3]
rows = [
    r for r in build_p1_matrix(full=False, n_trials=1)['rows']
    if r.get('column_indicator') == indicator
]
if not rows:
    raise SystemExit(f'No P1 row with column_indicator={indicator!r} found.')
with open(out_path, 'w', encoding='utf-8') as f:
    yaml.safe_dump({'rows': rows}, f, sort_keys=False)
print(f"Wrote 1 row to '{out_path}': {rows[0]['system_name']} / {rows[0]['encoding']}")
PYEOF

if [[ -f "$OUT_DIR/master.csv" ]]; then
  log "Stripping any stale $TARGET_INDICATOR row from $OUT_DIR/master.csv before appending"
  python3 - "$OUT_DIR/master.csv" "$TARGET_INDICATOR" <<'PYEOF'
import csv
import sys

master_path, indicator = sys.argv[1:3]
with open(master_path, newline='', encoding='utf-8') as f:
    reader = csv.DictReader(f)
    fieldnames = reader.fieldnames
    all_rows = list(reader)
rows = [row for row in all_rows if row.get('column_indicator') != indicator]
dropped = len(all_rows) - len(rows)
with open(master_path, 'w', newline='', encoding='utf-8') as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows)
print(f"master.csv: dropped {dropped} stale {indicator!r} row(s), {len(rows)} row(s) remain")
PYEOF
  APPEND_FLAG=(--append)
else
  log "No existing run at $OUT_DIR -- starting fresh (no --append)"
  APPEND_FLAG=()
fi

log "Running $TARGET_INDICATOR with synthesis-timeout-s=$SYNTHESIS_TIMEOUT_S well-separation-timeout-s=$WELL_SEPARATION_TIMEOUT_S"
run_cmd python3 -m flexbe_synthesis_slugs.helpers.batch_run_harness \
  --matrix "$MATRIX_FILE" \
  --out-dir "$OUT_DIR" \
  --state-mappings "$PYROBOSIM_SYNTHESIS_PKG/mappings/batch_mappings.yaml" \
  --slugs-bin /usr/local/bin/slugs \
  --synthesis-timeout-s "$SYNTHESIS_TIMEOUT_S" \
  --well-separation-timeout-s "$WELL_SEPARATION_TIMEOUT_S" \
  "${APPEND_FLAG[@]}"

run_cmd python3 -m flexbe_synthesis_slugs.helpers.experiment_postprocess \
  --run-dir "$OUT_DIR" \
  --refresh-summary \
  --table-policy auto \
  --table-ordering alphabetic

log "COMPLETE pyrobosim_p1 outliers ($TARGET_INDICATOR)"
