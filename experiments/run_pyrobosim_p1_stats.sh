#!/usr/bin/env bash
set -Eeuo pipefail

# Sweep ordering for PyRoboSim P1's 4S/4F cells only (2 fixed + --random-seeds
# random, threshold_250 only -- same shape as p0 --stats), to get real
# dispersion data instead of the n=1 single-alphabetic-ordering directional
# read the pyrobosim_p1 main-track task leaves for every P1 row.
#
# 4SP/4FP and FullP1 are deliberately left alone: 4SP/4FP's own single-trial
# baseline (~3875 s / ~10,866 s) already flags high blowup risk under an
# unfavorable ordering, and FullP1 never resolves its well-separation check
# at all (see run_pyrobosim_p1_outliers.sh) -- sweeping either would just
# multiply a known-expensive or known-timeout row 22x for uncertain payoff.
# 4S/4F are cheap by comparison (~219 s / ~31 s on alphabetic), so even a
# 20-50x blowup under a bad ordering lands around 1-3h for a single trial,
# not a multi-hour cap-hit -- floor estimate for the whole sweep (44 rows)
# is ~1.5h, budget up to ~15h for real margin given every other ordering
# treatment in this codebase has eventually turned up at least one outlier.
#
# Rows land in the same pyrobosim_p1_default run directory as the main-track
# task and run_pyrobosim_p1_outliers.sh, via --append -- summary.csv then
# reflects 4S/4F's original single alphabetic/auto-reorder trial (from the
# main track) plus this sweep's 22 threshold_250 treatments each, as
# distinct data points (different reordering_threshold), not duplicates.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "$REPO_ROOT/../.." && pwd)}"
SETUP_FILE="${SETUP_FILE:-$WORKSPACE_ROOT/install/setup.bash}"
SYNTHESIS_HOME="${FLEXBE_SYNTHESIS_HOME:-$HOME/.flexbe_synthesis}"
RUN_LOG="$SYNTHESIS_HOME/run_logs/pyrobosim_p1_stats.log"
SYNTHESIS_TIMEOUT_S="${SYNTHESIS_TIMEOUT_S:-28800}"              # 8h -- same cap as the main-track fast rows
WELL_SEPARATION_TIMEOUT_S="${WELL_SEPARATION_TIMEOUT_S:-28800}"  # 8h
RANDOM_SEEDS="${RANDOM_SEEDS:-20}"

PYROBOSIM_SYNTHESIS_PKG="$WORKSPACE_ROOT/src/flexbe_synthesis_demo/pyrobosim_demo/pyrobosim_flexbe_synthesis"
PYROBOSIM_EXP="$SYNTHESIS_HOME/experiments/pyrobosim"
MATRIX_FILE="$PYROBOSIM_EXP/matrices/pyrobosim_p1_stats.yaml"
OUT_DIR="$PYROBOSIM_EXP/runs/pyrobosim_p1_default"

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

log "Building 4S/4F ordering-sweep matrix (--random-seeds $RANDOM_SEEDS)"
cd "$PYROBOSIM_SYNTHESIS_PKG"
python3 - "$MATRIX_FILE" "$RANDOM_SEEDS" <<'PYEOF'
import sys
import yaml
from pyrobosim_flexbe_synthesis.pyrobosim_matrix import build_p1_matrix

out_path, random_seeds = sys.argv[1], int(sys.argv[2])
rows = [
    r for r in build_p1_matrix(stats=True, random_seeds=random_seeds, n_trials=1)['rows']
    if r.get('column_indicator') in ('4S', '4F')
]
expected = 2 * (2 + random_seeds)
if len(rows) != expected:
    raise SystemExit(f"Expected {expected} row(s) (4S+4F swept), got {len(rows)}.")
with open(out_path, 'w', encoding='utf-8') as f:
    yaml.safe_dump({'rows': rows}, f, sort_keys=False)
print(f"Wrote {len(rows)} row(s) to '{out_path}' (4S/4F x {2 + random_seeds} ordering treatments each)")
PYEOF

if [[ -f "$OUT_DIR/master.csv" ]]; then
  log "Stripping any stale 4S/4F threshold_250 rows from $OUT_DIR/master.csv before appending"
  python3 - "$OUT_DIR/master.csv" <<'PYEOF'
import csv
import sys

master_path = sys.argv[1]
with open(master_path, newline='', encoding='utf-8') as f:
    reader = csv.DictReader(f)
    fieldnames = reader.fieldnames
    all_rows = list(reader)


def is_prior_stats_row(row):
    return (
        row.get('column_indicator') in ('4S', '4F')
        and row.get('reordering_policy') == 'threshold_250'
    )


rows = [row for row in all_rows if not is_prior_stats_row(row)]
dropped = len(all_rows) - len(rows)
with open(master_path, 'w', newline='', encoding='utf-8') as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows)
print(f"master.csv: dropped {dropped} stale 4S/4F threshold_250 row(s), {len(rows)} row(s) remain")
PYEOF
  APPEND_FLAG=(--append)
else
  log "No existing run at $OUT_DIR -- run the pyrobosim_p1 main-track task first."
  exit 1
fi

log "Running 4S/4F sweep at synthesis-timeout-s=$SYNTHESIS_TIMEOUT_S well-separation-timeout-s=$WELL_SEPARATION_TIMEOUT_S"
run_cmd python3 -m flexbe_synthesis_slugs.helpers.batch_run_harness \
  --matrix "$MATRIX_FILE" \
  --out-dir "$OUT_DIR" \
  --state-mappings "$PYROBOSIM_SYNTHESIS_PKG/mappings/batch_mappings.yaml" \
  --slugs-bin /usr/local/bin/slugs \
  --synthesis-timeout-s "$SYNTHESIS_TIMEOUT_S" \
  --well-separation-timeout-s "$WELL_SEPARATION_TIMEOUT_S" \
  "${APPEND_FLAG[@]}"

# Unlike the rest of pyrobosim_p1 (single fixed alphabetic/auto ordering),
# 4S/4F's swept rows use experiment_postprocess table writer's own defaults (threshold_250 /
# random), same as P0's --stats convention -- no --table-policy/--table-ordering
# override needed here.
run_cmd python3 -m flexbe_synthesis_slugs.helpers.experiment_postprocess \
  --run-dir "$OUT_DIR" \
  --refresh-summary \
  --summary-plot

log "COMPLETE pyrobosim_p1 stats sweep (4S/4F)"
