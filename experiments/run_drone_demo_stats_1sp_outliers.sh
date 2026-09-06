#!/usr/bin/env bash
set -Eeuo pipefail

# Retry drone_demo_stats' 1SP (one-hot, S-liveness, pending) cell, but only
# the specific (ordering_mode, ordering_seed) rows still incomplete after the
# main-track run -- not all 22 1SP orderings.
#
# The drone_demo_stats main-track task (run_experiment_plan.sh) already
# splits 1SP into 16 orderings that reliably resolve in well under 700s at
# its normal 3600s budget, and 6 known-slow orderings (alphabetic, domain,
# random seeds 6/10/12/16) it deliberately caps at 20s instead of chasing --
# those 6 are a placeholder there, always incomplete by construction, exactly
# like FullP1/1FP/2FP elsewhere. This script is their real attempt, at a
# longer, previously-confirmed-sufficient well-separation budget (21600s/6h,
# from the retired run_drone_demo_long_retry.sh this script narrows down to
# just the 1SP piece of).
#
# "1SP's realizability half was never the problem" turned out to be false in
# general -- it's true for most orderings but not for their slow tail: a
# 2026-08-24 second-machine run found 7 *different* random seeds (3, 4, 5, 8,
# 11, 13, 16; not the same 6 as above) whose well-separation resolved in
# seconds but whose realizability hit exactly this script's synthesis
# timeout (`synthesizer_time_s` landed at 3600.0s on every one) and never got
# a verdict. Cross-checked against this machine's own historical
# drone_demo_stats data, those same 7 seeds were always the slowest 1SP
# orderings for realizability (2540s-12194s, up to 3.4h) -- previously
# resolved only by older, more patient long-retry passes with a much bigger
# synthesis budget than 3600s, not by the standard main-track/outlier path.
# `--synthesis-timeout-s` now defaults to the same 21600s/6h budget as
# well-separation (>1.7x the worst historical case) instead of 3600s, and the
# discovery step below treats a row as incomplete if realizability never got
# a verdict too, not just if well-separation didn't.
#
# Rather than trust the main-track task's hardcoded 6-seed list, this script
# re-discovers whichever rows are actually incomplete straight from
# drone_demo_stats' own master.csv, so it also self-corrects if an unlisted
# ordering turns out to be slow this run (this is exactly how the 7 seeds
# above were found, not from a hardcoded list). If every 1SP ordering is
# already resolved, it has nothing to do and exits cleanly rather than
# erroring.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "$REPO_ROOT/../.." && pwd)}"
SETUP_FILE="${SETUP_FILE:-$WORKSPACE_ROOT/install/setup.bash}"
SYNTHESIS_HOME="${FLEXBE_SYNTHESIS_HOME:-$HOME/.flexbe_synthesis}"
RUN_LOG="$SYNTHESIS_HOME/run_logs/drone_demo_stats_1sp_outliers.log"
SYNTHESIS_TIMEOUT_S="${SYNTHESIS_TIMEOUT_S:-21600}"               # 6h -- covers the 12194s/3.4h worst historical case
WELL_SEPARATION_TIMEOUT_S="${WELL_SEPARATION_TIMEOUT_S:-21600}"   # 6h -- the confirmed-sufficient budget
RANDOM_SEEDS="${RANDOM_SEEDS:-20}"                                # must match drone_demo_stats' --random-seeds

DRONE_PKG="$WORKSPACE_ROOT/src/flexbe_synthesis_demo/drone_demo/drone_demo_flexbe_synthesis"
DRONE_EXP="$SYNTHESIS_HOME/experiments/drone_demo"
MATRIX_FILE="$DRONE_EXP/matrices/drone_demo_stats_1sp_outliers.yaml"
OUT_DIR="$DRONE_EXP/runs/drone_demo_stats"

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

if [[ ! -f "$OUT_DIR/master.csv" ]]; then
  log "No existing run at $OUT_DIR -- run the drone_demo_stats main-track task first."
  exit 1
fi

log "Finding incomplete 1SP (one-hot/S/pending) orderings in $OUT_DIR/master.csv"
cd "$DRONE_PKG"
FOUND_ANY_FILE="$(mktemp)"
MASTER_BACKUP=""
RESTORE_MASTER_ON_FAILURE=0
cleanup() {
  local rc=$?
  rm -f "$FOUND_ANY_FILE"
  if (( rc != 0 && RESTORE_MASTER_ON_FAILURE )) && [[ -n "$MASTER_BACKUP" && -f "$MASTER_BACKUP" ]]; then
    log "Restoring $OUT_DIR/master.csv from $MASTER_BACKUP after failure rc=$rc"
    cp "$MASTER_BACKUP" "$OUT_DIR/master.csv"
  fi
  exit "$rc"
}
trap cleanup EXIT

MASTER_BACKUP="$OUT_DIR/master.csv.pre_1sp_outliers_$(date '+%Y%m%d_%H%M%S').bak"
log "Backing up $OUT_DIR/master.csv to $MASTER_BACKUP before discovering/stripping retry rows"
cp "$OUT_DIR/master.csv" "$MASTER_BACKUP"
RESTORE_MASTER_ON_FAILURE=1

python3 - "$MATRIX_FILE" "$OUT_DIR/master.csv" "$RANDOM_SEEDS" "$FOUND_ANY_FILE" <<'PYEOF'
import csv
import sys
import yaml
from drone_demo_flexbe_synthesis.drone_demo_matrix import build_matrix

out_path, stats_master_csv, random_seeds, found_any_path = sys.argv[1:5]
random_seeds = int(random_seeds)


def row_key(row):
    seed = row.get('ordering_seed')
    return (row.get('ordering_mode'), None if seed in (None, '', 'None') else str(seed))


def is_incomplete(row):
    """A 1SP row is only done once BOTH well-separation and realizability
    have a verdict. Well-separation resolving on its own (fast, independent
    of the full realizability search) is not enough -- see this file's
    header comment for the 2026-08-24 case (7 seeds) this check exists to
    catch: well_separation_status was WELL_SEPARATED for all of them, but
    `realizable` was still empty because synthesis hit its timeout.
    """
    if row.get('well_separation_status') not in ('WELL_SEPARATED', 'NON_WELL_SEPARATED'):
        return True
    if row.get('realizable') in (None, ''):
        return True
    return False


with open(stats_master_csv, newline='', encoding='utf-8') as f:
    reader = csv.DictReader(f)
    fieldnames = reader.fieldnames
    stats_rows = list(reader)

failing_keys = {
    row_key(row)
    for row in stats_rows
    if row.get('encoding') == 'one-hot'
    and row.get('liveness') == 'S'
    and row.get('pending') == 'True'
    and is_incomplete(row)
}

if not failing_keys:
    print("No incomplete 1SP rows found (well-separation + realizability) -- nothing to retry.")
    with open(found_any_path, 'w', encoding='utf-8') as f:
        f.write('0')
    sys.exit(0)

# random_seeds must match the --random-seeds used to generate drone_demo_stats'
# original matrix, so the regenerated reference rows line up with the
# (ordering_mode, ordering_seed) keys read from master.csv.
reference = build_matrix(stats=True, random_seeds=random_seeds)['rows']

rows = [
    row for row in reference
    if row.get('encoding') == 'one-hot'
    and row.get('liveness') == 'S'
    and row.get('pending') is True
    and row_key(row) in failing_keys
]
found_keys = {row_key(row) for row in rows}
missing = failing_keys - found_keys
if missing:
    raise SystemExit(
        f"{len(missing)} failing key(s) from master.csv have no match in the "
        f"regenerated --stats --random-seeds {random_seeds} matrix: {sorted(missing)}. "
        "Did --random-seeds change between the source run and this retry?"
    )

with open(out_path, 'w', encoding='utf-8') as f:
    yaml.safe_dump({'rows': rows}, f, sort_keys=False)
for key in sorted(found_keys, key=lambda k: (k[0], k[1] or '')):
    print(f"  retrying ordering_mode={key[0]} ordering_seed={key[1]}")
print(f"Wrote {len(rows)} row(s) to '{out_path}'")

# Strip exactly these failing (ordering_mode, ordering_seed) 1SP rows from
# master.csv -- NOT all 22 rows sharing column_indicator='1SP', which would
# also discard the already-resolved orderings.
kept_rows = [
    row for row in stats_rows
    if not (
        row.get('encoding') == 'one-hot'
        and row.get('liveness') == 'S'
        and row.get('pending') == 'True'
        and row_key(row) in failing_keys
    )
]
dropped = len(stats_rows) - len(kept_rows)
with open(stats_master_csv, 'w', newline='', encoding='utf-8') as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(kept_rows)
print(f"master.csv: dropped {dropped} stale 1SP row(s), {len(kept_rows)} row(s) remain")

with open(found_any_path, 'w', encoding='utf-8') as f:
    f.write('1')
PYEOF

if [[ "$(cat "$FOUND_ANY_FILE")" != "1" ]]; then
  RESTORE_MASTER_ON_FAILURE=0
  rm -f "$MASTER_BACKUP"
  MASTER_BACKUP=""
  log "Nothing to retry; drone_demo_stats' 1SP cell is already fully resolved."
  exit 0
fi

log "Running 1SP retry at synthesis-timeout-s=$SYNTHESIS_TIMEOUT_S well-separation-timeout-s=$WELL_SEPARATION_TIMEOUT_S"
run_cmd python3 -m flexbe_synthesis_slugs.helpers.batch_run_harness \
  --matrix "$MATRIX_FILE" \
  --out-dir "$OUT_DIR" \
  --state-mappings "$DRONE_PKG/mappings/batch_mappings.yaml" \
  --slugs-bin /usr/local/bin/slugs \
  --synthesis-timeout-s "$SYNTHESIS_TIMEOUT_S" \
  --well-separation-timeout-s "$WELL_SEPARATION_TIMEOUT_S" \
  --append

RESTORE_MASTER_ON_FAILURE=0

# drone_demo_stats uses the standard --stats sweep convention (threshold_250/random
# defaults), same as its original main-track postprocess invocation.
run_cmd python3 -m flexbe_synthesis_slugs.helpers.experiment_postprocess \
  --run-dir "$OUT_DIR" \
  --refresh-summary \
  --summary-plot

log "COMPLETE drone_demo_stats 1SP outliers (well-separation + realizability)"
