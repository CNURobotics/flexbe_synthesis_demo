#!/usr/bin/env bash
set -Eeuo pipefail

# Final long-budget retry of drone_demo's three unresolved cells:
#
#   - 2FP (enumerated, F-liveness, pending): realizability attempts at 900s, 7200s,
#     18000s, 36000s, and 57600s all timed out (SYNTHESIS_FAILED, no verdict). This
#     gives it one more shot at 24h. Well-separation for 2FP is already resolved
#     (NON_WELL_SEPARATED, ~75s) and not the bottleneck -- included here only because
#     retrying the row means resynthesizing it end to end.
#   - 1FP (one-hot, F-liveness, pending): stuck at its last (36000s) attempt --
#     SYNTHESIS_FAILED, no realizability verdict, AND well-separation itself never
#     finished either (still ANALYSIS_INCOMPLETE from the original 1800s-default run).
#     Included for completeness alongside 2FP, same single fixed alphabetic ordering,
#     not swept across seeds.
#   - 1SP (one-hot, S-liveness, pending): already realizable, but drone_demo_stats
#     reports "Mixed" well-separation -- 6 of 22 orderings (alphabetic, domain, and
#     random seeds 6/10/12/16) hit the well-separation analyzer's default 1800s cap
#     and never got a verdict, while the other 16 resolved cleanly (all
#     WELL_SEPARATED, none NON_WELL_SEPARATED, longest resolved case ~700s) -- so a
#     longer well-separation timeout should clear the remaining 6 without needing
#     anywhere near 24h. Only those 6 specific orderings are retried, read directly
#     out of drone_demo_stats' master.csv (not guessed); the 16 already-resolved
#     orderings are left untouched.
#
# 1FP/2FP stay single fixed-ordering (alphabetic), matching their existing --fp
# treatment -- not swept across random seeds, per instructions.
#
# Two separate output directories are involved (1FP/2FP live in drone_demo_fp, 1SP
# lives in drone_demo_stats), so this runs batch_run_harness twice, back to back,
# both at the same --synthesis-timeout-s/--well-separation-timeout-s. Stale rows for
# everything being retried are stripped from each master.csv before appending, same
# reasoning as the PyRoboSim P1 retry scripts: otherwise write_summary groups the old
# incomplete row with the new one and the mismatched values collapse the summary cell
# to "Mixed" instead of showing the fresh verdict.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "$REPO_ROOT/../.." && pwd)}"
SETUP_FILE="${SETUP_FILE:-$WORKSPACE_ROOT/install/setup.bash}"
SYNTHESIS_HOME="${FLEXBE_SYNTHESIS_HOME:-$HOME/.flexbe_synthesis}"
RUN_LOG="$SYNTHESIS_HOME/run_logs/drone_demo_long_retry.log"
SYNTHESIS_TIMEOUT_S="${SYNTHESIS_TIMEOUT_S:-86400}"              # 24h
WELL_SEPARATION_TIMEOUT_S="${WELL_SEPARATION_TIMEOUT_S:-21600}"  # 6h

DRONE_PKG="$WORKSPACE_ROOT/src/flexbe_synthesis_demo/drone_demo/drone_demo_flexbe_synthesis"
DRONE_EXP="$SYNTHESIS_HOME/experiments/drone_demo"
FP_MATRIX_FILE="$DRONE_EXP/matrices/drone_demo_fp_24h.yaml"
SP_MATRIX_FILE="$DRONE_EXP/matrices/drone_demo_1sp_wellsep_retry.yaml"
FP_OUT_DIR="$DRONE_EXP/runs/drone_demo_fp"
STATS_OUT_DIR="$DRONE_EXP/runs/drone_demo_stats"

timestamp() {
  date '+%Y-%m-%d %H:%M:%S %z'
}

log() {
  mkdir -p "$(dirname "$RUN_LOG")"
  printf '[%s] %s\n' "$(timestamp)" "$*" | tee -a "$RUN_LOG"
}

die() {
  log "ERROR: $*"
  exit 1
}

run_cmd() {
  printf '\n[%s] $' "$(timestamp)" | tee -a "$RUN_LOG"
  printf ' %q' "$@" | tee -a "$RUN_LOG"
  printf '\n' | tee -a "$RUN_LOG"
  "$@" 2>&1 | tee -a "$RUN_LOG"
}

strip_indicators() {
  # strip_indicators MASTER_CSV INDICATOR [INDICATOR...]
  local master_csv="$1"
  shift
  [[ -f "$master_csv" ]] || { log "No existing master.csv at $master_csv -- nothing to strip"; return; }
  python3 - "$master_csv" "$@" <<'PYEOF'
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
}

set +u
source "$SETUP_FILE"
set -u
export FLEXBE_SYNTHESIS_HOME="$SYNTHESIS_HOME"

mkdir -p "$DRONE_EXP/matrices"

BACKUP_SUFFIX="pre_long_retry_backup_$(date +%Y%m%d_%H%M%S)"
if [[ -d "$FP_OUT_DIR" ]]; then
  log "Backing up $FP_OUT_DIR"
  cp -r "$FP_OUT_DIR" "${FP_OUT_DIR}_${BACKUP_SUFFIX}"
fi
if [[ -d "$STATS_OUT_DIR" ]]; then
  log "Backing up $STATS_OUT_DIR"
  cp -r "$STATS_OUT_DIR" "${STATS_OUT_DIR}_${BACKUP_SUFFIX}"
fi

cd "$DRONE_PKG"

log "Building FP matrix (1FP + 2FP, single alphabetic ordering)"
python3 - "$FP_MATRIX_FILE" <<'PYEOF'
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

log "Stripping stale 1FP/2FP rows from $FP_OUT_DIR/master.csv before appending"
strip_indicators "$FP_OUT_DIR/master.csv" 1FP 2FP

log "Building 1SP well-separation retry matrix and stripping its stale rows (only the orderings that timed out -- NOT all 22 1SP rows)"
python3 - "$SP_MATRIX_FILE" "$STATS_OUT_DIR/master.csv" <<'PYEOF'
import csv
import sys
import yaml
from drone_demo_flexbe_synthesis.drone_demo_matrix import build_matrix

out_path, stats_master_csv = sys.argv[1:3]


def row_key(row):
    seed = row.get('ordering_seed')
    return (row.get('ordering_mode'), None if seed in (None, '', 'None') else str(seed))


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
    and row.get('well_separation_status') not in ('WELL_SEPARATED', 'NON_WELL_SEPARATED')
}
if not failing_keys:
    raise SystemExit(
        "No incomplete 1SP (one-hot/S/pending) well-separation rows found in "
        f"{stats_master_csv!r} -- nothing to retry."
    )

# random_seeds must match the --random-seeds used to generate drone_demo_stats'
# original matrix (20), so the regenerated reference rows line up with the
# (ordering_mode, ordering_seed) keys read from master.csv.
reference = build_matrix(stats=True, random_seeds=20)['rows']

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
        f"regenerated --stats --random-seeds 20 matrix: {sorted(missing)}. Did "
        "--random-seeds change between the source run and this retry?"
    )

with open(out_path, 'w', encoding='utf-8') as f:
    yaml.safe_dump({'rows': rows}, f, sort_keys=False)
for key in sorted(found_keys, key=lambda k: (k[0], k[1] or '')):
    print(f"  retrying ordering_mode={key[0]} ordering_seed={key[1]}")
print(f"Wrote {len(rows)} row(s) to '{out_path}'")

# Strip exactly these failing (ordering_mode, ordering_seed) 1SP rows from
# master.csv -- NOT all 22 rows sharing column_indicator='1SP', which would also
# discard the 16 already-resolved orderings.
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
PYEOF

log "Running FP matrix (1FP, 2FP) at synthesis-timeout-s=$SYNTHESIS_TIMEOUT_S well-separation-timeout-s=$WELL_SEPARATION_TIMEOUT_S"
run_cmd python3 -m flexbe_synthesis_slugs.helpers.batch_run_harness \
  --matrix "$FP_MATRIX_FILE" \
  --out-dir "$FP_OUT_DIR" \
  --state-mappings "$DRONE_PKG/mappings/batch_mappings.yaml" \
  --slugs-bin /usr/local/bin/slugs \
  --synthesis-timeout-s "$SYNTHESIS_TIMEOUT_S" \
  --well-separation-timeout-s "$WELL_SEPARATION_TIMEOUT_S" \
  --append

log "Running 1SP well-separation retry at synthesis-timeout-s=$SYNTHESIS_TIMEOUT_S well-separation-timeout-s=$WELL_SEPARATION_TIMEOUT_S"
run_cmd python3 -m flexbe_synthesis_slugs.helpers.batch_run_harness \
  --matrix "$SP_MATRIX_FILE" \
  --out-dir "$STATS_OUT_DIR" \
  --state-mappings "$DRONE_PKG/mappings/batch_mappings.yaml" \
  --slugs-bin /usr/local/bin/slugs \
  --synthesis-timeout-s "$SYNTHESIS_TIMEOUT_S" \
  --well-separation-timeout-s "$WELL_SEPARATION_TIMEOUT_S" \
  --append

# drone_demo_fp's rows are alphabetic-ordering / threshold_250 -- experiment_postprocess table writer's
# default --table-ordering ('random') would match zero rows here.
run_cmd python3 -m flexbe_synthesis_slugs.helpers.experiment_postprocess \
  --run-dir "$FP_OUT_DIR" \
  --refresh-summary \
  --table-policy threshold_250 \
  --table-ordering alphabetic

# drone_demo_stats uses the standard --stats sweep convention (threshold_250/random
# defaults), same as its original postprocess invocation.
run_cmd python3 -m flexbe_synthesis_slugs.helpers.experiment_postprocess \
  --run-dir "$STATS_OUT_DIR" \
  --refresh-summary \
  --summary-plot

log "COMPLETE drone_demo long retry (1FP, 2FP, 1SP well-separation)"
