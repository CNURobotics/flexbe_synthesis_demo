#!/usr/bin/env python3
"""Retry drone_demo_stats trials that did not complete within their original
per-trial timeout, using a longer one.

A prior `drone_demo_stats` run (`--stats --random-seeds 20`, at the old
default of `--synthesis-timeout-s 900`) left
20 of 132 trials incomplete: 15 of 20 random-seed trials for the one-hot
`S`+`pending=True` (1SP) configuration, and 5 of 20 for one-hot
`F`+`pending=False` (1F). Enumerated encoding had zero timeouts.

This script does not guess which rows to retry: it reads them directly out
of an existing run's `master.csv` (any row with a non-zero `return_code`),
regenerates the matching rows from the *same* matrix generator used for the
original run (`drone_demo_matrix.build_matrix`, so row content is guaranteed
identical to the source run, not reconstructed by hand), and re-runs just
those trials through the batch harness with a longer timeout. It does not
touch the source run's directory.

Usage (after sourcing the workspace):

    cd "$WORKSPACE_ROOT/src/flexbe_synthesis_demo/drone_demo/drone_demo_flexbe_synthesis"
    python3 "$WORKSPACE_ROOT/src/flexbe_synthesis_demo/experiments/retry_drone_demo_timeouts.py"

By default this retries the trials that failed in
`$FLEXBE_SYNTHESIS_HOME/experiments/drone_demo/runs/drone_demo_stats` at
3600s (4x the original 900s budget) into a new sibling `_retry_3600s`
output directory, then leaves postprocessing (`experiment_postprocess
--refresh-summary --summary-plot`) as a separate, explicit step -- run it
yourself once you've looked at whether the retry actually resolved
anything, same as every other run in this repo.

All of --source-run / --out-dir / --timeout-s / --random-seeds /
--slugs-bin / --state-mappings / --auditor-timeout-s can be overridden; see
--help.
"""

import argparse
import csv
import os
import subprocess
import sys
from pathlib import Path

import yaml

try:
    from drone_demo_flexbe_synthesis.drone_demo_matrix import build_matrix
except ImportError:
    print(
        "ERROR: could not import drone_demo_flexbe_synthesis.drone_demo_matrix.\n"
        "Run this from the drone_demo_flexbe_synthesis package directory (or\n"
        "after sourcing the workspace install/setup.bash), same as the other\n"
        "manual experiment commands.",
        file=sys.stderr,
    )
    raise

DEFAULT_SYNTHESIS_HOME = Path(os.environ.get("FLEXBE_SYNTHESIS_HOME", Path.home() / ".flexbe_synthesis"))
DEFAULT_SOURCE_RUN = DEFAULT_SYNTHESIS_HOME / "experiments" / "drone_demo" / "runs" / "drone_demo_stats"
DEFAULT_STATE_MAPPINGS = (
    "$WORKSPACE_ROOT/src/flexbe_synthesis_demo/drone_demo/drone_demo_flexbe_synthesis/"
    "mappings/batch_mappings.yaml"
)

ROW_KEY_FIELDS = ("encoding", "liveness", "pending", "ordering_mode", "ordering_seed")


def row_key(row: dict) -> tuple:
    """(encoding, liveness, pending, ordering_mode, ordering_seed), normalized for
    comparison between a matrix-generator row (Python bool/int/None) and a
    master.csv row (string columns)."""
    pending = row["pending"]
    if isinstance(pending, str):
        pending = pending.strip().lower() == "true"
    seed = row["ordering_seed"]
    if seed in (None, "", "null", "None"):
        seed = None
    else:
        seed = int(seed)
    return (row["encoding"], row["liveness"], bool(pending), row["ordering_mode"], seed)


def failing_row_keys(master_csv: Path) -> list:
    with master_csv.open(newline="") as f:
        rows = list(csv.DictReader(f))
    fails = [r for r in rows if r.get("return_code") not in ("0", 0, None, "")]
    keys = [row_key(r) for r in fails]
    # De-duplicate while preserving order, in case a row was repeated (n_trials > 1).
    seen = set()
    unique = []
    for k in keys:
        if k not in seen:
            seen.add(k)
            unique.append(k)
    return unique


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--source-run",
        type=Path,
        default=DEFAULT_SOURCE_RUN,
        help=f"Run directory containing the master.csv to read failures from "
        f"(default: {DEFAULT_SOURCE_RUN}).",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Output directory for the retry run (default: "
        "<source-run>_retry_<timeout-s>s, as a sibling of --source-run).",
    )
    parser.add_argument(
        "--timeout-s",
        type=int,
        default=3600,
        help="Per-trial Slugs timeout for the retry, in seconds (default: 3600, "
        "4x the original 900s default).",
    )
    parser.add_argument(
        "--random-seeds",
        type=int,
        default=20,
        help="Must match --random-seeds used to generate --source-run's matrix "
        "(default: 20), so the regenerated reference matrix's rows line up with "
        "the failing (encoding, liveness, pending, ordering_mode, ordering_seed) "
        "keys read from master.csv.",
    )
    parser.add_argument(
        "--slugs-bin",
        default="/usr/local/bin/slugs",
        help="Path to the Slugs binary (default: /usr/local/bin/slugs).",
    )
    parser.add_argument(
        "--state-mappings",
        default=None,
        help="State mappings YAML path, passed through to batch_run_harness. "
        f"Required (no default resolvable without $WORKSPACE_ROOT; the other "
        f"drone_demo tasks use {DEFAULT_STATE_MAPPINGS}).",
    )
    parser.add_argument(
        "--auditor-timeout-s",
        type=int,
        default=None,
        help="Per-trial auditor timeout, passed through to batch_run_harness "
        "if given.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print which rows would be retried and where, but do not write the "
        "filtered matrix or invoke batch_run_harness.",
    )
    args = parser.parse_args()

    if args.state_mappings is None:
        parser.error(
            "--state-mappings is required, e.g.\n  "
            f"  {DEFAULT_STATE_MAPPINGS}\n"
            "  (with $WORKSPACE_ROOT expanded to your actual workspace root)"
        )

    master_csv = args.source_run / "master.csv"
    if not master_csv.is_file():
        parser.error(f"No master.csv found at {master_csv}")

    keys_to_retry = failing_row_keys(master_csv)
    if not keys_to_retry:
        print(f"No failing rows found in {master_csv}; nothing to retry.")
        return

    print(f"Found {len(keys_to_retry)} failing row(s) in {master_csv}:")
    for k in keys_to_retry:
        encoding, liveness, pending, ordering_mode, seed = k
        print(
            f"  encoding={encoding} liveness={liveness} pending={pending} "
            f"ordering_mode={ordering_mode} ordering_seed={seed}"
        )

    reference = build_matrix(stats=True, random_seeds=args.random_seeds)
    wanted = set(keys_to_retry)
    filtered_rows = [r for r in reference["rows"] if row_key(r) in wanted]

    found_keys = {row_key(r) for r in filtered_rows}
    missing = wanted - found_keys
    if missing:
        print(
            f"WARNING: {len(missing)} failing row(s) from master.csv have no match "
            f"in the regenerated --stats --random-seeds {args.random_seeds} matrix; "
            "did --random-seeds change between the source run and this retry?",
            file=sys.stderr,
        )
        for k in sorted(missing):
            print(f"  no match for {k}", file=sys.stderr)

    out_dir = args.out_dir or args.source_run.parent / f"{args.source_run.name}_retry_{args.timeout_s}s"
    matrices_dir = out_dir.parent / "matrices"

    print(
        f"\nRetrying {len(filtered_rows)} row(s) at --synthesis-timeout-s "
        f"{args.timeout_s} into {out_dir}"
    )

    if args.dry_run:
        print("(--dry-run: stopping before writing the matrix or running the harness)")
        return

    matrices_dir.mkdir(parents=True, exist_ok=True)
    matrix_path = matrices_dir / f"drone_demo_stats_retry_{args.timeout_s}s.yaml"
    matrix_path.write_text(
        yaml.safe_dump({"rows": filtered_rows}, sort_keys=False), encoding="utf-8"
    )
    print(f"Wrote {len(filtered_rows)} row(s) to {matrix_path}")

    cmd = [
        sys.executable,
        "-m",
        "flexbe_synthesis_slugs.helpers.batch_run_harness",
        "--matrix",
        str(matrix_path),
        "--out-dir",
        str(out_dir),
        "--state-mappings",
        args.state_mappings,
        "--slugs-bin",
        args.slugs_bin,
        "--synthesis-timeout-s",
        str(args.timeout_s),
    ]
    if args.auditor_timeout_s is not None:
        cmd += ["--auditor-timeout-s", str(args.auditor_timeout_s)]

    print("\n$", " ".join(cmd))
    subprocess.run(cmd, check=True)

    print(
        f"\nDone. Next step (same pattern as the other runs):\n"
        f"  python3 -m flexbe_synthesis_slugs.helpers.experiment_postprocess \\\n"
        f"    --run-dir {out_dir} \\\n"
        f"    --refresh-summary \\\n"
        f"    --summary-plot\n"
        f"\nThen compare {out_dir}/master.csv against {master_csv} for the same\n"
        f"(encoding, liveness, pending, ordering_mode, ordering_seed) rows to see\n"
        f"whether the longer budget resolved them, and merge/report accordingly --\n"
        f"this script does not modify {args.source_run} or merge results itself."
    )


if __name__ == "__main__":
    main()
