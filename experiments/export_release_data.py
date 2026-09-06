#!/usr/bin/env python3
"""Export a clean, publishable data bundle from local experiment run folders.

``$FLEXBE_SYNTHESIS_HOME/experiments/<domain>/runs/`` accumulates one folder
per run attempt, including backups and smoke tests taken before retries
(e.g. ``two_rivers_full_rerun_pre_baseline_fix_backup``,
``pyrobosim_stats_pre_relabel_backup_20260813_124635``). Only one folder per
domain is the canonical run for this release. This script copies just
the structured, per-trial results (``master.csv``, ``summary.csv``,
``tables/``, ``plots/``) from the canonical folder for each domain into
``release_data/<label>/``, and scrubs local filesystem paths and the
hostname from every copied CSV cell so nothing machine-identifying ships
in the public release.

It deliberately does NOT copy each run's nested ``runs/<config>/trial_NN/``
subtree (raw per-trial Slugs byproducts: structuredslugs output, extracted
Mealy JSON, variable-order files). Every number in those files is already
flattened into ``master.csv``; the byproducts add gigabytes without adding
anything needed for release reproducibility or for a reader's own
percentile/IQR/paired-seed analysis. Publish them separately (e.g. as a
Zenodo archive) only if you want to support deeper artifact-level
reproducibility on top of what this script already exports.

Usage:
    python3 export_release_data.py [--home PATH] [--out PATH] [--dry-run]

Canonical run selection is deliberately hardcoded below, not
auto-discovered: guessing which sibling folder is "the real one" among
several backups is exactly the mistake this script exists to prevent.
Update CANONICAL_RUNS by hand when a domain's reported run changes.
"""

from __future__ import annotations

import argparse
import csv
import re
import shutil
import socket
import sys
from pathlib import Path

# (release label, domain subdir under experiments/, run folder name under
# runs/) -- one entry per canonical run exported in this release.
# PyRoboSim and the quadcopter demo each contribute two entries (P0/P1,
# and the main sweep/Fair-Outcome-with-pending attempts respectively).
CANONICAL_RUNS: list[tuple[str, str, str]] = [
    ("coffee", "coffee", "coffee_full_rerun"),
    ("two_rivers", "two_rivers", "two_rivers_full_rerun"),
    ("pyrobosim_p0", "pyrobosim", "pyrobosim_stats"),
    ("pyrobosim_p1", "pyrobosim", "pyrobosim_p1_default"),
    ("crazyflie", "drone_demo", "drone_demo_stats"),
    ("crazyflie_fair_outcome_pending", "drone_demo", "drone_demo_fp"),
]

# Files/dirs copied from each run folder. Deliberately excludes the nested
# "runs/" byproducts subtree and "harness.log" (console output, not data).
COPY_ITEMS = ["master.csv", "summary.csv", "tables", "plots"]


def _home_pattern(home: Path) -> re.Pattern[str]:
    """Return a regex matching `home` only at a path boundary.

    Requires the match be followed by '/' or by nothing path-safe-character-
    like, so a *different* home directory that merely starts with the same
    string (e.g. /home/alice2 on a shared machine, against home=/home/alice)
    is left untouched instead of being mangled.
    """
    return re.compile(re.escape(str(home)) + r"(/|(?![A-Za-z0-9._-]))")


def scrub_csv(src: Path, dst: Path, home: Path) -> tuple[int, int]:
    """Copy a CSV, stripping the local home-directory prefix from every cell.

    Returns (row_count, cells_changed) for the printed manifest. A generic
    pattern replacement -- rather than special-casing path-bearing column
    names -- is robust to schema changes and catches every absolute path in
    the file (trial/byproduct paths, capability-file paths) in one pass,
    since they all currently hang off the same home directory.
    """
    home_re = _home_pattern(home)

    def scrub_cell(cell: str) -> tuple[str, bool]:
        changed = False

        def repl(m: re.Match[str]) -> str:
            nonlocal changed
            changed = True
            return "" if m.group(1) == "/" else "."

        return home_re.sub(repl, cell), changed

    row_count = 0
    cells_changed = 0
    with src.open("r", newline="", encoding="utf-8") as f_in, \
            dst.open("w", newline="", encoding="utf-8") as f_out:
        reader = csv.reader(f_in)
        writer = csv.writer(f_out)
        header = next(reader, None)
        if header is not None:
            writer.writerow(header)
            hostname_idx = header.index("hostname") if "hostname" in header else None
        else:
            hostname_idx = None
        for row in reader:
            row_count += 1
            new_row = []
            for idx, cell in enumerate(row):
                if idx == hostname_idx and cell:
                    new_row.append("REDACTED-HOST")
                    cells_changed += 1
                else:
                    new_cell, changed = scrub_cell(cell)
                    new_row.append(new_cell)
                    if changed:
                        cells_changed += 1
            writer.writerow(new_row)
    return row_count, cells_changed


def scan_dir_for_leaks(directory: Path, home: Path) -> list[Path]:
    """Return files under `directory` whose raw bytes contain the home path or hostname.

    tables/ and plots/ are copied verbatim below, not cell-scrubbed like the CSVs --
    nothing in the current table/plot generators embeds a filesystem path or
    hostname (verified empirically against the real generated files), but that's an
    observation about today's generator code, not a guarantee. This is a best-effort
    safety net against that assumption going stale (e.g. a future plot title that
    includes a source file path), scanning raw bytes so it catches both text files
    (.tex) and binary ones (.pdf/.png) the same way. It does not replace the
    pre-publish spot-check `main()` already tells you to do.
    """
    needles = [str(home).encode("utf-8")]
    hostname = socket.gethostname()
    if hostname:
        needles.append(hostname.encode("utf-8"))
    hits = []
    for path in directory.rglob("*"):
        if not path.is_file():
            continue
        try:
            data = path.read_bytes()
        except OSError:
            continue
        if any(needle in data for needle in needles):
            hits.append(path)
    return hits


def export_one(label: str, domain: str, run_name: str, experiments_root: Path,
                out_root: Path, home: Path, dry_run: bool) -> int:
    """Export a single canonical run's structured results. Returns a leak-warning count."""
    src_run = experiments_root / domain / "runs" / run_name
    dst_run = out_root / label

    if not src_run.is_dir():
        print(f"  SKIP {label}: source not found at {src_run}", file=sys.stderr)
        return 0

    print(f"  {label}  <-  {domain}/runs/{run_name}")
    if dry_run:
        for item in COPY_ITEMS:
            src_item = src_run / item
            if src_item.exists():
                print(f"    would copy {item}")
        return 0

    dst_run.mkdir(parents=True, exist_ok=True)

    warnings = 0
    for item in COPY_ITEMS:
        src_item = src_run / item
        if not src_item.exists():
            continue
        if src_item.is_dir():
            dst_item = dst_run / item
            shutil.copytree(src_item, dst_item, dirs_exist_ok=True)
            print(f"    copied {item}/")
            for hit in scan_dir_for_leaks(dst_item, home):
                warnings += 1
                print(f"    WARNING: {hit} contains the local home path or hostname unscrubbed")
        elif src_item.suffix == ".csv":
            rows, changed = scrub_csv(src_item, dst_run / item, home)
            print(f"    scrubbed {item} ({rows} rows, {changed} cells rewritten)")
        else:
            shutil.copy2(src_item, dst_run / item)
            print(f"    copied {item}")
    return warnings


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--home", type=Path, default=Path("~/.flexbe_synthesis").expanduser(),
        help="FLEXBE_SYNTHESIS_HOME, where experiments/ lives (default: ~/.flexbe_synthesis)",
    )
    parser.add_argument(
        "--scrub-prefix", type=Path, default=Path.home(),
        help=(
            "filesystem prefix to strip from every CSV cell (default: the "
            "actual $HOME, e.g. /home/alice -- NOT --home/FLEXBE_SYNTHESIS_HOME. "
            "capability_file paths point outside FLEXBE_SYNTHESIS_HOME (into the "
            "flexbe_synthesis source checkout), so scrubbing only the "
            "experiments root would leave those exposed."
        ),
    )
    parser.add_argument(
        "--out", type=Path, default=None,
        help=(
            "output directory for the exported bundle "
            "(default: <--home>/release_data, e.g. ~/.flexbe_synthesis/release_data, "
            "kept alongside the rest of the local experiment data rather than "
            "inside the git-tracked demo repo -- zip it up separately when ready)"
        ),
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="print what would be exported without writing anything",
    )
    args = parser.parse_args()
    if args.out is None:
        args.out = args.home / "release_data"

    experiments_root = args.home / "experiments"
    if not experiments_root.is_dir():
        print(f"error: {experiments_root} does not exist", file=sys.stderr)
        return 1

    print(f"Exporting canonical runs from {experiments_root}")
    print(f"  scrub prefix: {args.scrub_prefix}")
    print(f"  output: {args.out.resolve()}" + (" (dry run)" if args.dry_run else ""))
    print()

    total_warnings = 0
    for label, domain, run_name in CANONICAL_RUNS:
        total_warnings += export_one(
            label, domain, run_name, experiments_root, args.out, args.scrub_prefix, args.dry_run
        )

    if not args.dry_run:
        print(f"\nDone. Inspect {args.out.resolve()} before publishing:")
        print("in particular, spot-check master.csv for this machine's home")
        print("directory or hostname before making the bundle public.")
        if total_warnings:
            print(
                f"\n{total_warnings} unscrubbed home-path/hostname hit(s) found in "
                "copied tables/plots -- see WARNING lines above before publishing."
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
