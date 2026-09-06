#!/usr/bin/env python3
"""Run Slugs well-separation checks over experiment Slugs byproducts.

The script discovers ``*.slugsin`` files below
``$FLEXBE_SYNTHESIS_HOME/experiments``, collapses repeated trials for each
scenario, runs Slugs' ``--checkWellSeparation`` mode, and writes labeled CSV
and JSONL summaries suitable for later table aggregation.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import statistics
import subprocess
import sys
import time
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


DEMO_LABELS = {
    "coffee": "coffee",
    "two_rivers": "two_rivers",
    "pyrobosim": "pyrobosim",
    "drone_demo": "drones",
}

SCENARIO_COLUMNS = [
    "example",
    "system_name",
    "spec_name",
    "capability_file",
    "encoding",
    "liveness",
    "pending",
    "column_indicator",
    "ordering_mode",
    "ordering_seed",
    "reordering_enabled",
    "reordering_threshold",
    "reordering_policy",
    "row_id",
]

NUMERIC_MASTER_COLUMNS = [
    "realizability_time_s",
    "extraction_time_s",
    "auditor_time_s",
    "overall_pipeline_time_s",
    "cudd_peak_nodes",
    "cudd_live_nodes",
    "cudd_var_count",
    "cudd_reorderings",
]

SUMMARY_COLUMNS = [
    "demo",
    "run_name",
    *SCENARIO_COLUMNS,
    "representative_trial",
    "trial_count_for_scenario_hash",
    "master_row_count",
    "slugsin_sha256",
    "slugsin_file",
    "ap_i",
    "ap_o",
    "env_init_count",
    "sys_init_count",
    "env_trans_count",
    "sys_trans_count",
    "env_liveness_count",
    "sys_liveness_count",
    "synthesis_realizability_time_s_mean",
    "synthesis_realizability_time_s_median",
    "synthesis_realizability_time_s_min",
    "synthesis_realizability_time_s_max",
    "synthesis_overall_pipeline_time_s_mean",
    "synthesis_cudd_peak_nodes_max",
    "well_separation_status",
    "well_separation_complete",
    "violated_assumption_type",
    "cases",
    "responsible_assumption_count",
    "core_enabled",
    "core_complete",
    "core_candidate_count",
    "core_solver_calls",
    "core_assumption_count",
    "core_assumption_signature",
    "core_assumption_kinds",
    "core_assumption_names",
    "core_assumption_lines",
    "core_assumption_source_indices",
    "core_slugsin_lines",
    "core_assumptions",
    "slugs_reported_elapsed_time_s",
    "slugs_timing_total_s",
    "measured_wall_time_s",
    "well_separation_return_code",
    "well_separation_error",
    "warnings",
    "json_result_file",
]

SUMMARY_BASENAME = "well_separation_summary"
MINCORE_SUMMARY_BASENAME = "well_separation_mincore_summary"
MINCORE_REPORT_BASENAME = "well_separation_mincore_report"

REPORT_COLUMNS = [
    "demo",
    "column_indicator",
    "spec_name",
    "capability_file",
    "encoding",
    "liveness",
    "pending",
    "well_separation_status",
    "violated_assumption_type",
    "scenario_count",
    "master_row_count",
    "trial_count",
    "ordering_modes",
    "ordering_seeds",
    "row_ids",
    "core_assumption_count_values",
    "distinct_core_signature_count",
    "core_assumption_signatures",
    "core_assumption_kinds",
    "core_assumption_names",
    "core_assumption_lines",
    "core_assumption_source_indices",
    "cases_values",
    "json_result_files",
    "slugsin_files",
]


@dataclass
class Scenario:
    demo: str
    run_name: str
    row_id: str
    sha256: str
    representative: Path
    trial_names: set[str] = field(default_factory=set)
    master_rows: list[dict[str, str]] = field(default_factory=list)


def parse_args() -> argparse.Namespace:
    default_home = Path(os.environ.get("FLEXBE_SYNTHESIS_HOME", "~/.flexbe_synthesis")).expanduser()
    parser = argparse.ArgumentParser(
        description="Run Slugs well-separation checks for distinct experiment .slugsin artifacts."
    )
    parser.add_argument(
        "--synthesis-home",
        type=Path,
        default=default_home,
        help=f"FlexBE synthesis output root (default: {default_home})",
    )
    parser.add_argument(
        "--slugs-bin",
        default="/usr/local/bin/slugs",
        help="Slugs binary with --checkWellSeparation support (default: /usr/local/bin/slugs)",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Output directory (default: <synthesis-home>/experiments/well_separation_checks)",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=1800.0,
        help="Per-check timeout in seconds (default: 1800)",
    )
    parser.add_argument(
        "--demo",
        action="append",
        choices=sorted(DEMO_LABELS.values()) + ["drone_demo"],
        help="Limit to a demo label. May be repeated. Use drones or drone_demo for the drone demo.",
    )
    parser.add_argument(
        "--column-indicator",
        action="append",
        help="Limit to a master.csv column_indicator value. May be repeated, e.g. --column-indicator 3FP.",
    )
    parser.add_argument(
        "--row-id-regex",
        action="append",
        help="Limit to scenarios whose row_id matches this regular expression. May be repeated.",
    )
    parser.add_argument(
        "--only-status",
        action="append",
        choices=[
            "WELL_SEPARATED",
            "NON_WELL_SEPARATED",
            "ANALYSIS_INCOMPLETE",
            "ANALYSIS_ERROR",
        ],
        help=(
            "Limit to scenarios with this status in an existing plain summary. "
            "Most useful with --minimize-core --only-status NON_WELL_SEPARATED."
        ),
    )
    parser.add_argument(
        "--sample-one-per-scenario",
        action="store_true",
        help=(
            "Keep only the fastest scenario for each demo/column_indicator/status group "
            "after other filters. This is useful for minimum-core spot checks."
        ),
    )
    parser.add_argument(
        "--skip-historical-timeout-s",
        type=float,
        default=None,
        help=(
            "Skip scenarios whose historical master.csv overall or realizability time "
            "is at least this many seconds. Use 18000 to skip prior 5-hour timeouts."
        ),
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=None,
        help="Run at most this many scenario/hash checks, after synthesis-time ordering.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Re-run checks even when a result JSON already exists.",
    )
    parser.add_argument(
        "--minimize-core",
        action="store_true",
        help="Pass --minimizeWellSeparationCore to Slugs for additional diagnostics.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Discover and summarize planned checks without invoking Slugs.",
    )
    parser.add_argument(
        "--report-only",
        action="store_true",
        help=(
            "Read an existing summary and result JSON files, then rewrite enriched summaries "
            "and grouped report files without invoking Slugs."
        ),
    )
    return parser.parse_args()


def stable_float(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    try:
        result = float(value)
    except ValueError:
        return None
    if not math.isfinite(result):
        return None
    return result


def scenario_master_value(scenario: Scenario, column: str) -> str:
    return first_master_row(scenario).get(column, "")


def historical_max_time_s(scenario: Scenario) -> float | None:
    values = []
    for row in scenario.master_rows:
        for column in ["overall_pipeline_time_s", "realizability_time_s"]:
            value = stable_float(row.get(column))
            if value is not None:
                values.append(value)
    return max(values) if values else None


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_master_rows(experiments_root: Path) -> dict[Path, dict[str, list[dict[str, str]]]]:
    rows_by_run: dict[Path, dict[str, list[dict[str, str]]]] = {}
    for master_csv in sorted(experiments_root.glob("*/runs/*/master.csv")):
        run_dir = master_csv.parent
        by_row_id: dict[str, list[dict[str, str]]] = defaultdict(list)
        with master_csv.open(newline="") as stream:
            reader = csv.DictReader(stream)
            for row in reader:
                by_row_id[row.get("row_id", "")].append(row)
        rows_by_run[run_dir] = by_row_id
    return rows_by_run


def parse_slugsin_location(path: Path, experiments_root: Path) -> tuple[str, Path, str, str] | None:
    try:
        rel = path.relative_to(experiments_root)
    except ValueError:
        return None
    parts = rel.parts
    if len(parts) < 7 or parts[1] != "runs":
        return None
    demo = parts[0]
    run_name = parts[2]
    if parts[3] != "runs":
        return None
    row_id = parts[4]
    trial_name = parts[5]
    run_dir = experiments_root / demo / "runs" / run_name
    return demo, run_dir, row_id, trial_name


def discover_scenarios(experiments_root: Path) -> list[Scenario]:
    master_rows = load_master_rows(experiments_root)
    scenarios: dict[tuple[str, str, str, str], Scenario] = {}
    for slugsin in sorted(experiments_root.glob("*/runs/*/runs/*/trial_*/synthesis_byproducts/*.slugsin")):
        parsed = parse_slugsin_location(slugsin, experiments_root)
        if parsed is None:
            continue
        demo, run_dir, row_id, trial_name = parsed
        sha = sha256_file(slugsin)
        key = (demo, run_dir.name, row_id, sha)
        scenario = scenarios.get(key)
        if scenario is None:
            scenario = Scenario(
                demo=demo,
                run_name=run_dir.name,
                row_id=row_id,
                sha256=sha,
                representative=slugsin,
                master_rows=master_rows.get(run_dir, {}).get(row_id, []),
            )
            scenarios[key] = scenario
        scenario.trial_names.add(trial_name)
    return list(scenarios.values())


def scenario_sort_key(scenario: Scenario) -> tuple[float, str, str, str]:
    times = [
        value
        for row in scenario.master_rows
        for value in [stable_float(row.get("realizability_time_s"))]
        if value is not None
    ]
    mean_time = statistics.fmean(times) if times else float("inf")
    return mean_time, DEMO_LABELS.get(scenario.demo, scenario.demo), scenario.run_name, scenario.row_id


def load_status_by_scenario(out_dir: Path) -> dict[tuple[str, str, str], str]:
    status_by_key: dict[tuple[str, str, str], str] = {}
    summary_csv = out_dir / f"{SUMMARY_BASENAME}_all.csv"
    if not summary_csv.exists():
        return status_by_key
    with summary_csv.open(newline="") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            key = (
                row.get("demo", ""),
                row.get("row_id", ""),
                row.get("slugsin_sha256", ""),
            )
            status_by_key[key] = row.get("well_separation_status", "")
    return status_by_key


def scenario_status(
    scenario: Scenario, status_by_key: dict[tuple[str, str, str], str]
) -> str:
    return status_by_key.get(
        (
            DEMO_LABELS.get(scenario.demo, scenario.demo),
            scenario.row_id,
            scenario.sha256,
        ),
        "",
    )


def filter_scenarios(
    scenarios: list[Scenario],
    args: argparse.Namespace,
    out_dir: Path,
) -> list[Scenario]:
    if args.column_indicator:
        allowed = set(args.column_indicator)
        scenarios = [
            scenario
            for scenario in scenarios
            if scenario_master_value(scenario, "column_indicator") in allowed
        ]
    if args.row_id_regex:
        patterns = [re.compile(pattern) for pattern in args.row_id_regex]
        scenarios = [
            scenario
            for scenario in scenarios
            if any(pattern.search(scenario.row_id) for pattern in patterns)
        ]
    if args.skip_historical_timeout_s is not None:
        threshold = args.skip_historical_timeout_s
        scenarios = [
            scenario
            for scenario in scenarios
            if (historical_max_time_s(scenario) or 0.0) < threshold
        ]

    status_by_key = load_status_by_scenario(out_dir)
    if args.only_status:
        allowed_statuses = set(args.only_status)
        scenarios = [
            scenario
            for scenario in scenarios
            if scenario_status(scenario, status_by_key) in allowed_statuses
        ]
    if args.sample_one_per_scenario:
        grouped: dict[tuple[str, str, str], Scenario] = {}
        for scenario in scenarios:
            key = (
                DEMO_LABELS.get(scenario.demo, scenario.demo),
                scenario_master_value(scenario, "column_indicator"),
                scenario_status(scenario, status_by_key),
            )
            grouped.setdefault(key, scenario)
        scenarios = list(grouped.values())
    return scenarios


def summarize_number(rows: list[dict[str, str]], column: str, op: str) -> str:
    values = [stable_float(row.get(column)) for row in rows]
    values = [value for value in values if value is not None]
    if not values:
        return ""
    if op == "mean":
        return f"{statistics.fmean(values):.9g}"
    if op == "median":
        return f"{statistics.median(values):.9g}"
    if op == "min":
        return f"{min(values):.9g}"
    if op == "max":
        return f"{max(values):.9g}"
    raise ValueError(op)


def safe_name(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", text).strip("_") or "scenario"


def subprocess_output_text(output: str | bytes | None) -> str:
    if output is None:
        return ""
    if isinstance(output, bytes):
        return output.decode(errors="replace")
    return output


def result_json_path(out_dir: Path, scenario: Scenario, ordinal: int) -> Path:
    demo_label = DEMO_LABELS.get(scenario.demo, scenario.demo)
    filename = f"{ordinal:04d}__{safe_name(scenario.row_id)}__{scenario.sha256[:12]}.json"
    return out_dir / "results" / demo_label / filename


def summary_basename(minimize_core: bool) -> str:
    return MINCORE_SUMMARY_BASENAME if minimize_core else SUMMARY_BASENAME


def run_check(
    slugs_bin: str,
    scenario: Scenario,
    json_path: Path,
    timeout_s: float,
    minimize_core: bool,
    force: bool,
    dry_run: bool,
) -> tuple[int | None, float | None, str, dict[str, Any] | None]:
    if dry_run:
        return None, None, "dry_run", None
    if json_path.exists() and not force:
        try:
            return 0, None, "", json.loads(json_path.read_text())
        except json.JSONDecodeError:
            pass
    json_path.parent.mkdir(parents=True, exist_ok=True)
    stdout_path = json_path.with_suffix(".stdout.txt")
    stderr_path = json_path.with_suffix(".stderr.txt")
    cmd = [
        slugs_bin,
        "--checkWellSeparation",
        "--jsonOutput",
    ]
    if minimize_core:
        cmd.append("--minimizeWellSeparationCore")
    cmd.extend([str(scenario.representative), str(json_path)])
    start = time.monotonic()
    try:
        proc = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout_s,
            check=False,
        )
        elapsed = time.monotonic() - start
    except subprocess.TimeoutExpired as exc:
        stdout_path.write_text(subprocess_output_text(exc.stdout))
        stderr_path.write_text(subprocess_output_text(exc.stderr))
        return 124, timeout_s, f"timeout_after_{timeout_s:g}s", None
    stdout_path.write_text(subprocess_output_text(proc.stdout))
    stderr_path.write_text(subprocess_output_text(proc.stderr))
    data = None
    error = ""
    if json_path.exists():
        try:
            data = json.loads(json_path.read_text())
        except json.JSONDecodeError as exc:
            error = f"invalid_json: {exc}"
    elif proc.returncode != 0:
        error = (proc.stderr or proc.stdout).strip().splitlines()[-1:] or ["slugs_failed_without_json"]
        error = error[0]
    return proc.returncode, elapsed, error, data


def load_existing_results_by_hash(
    out_dir: Path, *, minimize_core: bool
) -> dict[str, tuple[Path, dict[str, Any]]]:
    results: dict[str, tuple[Path, dict[str, Any]]] = {}
    for json_path in sorted((out_dir / "results").glob("*/*.json")):
        match = re.search(r"__([0-9a-f]{12})(?:__mincore)?\.json$", json_path.name)
        if not match:
            continue
        try:
            data = json.loads(json_path.read_text())
        except json.JSONDecodeError:
            continue
        if bool(data.get("core_enabled", False)) != minimize_core:
            continue
        results.setdefault(match.group(1), (json_path, data))
    return results


def first_master_row(scenario: Scenario) -> dict[str, str]:
    return scenario.master_rows[0] if scenario.master_rows else {"row_id": scenario.row_id}


def compact_json_list(value: Any) -> str:
    if not value:
        return ""
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def stable_join(values: list[Any]) -> str:
    return ";".join(str(value) for value in values if value not in (None, ""))


def assumption_signature(assumptions: list[dict[str, Any]]) -> str:
    parts = []
    for assumption in assumptions:
        parts.append(
            ":".join(
                [
                    str(assumption.get("kind", "")),
                    str(assumption.get("name", "")),
                    f"line={assumption.get('line', '')}",
                    f"source_index={assumption.get('source_index', '')}",
                ]
            )
        )
    return "|".join(parts)


def assumption_values(assumptions: list[dict[str, Any]], key: str) -> list[Any]:
    return [assumption.get(key, "") for assumption in assumptions]


def core_slugsin_lines(slugsin_file: str, assumptions: list[dict[str, Any]]) -> str:
    if not slugsin_file or not assumptions:
        return ""
    path = Path(slugsin_file)
    if not path.exists():
        return ""
    lines = path.read_text(errors="replace").splitlines()
    result = []
    seen_lines = set()
    for assumption in assumptions:
        line = assumption.get("line")
        if not isinstance(line, int) or line in seen_lines:
            continue
        seen_lines.add(line)
        if 1 <= line <= len(lines):
            result.append(
                {
                    "line": line,
                    "kind": assumption.get("kind", ""),
                    "name": assumption.get("name", ""),
                    "text": lines[line - 1],
                }
            )
    return compact_json_list(result)


def add_core_summary_fields(
    row: dict[str, str],
    slugsin_file: str,
    data: dict[str, Any] | None,
) -> None:
    core_assumptions = data.get("core_assumptions", []) if data else []
    if not isinstance(core_assumptions, list):
        core_assumptions = []
    row["core_assumption_count"] = str(len(core_assumptions)) if data else ""
    row["core_assumption_signature"] = assumption_signature(core_assumptions)
    row["core_assumption_kinds"] = stable_join(assumption_values(core_assumptions, "kind"))
    row["core_assumption_names"] = stable_join(assumption_values(core_assumptions, "name"))
    row["core_assumption_lines"] = stable_join(assumption_values(core_assumptions, "line"))
    row["core_assumption_source_indices"] = stable_join(
        assumption_values(core_assumptions, "source_index")
    )
    row["core_slugsin_lines"] = core_slugsin_lines(slugsin_file, core_assumptions)
    row["core_assumptions"] = compact_json_list(core_assumptions)


def make_summary_row(
    scenario: Scenario,
    json_path: Path,
    rc: int | None,
    wall_time: float | None,
    error: str,
    data: dict[str, Any] | None,
) -> dict[str, str]:
    master = first_master_row(scenario)
    checks = data.get("checks", {}) if data else {}
    timing = data.get("timing", {}) if data else {}
    row: dict[str, str] = {
        "demo": DEMO_LABELS.get(scenario.demo, scenario.demo),
        "run_name": scenario.run_name,
        "representative_trial": sorted(scenario.trial_names)[0] if scenario.trial_names else "",
        "trial_count_for_scenario_hash": str(len(scenario.trial_names)),
        "master_row_count": str(len(scenario.master_rows)),
        "slugsin_sha256": scenario.sha256,
        "slugsin_file": str(scenario.representative),
        "synthesis_realizability_time_s_mean": summarize_number(
            scenario.master_rows, "realizability_time_s", "mean"
        ),
        "synthesis_realizability_time_s_median": summarize_number(
            scenario.master_rows, "realizability_time_s", "median"
        ),
        "synthesis_realizability_time_s_min": summarize_number(
            scenario.master_rows, "realizability_time_s", "min"
        ),
        "synthesis_realizability_time_s_max": summarize_number(
            scenario.master_rows, "realizability_time_s", "max"
        ),
        "synthesis_overall_pipeline_time_s_mean": summarize_number(
            scenario.master_rows, "overall_pipeline_time_s", "mean"
        ),
        "synthesis_cudd_peak_nodes_max": summarize_number(scenario.master_rows, "cudd_peak_nodes", "max"),
        "well_separation_status": str(data.get("status", "")) if data else "",
        "well_separation_complete": str(data.get("complete", "")) if data else "",
        "violated_assumption_type": str(data.get("violated_assumption_type", "")) if data else "",
        "cases": compact_json_list(data.get("cases")) if data else "",
        "responsible_assumption_count": str(len(data.get("responsible_assumptions", []))) if data else "",
        "core_enabled": str(data.get("core_enabled", "")) if data else "",
        "core_complete": str(data.get("core_complete", "")) if data else "",
        "core_candidate_count": str(data.get("core_candidate_count", "")) if data else "",
        "core_solver_calls": str(data.get("core_solver_calls", "")) if data else "",
        "slugs_reported_elapsed_time_s": str(data.get("elapsed_time", "")) if data else "",
        "slugs_timing_total_s": str(timing.get("total", "")),
        "measured_wall_time_s": f"{wall_time:.9g}" if wall_time is not None else "",
        "well_separation_return_code": "" if rc is None else str(rc),
        "well_separation_error": error,
        "warnings": compact_json_list(data.get("warnings")) if data else "",
        "json_result_file": str(json_path),
    }
    for column in SCENARIO_COLUMNS:
        row[column] = master.get(column, "")
    for column in [
        "ap_i",
        "ap_o",
        "env_init_count",
        "sys_init_count",
        "env_trans_count",
        "sys_trans_count",
        "env_liveness_count",
        "sys_liveness_count",
    ]:
        row[column] = master.get(column, "")
    if checks:
        for key, value in checks.items():
            row[f"slugs_check_{key}"] = str(value)
    add_core_summary_fields(row, str(scenario.representative), data)
    return row


def write_csv(
    path: Path,
    rows: list[dict[str, str]],
    base_columns: list[str] | None = None,
) -> None:
    base_columns = SUMMARY_COLUMNS if base_columns is None else base_columns
    extra_columns = sorted({key for row in rows for key in row} - set(base_columns))
    columns = [*base_columns, *extra_columns]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_jsonl(path: Path, rows: list[dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as stream:
        for row in rows:
            stream.write(json.dumps(row, sort_keys=True) + "\n")


def unique_join(rows: list[dict[str, str]], column: str) -> str:
    values = sorted({row.get(column, "") for row in rows if row.get(column, "")})
    return ";".join(values)


def sum_int_column(rows: list[dict[str, str]], column: str) -> int:
    total = 0
    for row in rows:
        value = row.get(column, "")
        try:
            total += int(value)
        except ValueError:
            continue
    return total


def report_group_key(row: dict[str, str]) -> tuple[str, ...]:
    return (
        row.get("demo", ""),
        row.get("column_indicator", ""),
        row.get("spec_name", ""),
        row.get("capability_file", ""),
        row.get("encoding", ""),
        row.get("liveness", ""),
        row.get("pending", ""),
        row.get("well_separation_status", ""),
        row.get("violated_assumption_type", ""),
    )


def make_report_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    grouped: dict[tuple[str, ...], list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        grouped[report_group_key(row)].append(row)

    report_rows: list[dict[str, str]] = []
    for key, group_rows in sorted(grouped.items()):
        (
            demo,
            column_indicator,
            spec_name,
            capability_file,
            encoding,
            liveness,
            pending,
            status,
            violated_type,
        ) = key
        signatures = [
            row.get("core_assumption_signature", "")
            for row in group_rows
            if row.get("core_assumption_signature", "")
        ]
        signature_counts = Counter(signatures)
        signature_summary = [
            {"signature": signature, "count": count}
            for signature, count in sorted(signature_counts.items())
        ]
        report_rows.append(
            {
                "demo": demo,
                "column_indicator": column_indicator,
                "spec_name": spec_name,
                "capability_file": capability_file,
                "encoding": encoding,
                "liveness": liveness,
                "pending": pending,
                "well_separation_status": status,
                "violated_assumption_type": violated_type,
                "scenario_count": str(len(group_rows)),
                "master_row_count": str(sum_int_column(group_rows, "master_row_count")),
                "trial_count": str(sum_int_column(group_rows, "trial_count_for_scenario_hash")),
                "ordering_modes": unique_join(group_rows, "ordering_mode"),
                "ordering_seeds": unique_join(group_rows, "ordering_seed"),
                "row_ids": unique_join(group_rows, "row_id"),
                "core_assumption_count_values": unique_join(group_rows, "core_assumption_count"),
                "distinct_core_signature_count": str(len(signature_counts)),
                "core_assumption_signatures": compact_json_list(signature_summary),
                "core_assumption_kinds": unique_join(group_rows, "core_assumption_kinds"),
                "core_assumption_names": unique_join(group_rows, "core_assumption_names"),
                "core_assumption_lines": unique_join(group_rows, "core_assumption_lines"),
                "core_assumption_source_indices": unique_join(
                    group_rows, "core_assumption_source_indices"
                ),
                "cases_values": unique_join(group_rows, "cases"),
                "json_result_files": unique_join(group_rows, "json_result_file"),
                "slugsin_files": unique_join(group_rows, "slugsin_file"),
            }
        )
    return report_rows


def write_report_files(
    out_dir: Path,
    rows_by_demo: dict[str, list[dict[str, str]]],
    all_rows: list[dict[str, str]],
) -> None:
    for demo_label, rows in rows_by_demo.items():
        report_rows = make_report_rows(rows)
        write_csv(
            out_dir / f"{MINCORE_REPORT_BASENAME}_{demo_label}.csv",
            report_rows,
            REPORT_COLUMNS,
        )
        write_jsonl(out_dir / f"{MINCORE_REPORT_BASENAME}_{demo_label}.jsonl", report_rows)
    report_rows = make_report_rows(all_rows)
    write_csv(out_dir / f"{MINCORE_REPORT_BASENAME}_all.csv", report_rows, REPORT_COLUMNS)
    write_jsonl(out_dir / f"{MINCORE_REPORT_BASENAME}_all.jsonl", report_rows)


def resolve_output_path(out_dir: Path, value: str) -> Path:
    path = Path(value)
    if path.exists():
        return path
    candidate = out_dir / value
    if candidate.exists():
        return candidate
    return path


def row_matches_report_filters(
    row: dict[str, str],
    demo_filter: set[str] | None,
    args: argparse.Namespace,
) -> bool:
    if demo_filter is not None:
        labels = {DEMO_LABELS.get(demo, demo) for demo in demo_filter}
        if row.get("demo", "") not in labels:
            return False
    if args.column_indicator and row.get("column_indicator", "") not in set(args.column_indicator):
        return False
    if args.row_id_regex:
        patterns = [re.compile(pattern) for pattern in args.row_id_regex]
        if not any(pattern.search(row.get("row_id", "")) for pattern in patterns):
            return False
    if args.only_status and row.get("well_separation_status", "") not in set(args.only_status):
        return False
    return True


def run_report_only(
    out_dir: Path,
    summary_name: str,
    demo_filter: set[str] | None,
    args: argparse.Namespace,
) -> int:
    source_csv = out_dir / f"{summary_name}_all.csv"
    if not source_csv.exists():
        print(f"ERROR: existing summary not found: {source_csv}", file=sys.stderr)
        return 2
    filtered_output = any(
        [
            args.demo,
            args.column_indicator,
            args.row_id_regex,
            args.only_status,
        ]
    )

    rows_by_demo: dict[str, list[dict[str, str]]] = defaultdict(list)
    all_rows: list[dict[str, str]] = []
    with source_csv.open(newline="") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            if not row_matches_report_filters(row, demo_filter, args):
                continue
            json_path = resolve_output_path(out_dir, row.get("json_result_file", ""))
            data = None
            if json_path.exists():
                try:
                    data = json.loads(json_path.read_text())
                except json.JSONDecodeError:
                    data = None
            add_core_summary_fields(row, row.get("slugsin_file", ""), data)
            demo_label = row.get("demo", "")
            rows_by_demo[demo_label].append(row)
            all_rows.append(row)

    for demo_label, rows in rows_by_demo.items():
        write_csv(out_dir / f"{summary_name}_{demo_label}.csv", rows)
        write_jsonl(out_dir / f"{summary_name}_{demo_label}.jsonl", rows)
    aggregate_label = "filtered" if filtered_output else "all"
    write_csv(out_dir / f"{summary_name}_{aggregate_label}.csv", all_rows)
    write_jsonl(out_dir / f"{summary_name}_{aggregate_label}.jsonl", all_rows)
    if summary_name == MINCORE_SUMMARY_BASENAME and not filtered_output:
        write_report_files(out_dir, rows_by_demo, all_rows)
    elif summary_name == MINCORE_SUMMARY_BASENAME:
        for demo_label, rows in rows_by_demo.items():
            report_rows = make_report_rows(rows)
            write_csv(
                out_dir / f"{MINCORE_REPORT_BASENAME}_{demo_label}.csv",
                report_rows,
                REPORT_COLUMNS,
            )
            write_jsonl(out_dir / f"{MINCORE_REPORT_BASENAME}_{demo_label}.jsonl", report_rows)
        report_rows = make_report_rows(all_rows)
        write_csv(
            out_dir / f"{MINCORE_REPORT_BASENAME}_{aggregate_label}.csv",
            report_rows,
            REPORT_COLUMNS,
        )
        write_jsonl(out_dir / f"{MINCORE_REPORT_BASENAME}_{aggregate_label}.jsonl", report_rows)

    print(f"Rewrote {len(all_rows)} enriched summary rows from existing JSON results.")
    if summary_name == MINCORE_SUMMARY_BASENAME:
        print(
            f"  {aggregate_label} report: "
            f"{out_dir / f'{MINCORE_REPORT_BASENAME}_{aggregate_label}.csv'}"
        )
    return 0


def normalized_demo_filter(values: list[str] | None) -> set[str] | None:
    if not values:
        return None
    result = set()
    for value in values:
        if value == "drones":
            result.add("drone_demo")
        else:
            result.add(value)
    return result


def main() -> int:
    args = parse_args()
    synthesis_home = args.synthesis_home.expanduser()
    experiments_root = synthesis_home / "experiments"
    out_dir = args.out_dir.expanduser() if args.out_dir else experiments_root / "well_separation_checks"
    demo_filter = normalized_demo_filter(args.demo)

    if not experiments_root.exists():
        print(f"ERROR: experiments directory not found: {experiments_root}", file=sys.stderr)
        return 2

    summary_name = summary_basename(args.minimize_core)
    if args.report_only:
        return run_report_only(out_dir, summary_name, demo_filter, args)

    scenarios = discover_scenarios(experiments_root)
    if demo_filter is not None:
        scenarios = [scenario for scenario in scenarios if scenario.demo in demo_filter]
    scenarios.sort(key=scenario_sort_key)
    scenarios = filter_scenarios(scenarios, args, out_dir)
    if args.limit is not None:
        scenarios = scenarios[: args.limit]

    print(f"Discovered {len(scenarios)} distinct scenario/hash checks.")
    print(f"Output directory: {out_dir}")
    if not scenarios:
        return 0

    rows_by_demo: dict[str, list[dict[str, str]]] = defaultdict(list)
    all_rows: list[dict[str, str]] = []
    cached_results = (
        load_existing_results_by_hash(out_dir, minimize_core=args.minimize_core)
        if not args.force
        else {}
    )
    for index, scenario in enumerate(scenarios, start=1):
        demo_label = DEMO_LABELS.get(scenario.demo, scenario.demo)
        json_path = result_json_path(out_dir, scenario, index)
        if args.minimize_core:
            json_path = json_path.with_name(f"{json_path.stem}__mincore.json")
        print(
            f"[{index}/{len(scenarios)}] {demo_label} "
            f"{scenario.row_id} sha={scenario.sha256[:12]} "
            f"synth_mean={summarize_number(scenario.master_rows, 'realizability_time_s', 'mean') or 'na'}",
            flush=True,
        )
        cached = cached_results.get(scenario.sha256[:12])
        reused_result = "False"
        if cached is not None and not json_path.exists() and not args.dry_run:
            cached_path, data = cached
            json_path.parent.mkdir(parents=True, exist_ok=True)
            json_path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")
            rc, wall_time, error = 0, None, ""
            reused_result = "True"
        else:
            rc, wall_time, error, data = run_check(
                args.slugs_bin,
                scenario,
                json_path,
                args.timeout_s,
                args.minimize_core,
                args.force,
                args.dry_run,
            )
            if data is not None and not args.force:
                cached_results.setdefault(scenario.sha256[:12], (json_path, data))
        row = make_summary_row(scenario, json_path, rc, wall_time, error, data)
        row["well_separation_reused_result"] = reused_result
        rows_by_demo[demo_label].append(row)
        all_rows.append(row)

        if not args.dry_run:
            write_csv(out_dir / f"{summary_name}_{demo_label}.csv", rows_by_demo[demo_label])
            write_jsonl(out_dir / f"{summary_name}_{demo_label}.jsonl", rows_by_demo[demo_label])
            write_csv(out_dir / f"{summary_name}_all.csv", all_rows)
            write_jsonl(out_dir / f"{summary_name}_all.jsonl", all_rows)
            if args.minimize_core:
                write_report_files(out_dir, rows_by_demo, all_rows)

    print("Done.")
    if not args.dry_run:
        for demo_label in sorted(rows_by_demo):
            print(f"  {demo_label}: {out_dir / f'{summary_name}_{demo_label}.csv'}")
            if args.minimize_core:
                print(
                    f"  {demo_label} report: "
                    f"{out_dir / f'{MINCORE_REPORT_BASENAME}_{demo_label}.csv'}"
                )
        print(f"  all: {out_dir / f'{summary_name}_all.csv'}")
        if args.minimize_core:
            print(f"  all report: {out_dir / f'{MINCORE_REPORT_BASENAME}_all.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
