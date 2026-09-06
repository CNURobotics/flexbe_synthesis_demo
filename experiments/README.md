# Experiment Utilities

This directory contains scripts for reproducing and packaging the release
experiment datasets for the FlexBE synthesis demonstrations. The scripts assume
they are run from a sourced ROS workspace that contains both this repository and
`flexbe_synthesis`.

Most scripts write under `FLEXBE_SYNTHESIS_HOME`, which defaults to
`~/.flexbe_synthesis`. For isolated runs, set it explicitly:

```bash
export FLEXBE_SYNTHESIS_HOME=/tmp/flexbe_synthesis_release_run
```

## Main Workflow

`run_experiment_plan.sh` runs the main experiment set:

- Coffee baseline from `flexbe_synthesis`
- Two Rivers release sweep
- PyRoboSim P0 statistics sweep
- PyRoboSim P1 baseline rows
- Drone demo statistics sweep
- Drone demo Fair-Outcome-with-pending rows

Start from an empty synthesis home:

```bash
./experiments/run_experiment_plan.sh --clear --no-terminal-hold
```

The script records machine information, starts each task, writes task state files
under `$FLEXBE_SYNTHESIS_HOME/run_state`, and stores logs under
`$FLEXBE_SYNTHESIS_HOME/run_logs`.

`weekend_run.sh` runs the main workflow and the follow-up outlier scripts in
sequence. Use it when you want the complete long-running dataset pass:

```bash
./experiments/weekend_run.sh --clear
```

## Per-Demo Scripts

The per-demo wrappers run the same task functions as `run_experiment_plan.sh`,
but select one demonstration and use `MAX_PARALLEL=1` by default. That default
is intentional for a 4-core machine with about 5.5 GB free memory: the larger
PyRoboSim and drone runs are more reliable when they are not competing with
another synthesis-heavy task.

```bash
./experiments/run_coffee.sh --clear
./experiments/run_two_rivers.sh --clear
./experiments/run_pyrobosim.sh --clear
./experiments/run_drone_demo.sh --clear
```

PyRoboSim and the drone demo also have long follow-up rows. Run them only when
you want the complete release dataset for that domain:

```bash
./experiments/run_pyrobosim.sh --clear --with-followups
./experiments/run_drone_demo.sh --clear --with-followups
```

All wrappers pass remaining arguments through to `run_experiment_plan.sh`, so
`--super-long`, `--slow-first`, and environment overrides such as
`FLEXBE_SYNTHESIS_HOME`, `WORKSPACE_ROOT`, and `SETUP_FILE` still work.

`run_experiment_plan.sh` can also run an explicit comma-separated task list:

```bash
./experiments/run_experiment_plan.sh --clear --tasks coffee_full_rerun,two_rivers_full_rerun
```

## Timing Estimates

These estimates are planning values for a modest 4-core x86_64 Linux
workstation with about 5.5 GB of free RAM available to the run, using the ROS
workspace layout assumed by these scripts and Slugs installed at
`/usr/local/bin/slugs`. They are based on the reported result tables in
`main-ar.pdf` and the long-run behavior encoded in the scripts. The table
values report per-trial pipeline timing; end-to-end script time also includes
ROS startup, workspace crawling, Python process overhead, logging, CSV
aggregation, plotting, and any rows intentionally capped as placeholders.

| Script | Workload | Reported per-trial scale | Planning estimate |
|---|---|---:|---:|
| `run_coffee.sh` | 1428 matrix rows, 5 trials each | roughly 0.20-0.25 ms overall per reported cell | 20-35 min |
| `run_two_rivers.sh` | 840 matrix rows, 5 trials each | roughly 0.14-0.94 ms overall per reported cell | 50-70 min |
| `run_pyrobosim.sh` | P0 20-seed statistics sweep plus P1 fixed-ordering rows | P0: 29-764 s; P1: 177 s to about 6.9 h for completed rows | 18-30 h |
| `run_pyrobosim.sh --with-followups` | main PyRoboSim run plus FullP1 retry and 4S/4F ordering sweep | FullP1 reached about 13.1 h overall with an unresolved well-separation check | 30-60 h |
| `run_drone_demo.sh` | 20-seed drone statistics sweep plus capped FP placeholder rows | 23-4409 s overall across the six reported statistics cells | 12-24 h |
| `run_drone_demo.sh --with-followups` | main drone run plus FP and incomplete-1SP retries | FP rows may need day-scale budgets; incomplete 1SP rows may finish quickly or run long | 36-96 h |
| `weekend_run.sh` | all main tasks plus PyRoboSim and drone follow-ups | combines the long rows above sequentially after the main pass | 3-7 days |

The Coffee and Two Rivers estimates are calibrated from end-to-end wrapper
runtime on this hardware class, because runner overhead dominates their small
per-row solver times. The large-domain estimates are intentionally
conservative: the reported means hide high variance from declaration ordering,
and several follow-up rows are useful precisely because they establish that a
configuration exceeds a large budget on this hardware class.

## Follow-Up Scripts

`run_pyrobosim_p1_outliers.sh` retries the long PyRoboSim P1 FullP1
well-separation row with a larger budget.

`run_pyrobosim_p1_stats.sh` adds the PyRoboSim P1 `4S` and `4F` ordering sweep
to the same run directory used by the main workflow.

`run_drone_demo_outliers.sh` retries the drone demo `1FP` and `2FP` rows with a
larger synthesis budget.

`run_drone_demo_stats_1sp_outliers.sh` retries only the drone demo `1SP`
ordering rows whose well-separation checks did not finish in the main run.

`run_drone_demo_long_retry.sh` is an older combined retry helper for `1FP`,
`2FP`, and incomplete `1SP` rows. Prefer the more focused scripts above for new
runs.

`retry_drone_demo_timeouts.py` builds a retry matrix from an existing
`drone_demo_stats/master.csv` by selecting rows with non-zero return codes.

## Repair And Export

`resume_realize_hfsm.py` repairs runs where synthesis finished but HFSM
realization failed or was interrupted. It reuses cached synthesis byproducts
instead of rerunning Slugs.

`run_well_separation_checks.py` scans existing Slugs byproducts and writes
well-separation CSV/JSONL summaries for later aggregation.

`export_release_data.py` copies the canonical completed run directories into a
clean `release_data/` bundle. It exports `master.csv`, `summary.csv`, `tables/`,
and `plots/`, while scrubbing local home-directory paths and hostnames from CSV
cells.

Run a dry pass first:

```bash
python3 experiments/export_release_data.py --dry-run
```

Then export:

```bash
python3 experiments/export_release_data.py
```

Before publishing the bundle, inspect the exported CSV files for local paths or
hostnames and confirm that the canonical run names in `export_release_data.py`
match the dataset you intend to ship.
