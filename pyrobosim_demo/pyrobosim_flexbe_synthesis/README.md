# FlexBE Synthesis - PyRoboSim Demonstrations

This package provides launch files, pipeline configurations, GR(1) specifications, and capability definitions
for the PyRoboSim HFSM synthesis demonstrations as described in

  * D. C. Conner, J. Luzier, E. R. Faith, W. J. Doyle, A. B. Kooiker, A. J. Farney, and I. G. Conner,
    "Capability-based robot controller synthesis," in Proc. 2026 IEEE Int. Conf.
    Electro/Information Technology (EIT), La Crosse, WI, USA, May 21–23, 2026, to be published.

The synthesis pipeline connects to `flexbe_synthesis_core` via a `FlexBESynthesis` ROS 2 action
and generates a reactive [FlexBE](https://github.com/FlexBE) HFSM controller that delivers
specified household items to target locations, regardless of action failures.

For setup and build instructions, see the [Base Setup](../../README.md).
For step-by-step setup of the PyRoboSim demonstration environment, see the
[PyRoboSim demo README](../README.md).


## FlexBE Configuration for Synthesis

To use synthesis, set the **system name** in the FlexBE WebUI configuration tab so the synthesized
behavior is associated with the correct pipeline output.

Set the system name to match the `system_name` used when launching
the synthesis server (e.g., `p0_full_spec_demo`).

## Synthesis Demonstration

**Quick start** — full-spec synthesis for problem P0, world 0:

```bash
# Terminal 1 — PyRoboSim (e.g., world 0 with all doors open)
ros2 run pyrobosim_flexbe_problems run --ros-args -p problem_number:=0

# Terminal 2 — Synthesis server
ros2 launch pyrobosim_flexbe_synthesis p0_full_spec.launch.py

# Terminal 3 — Request synthesis (full-spec goals are encoded in the YAML)
ros2 run pyrobosim_flexbe_synthesis request_synthesis \
    --ros-args -p system_name:=p0_full_spec_demo
```

The request client triggers the full synthesis pipeline (LTL compilation, slugs synthesis,
automaton reduction, FlexBE state-machine generation) and saves the output to the configured
`synthesis_home` directory, keyed by `system_name` and `spec_name`.

The synthesis process can take several minutes.

Request goals are ignored by full-spec pipelines.  To override goals, relaunch Terminal 2
with a capability pipeline (e.g., `p0_capabilities.launch.py`) and then request:

```bash
ros2 run pyrobosim_flexbe_synthesis request_synthesis \
    --ros-args -p system_name:=p0_capabilities_demo \
    -p goals:='["bread0=3", "butter0=3"]'
```

## Launching with FlexBE

Start terminals 1 and 2 as above (PyRoboSim and synthesis server).

Then start terminals 3 and 4:

```bash
# Terminal 3 - Robot onboard behavior
ros2 launch flexbe_onboard behavior_onboard.launch.py

# Terminal 4 - FlexBE Operator Control Station
ros2 launch flexbe_webui flexbe_ocs.launch.py
```

### Loading and running the synthesized behavior

1. In the FlexBE WebUI, create a **New Behavior** on the Dashboard.
   - Set the system name in the **Configuration** tab to match `system_name`.
2. Add a `StateMachine` container and tick **Display Synthesis**
    * Enter goals for capability-based synthesis (e.g., `bread0=3, butter0=3`).
    * Full-spec pipelines encode goals in the YAML, so request goals are ignored.
    * Confirm the system name (e.g., `p0_full_spec_demo`)
3. Click **Synthesize** to populate the container with the generated HFSM.
4. Open the container to inspect the synthesized state machine.
   - Use `Shift + Left-click Drag` to pan the view.
5. Define `object` userdata on the Behavior Dashboard and add it as an input key
   on the container.
   - This is set by states at runtime, but it must be declared before execution.
6. Save the behavior and switch to **Runtime Control** to execute.

> Synthesized behaviors use `Low` autonomy per transition by default; set runtime autonomy to `High` or `Full` for transitions to occur automatically without operator confirmation.

> **Note:** The synthesized strategy is correct but not path-optimal — slugs may choose
> an arbitrary valid first move (e.g., toward the dumpster) before heading to goal items.
> To constrain the initial destination, uncomment the `SYS_INIT` line in the spec:
> `#- "move_dest=butter0 | move_dest=bread0"  # move to one of sub goals first`

---

## Demo Modes

Seven synthesis modes are provided, pairing a problem variant (P0 or P1) with a
specification style and pipeline variant.
Use [`startup.txt`](startup.txt) as a quick command reference.

| World | Spec Variant | Pipeline | Launch file | `system_name` | `spec_name` | Spec file (`spec_path`) | Expected synthesis time |
|---|---|---|---|---|---|---|---|
| P0 | Move-only spec | `processes_def.yaml` | `p0_move_only_spec.launch.py` | `p0_move_only_spec_demo` | `p0_move_only_spec` | `p0_move_only_spec.yaml` | 10 s |
| P0 | Full spec | `processes_def.yaml` | `p0_full_spec.launch.py` | `p0_full_spec_demo` | `p0_full_spec` | `p0_full_spec.yaml` | 400 s |
| P0 | Capabilities | `capability_processes_def.yaml` | `p0_capabilities.launch.py` | `p0_capabilities_demo` | `p0_capabilities` | `p0_capabilities.yaml` | 700 s |
| P0 | Capabilities parsed | `capability_processes_def_parsed.yaml` | `p0_capabilities_parsed.launch.py` | `p0_capabilities_parsed_demo` | `p0_parsed` | `p0_capabilities.yaml` | 200 s |
| P1 | Full spec | `processes_def.yaml` | `p1_full_spec.launch.py` | `p1_full_spec_demo` | `p1_full_spec` | `p1_full_spec.yaml` | 6100 s |
| P1 | Capabilities | `capability_processes_def.yaml` | `p1_capabilities.launch.py` | `p1_capabilities_demo` | `p1_capabilities` | `p1_capabilities.yaml` | Not solvable within 10 hours |
| P1 | Capabilities parsed | `capability_processes_def_parsed.yaml` | `p1_capabilities_parsed.launch.py` | `p1_capabilities_parsed_demo` | `p1_parsed` | `p1_capabilities.yaml` | 5400 s |

Each mode stores its synthesis outputs in a subdirectory of `synthesis_home` keyed by `system_name/spec_name`.
The launch defaults use unique `system_name` values so modes can be tested without sharing output directories.
Expected synthesis times are based on successful test runs with margin.

### Pre-built automaton modes

Six usable pre-synthesized automata are distributed with the repository in `slugs/` and can be
loaded directly without running slugs.  This is useful for demonstration and development when
skipping the full synthesis wait.  Use `load_automaton.launch.py` with a `prefix` argument:

```bash
ros2 launch pyrobosim_flexbe_synthesis load_automaton.launch.py prefix:=<prefix>
```

The launcher derives `system_name`, `spec_path`, `capabilities_path`, and `automaton_path`
from the prefix.  In the FlexBE WebUI Configuration tab, enter the matching `system_name`
(equal to the prefix) and `spec_name`:

| `prefix` / `system_name` | `spec_name` | Automaton file |
|---|---|---|
| `p0_full_spec_demo` | `p0_full_spec` | `slugs/p0_full_spec_demo_automaton.yaml` |
| `p0_move_only_spec_demo` | `p0_move_only_spec` | `slugs/p0_move_only_spec_demo_automaton.yaml` |
| `p0_capabilities_demo` | `p0_capabilities` | `slugs/p0_capabilities_demo_automaton.yaml` |
| `p0_capabilities_parsed_demo` | `p0_parsed` | `slugs/p0_capabilities_parsed_demo_automaton.yaml` |
| `p1_full_spec_demo` | `p1_full_spec` | `slugs/p1_full_spec_demo_automaton.yaml` |
| `p1_capabilities_parsed_demo` | `p1_parsed` | `slugs/p1_capabilities_parsed_demo_automaton.yaml` |

`p1_capabilities_demo` is intentionally not listed: the available artifact is an empty
timeout/failed-synthesis placeholder, not a usable pre-built automaton.

See [`startup.txt`](startup.txt) for complete command sequences.

### `spec_name` vs `spec_path` — two independent roles

These two parameters are **intentionally decoupled** and serve entirely different purposes:

- **`spec_path`** (server-side, set in the launch file) — the filesystem path to the YAML file
  containing the GR(1) constraints that the synthesizer reads.  This determines *what is
  synthesized* and is fixed at server launch time.

- **`spec_name`** (request-side, sent by `request_synthesis`) — a label used to name the output
  subdirectory (`<synthesis_home>/<system_name>/<spec_name>/`) and Mealy-graph files.  It
  determines *where synthesis results are written*, not which spec is run.

The `_parsed` variants illustrate this: both `p0_capabilities_parsed.launch.py` and
`p0_capabilities.launch.py` set `spec_path` to `p0_capabilities.yaml` (identical spec content),
but their requests use different `spec_name` values (`p0_parsed` vs `p0_capabilities`) so their
outputs land in separate directories.

For the documented PyRoboSim demos, `request_synthesis` can derive `spec_name` from
`system_name` when only `system_name` is overridden.  For example,
`system_name:=p1_capabilities_parsed_demo` maps to `spec_name:=p1_parsed`.

The full-spec pipeline (`processes_def.yaml`) does not merge request goals into the GR(1)
specification; its goals are encoded directly in `p0_full_spec.yaml`, `p1_full_spec.yaml`,
and `p0_move_only_spec.yaml`.  Request goals are synthesized dynamically only by the
capability pipelines, which run `slugs_request_specification`.

> The spec loader cross-checks the `spec_name:` field inside the loaded YAML against the
> requested `spec_name` and logs a warning on mismatch, but this is non-fatal — `spec_path`
> always controls which file is read.

### Quick start (P0 full spec)

```bash
# Terminal 1 — synthesis server
ros2 launch pyrobosim_flexbe_synthesis p0_full_spec.launch.py

# Terminal 2 — synthesis request (full-spec goals are encoded in the YAML)
ros2 run pyrobosim_flexbe_synthesis request_synthesis \
    --ros-args -p system_name:=p0_full_spec_demo

# Override goals for capability pipelines, not full-spec pipelines
ros2 run pyrobosim_flexbe_synthesis request_synthesis \
    --ros-args -p system_name:=p0_capabilities_demo \
    -p goals:='["bread0=3", "butter0=3"]'
```

### Capabilities quick start (P0)

```bash
ros2 launch pyrobosim_flexbe_synthesis p0_capabilities.launch.py
ros2 run pyrobosim_flexbe_synthesis request_synthesis \
    --ros-args -p system_name:=p0_capabilities_demo
```

### Pre-built automaton quick start

Load a distributed pre-built automaton to skip synthesis entirely.
Choose any `prefix` from the [Pre-built automaton modes](#pre-built-automaton-modes) table.
Start PyRoboSim as in the Synthesis Demonstration above (Terminal 1), then:

```bash
# Terminal 2 — Load pre-built automaton (example: P0 full spec)
ros2 launch pyrobosim_flexbe_synthesis load_automaton.launch.py prefix:=p0_full_spec_demo

# Terminal 3 — Request behavior generation from the loaded automaton
ros2 run pyrobosim_flexbe_synthesis request_synthesis \
    --ros-args -p system_name:=p0_full_spec_demo
```

### Test timeout override

The production pipeline data sets `synthesis_timeout_s` to 720 seconds by default.
The `p1_full_spec` and `p1` capabilities scenarios may require a larger per-request or test timeout.
The `request_synthesis` node also exposes `synthesis_timeout_s` as a ROS parameter,
defaulting to `0.0`, which leaves the pipeline-configured timeout unchanged. Pass a
larger value in the action request for modes whose expected synthesis time exceeds
the pipeline default.

### Test output directory

Synthesis outputs (automaton YAML, FlexBE state-machine files, Mealy `.dot` graphs)
are written to the standard FlexBE synthesis home during test runs so overnight results
are preserved for inspection.  Test runs prepend `test_<system_name>` to the normal
synthesis home so parallel workers do not share `workspace_defn.yaml` or other
preprocessing artifacts. Set `FLEXBE_SYNTHESIS_HOME` to redirect the collection root:

```bash
export FLEXBE_SYNTHESIS_HOME=~/my_synthesis_outputs
```

If unset, outputs go to
`~/.flexbe_synthesis/test_<system_name>/<system_name>/<spec_name>/`.
For example, the P0 full-spec test writes under
`~/.flexbe_synthesis/test_p0_full_spec_demo/p0_full_spec_demo/p0_full_spec/`.
On GitHub CI the directory is ephemeral (runner home), so CI passes without any
persistent-storage requirement.  ROS logs remain in pytest's temporary directory.

Graphviz image rendering is disabled by default (`draw_graph: false`); Mealy `.dot`
files are still written and can be rendered manually:

```bash
dot -Tpng -O ~/.flexbe_synthesis/test_p0_full_spec_demo/p0_full_spec_demo/p0_full_spec/p0_full_spec.dot
```

The demo-pair tests use a 10 second synthesis timeout by default so normal CI stays
short:

```bash
colcon test --packages-select pyrobosim_flexbe_synthesis
```

For overnight validation, override the test timeout with either pytest args or
an environment variable.

For a single test:
```bash
colcon test --packages-select pyrobosim_flexbe_synthesis \
    --pytest-args --pyrobosim-synthesis-timeout-s=7200
```

For a batch test of the workspace, use the environment variable form:
```bash
PYROBOSIM_SYNTHESIS_TIMEOUT_S=7200  colcon test
```

### Parallel test execution

The eight demo-pair tests can run in parallel via `pytest-xdist` (`pip3 install pytest-xdist`).  Each test spawns
its own isolated child process, so workers do not share file-system state.  With a
7200 s timeout, parallel execution reduces total wall time from roughly 9 hours budgeted
sequentially to about 3 hours budgeted in parallel.

Parallelism is opt-in to avoid leaving lingering ROS/DDS state that can interfere
with other packages under `colcon test`.  Pass `-n auto` (or `-n 8`) explicitly:

```bash
PYROBOSIM_SYNTHESIS_TIMEOUT_S=7200 colcon test \
    --packages-select pyrobosim_flexbe_synthesis \
    --pytest-args " -n" auto
```

The leading space before `-n` is required — colcon's `--pytest-args` parser drops
dash-prefixed tokens it mistakes for its own options.

---

## Specifications (`specs/`)

Spec YAMLs define the GR(1) constraints for environment assumptions and system guarantees.
Full-spec modes provide all five sections by hand; capability modes provide only the
environment side and let the pipeline auto-generate the system side from capabilities.

### Variable naming convention

| Suffix | Meaning | Section |
|---|---|---|
| *(none)* | World-state variable | `INPUT` |
| `_a` | System **action** — activates a FlexBE state | `OUTPUT` |
| `_c` | Action **completed** — confirmed by outcome `done` | `INPUT` |
| `_f` | Action **failed** — triggered by outcome `failed`/`canceled` | `INPUT` |

### P0 full spec (`specs/p0_full_spec.yaml`)

Simplified variant: all doors are assumed always open.

Integer variables:
- `robot_loc:1...20` — robot location (locations 1–7 are named spots; 16–20 are rooms)
- `butter0:0...7`, `bread0:0...7`, etc. — item locations (0 = robot's hand)

Key constraints:
- `ENV_TRANS` ties move completions to `robot_loc` updates; grab completions set item to 0 (held).
- `SYS_TRANS` enforces mutual exclusion of actions and failure handling (`log_*_failed_a` fired on `_f`).
- The full-spec goal is encoded in `SYS_TRANS`: `bread0=3` and `butter0=3` are required before
  `finished`.
- `SYS_LIVENESS` is adversarial: the system wins when the goal is achieved *or* when an
  activated action is never completed (forcing the environment to eventually deliver an outcome).

### P0a move-only spec (`specs/p0_move_only_spec.yaml`)

Minimal move-only P0 sanity case: tracks robot location, move outcomes, and finished state only.

### P1 full spec (`specs/p1_full_spec.yaml`)

Extends P0 with door-state input variables, enabling synthesis of controllers that
explicitly open doors before navigating through them.

Like P0 full spec, the goal is encoded in the YAML: `bread0=3` and `butter0=3` are required
before `finished`.  Request `goals` are ignored by this pipeline.

### P0/P1 capabilities specs (`specs/p0_capabilities.yaml`, `specs/p1_capabilities.yaml`)

Omit `INPUT` / `OUTPUT` sections — these are auto-generated from the capabilities definition.
Only `ENV_INIT`, `SYS_INIT`, `ENV_TRANS`, and a small amount of `SYS_TRANS` glue are written
by hand.  The request goal stays dynamic: a synthesis request supplies world-state goals such
as `bread0=3` and `butter0=3`, and `slugs_request_specification` routes those goals through
the `log_finished` capability.  The specs guard `finished` so it can only become true after
`log_finished_c'`, preventing tiny controllers that report success without executing the
request-goal logging path.

---

## Capabilities (`capabilities/`)

Capability YAMLs map synthesis variables and action names to concrete FlexBE state
implementations and their parameters.  The pipeline uses these to auto-generate `SYS_TRANS`
and `SYS_LIVENESS` constraints; only the environment side of the spec is supplied by hand.

### `capabilities/pyrobosim.yaml`

Maps the full-spec variables to `NavigateActionState`, `PickActionState`, `PlaceActionState`,
and logging states.  Parameters with the `@variable` syntax are resolved at runtime from the
synthesized integer variable mapping (e.g., `target_location: '@move_dest'`).

### `capabilities/p0_capabilities.yaml` / `capabilities/p1_capabilities.yaml`

Capability-specific variants matched to the P0 and P1 world structures.

### Custom mappings (`mappings/custom_mappings.yaml`)

`state_outcome_mappings` normalize state outcome names to synthesis-compatible identifiers
(e.g., `motion_failed` → `failure`, `planning_failed` → `failure`) so that the synthesizer
can treat different failure modes uniformly.

---

## Pipeline Variants (`pipeline/`)

| File | Used by | Liveness step |
|---|---|---|
| `preprocesses_def.yaml` | All modes (runs once at server startup) | — |
| `processes_def.yaml` | Full-spec modes | *(liveness in hand-written `SYS_LIVENESS`)* |
| `capability_processes_def.yaml` | Capability modes | `slugs_system_goal_liveness` |
| `capability_processes_def_parsed.yaml` | Parsed-capability modes | `slugs_system_goal_liveness` |
| `load_processes_def.yaml` | Load pre-built | skips synthesis; loads saved automaton |

**Pre-process steps** (`preprocesses_def.yaml`): validate slugs install, crawl workspace for
FlexBE states, load state mappings, parse capabilities, generate transition relations,
generate discrete abstraction.

**Process steps** (varies by pipeline): load slugs spec, compile, run synthesizer,
generate Mealy graph, reduce automaton, generate FlexBE state machine, lay out states.

**System-goal liveness** (`slugs_system_goal_liveness`): the controller wins if either the
goal is achieved *or* the environment fails to deliver an outcome to an activated state,
modeling an uncooperative environment.  This is the correct liveness assumption for reactive
synthesis.

The synthesis pair tests require successful results to contain at least four generated states.
This catches degenerate capability controllers that satisfy liveness through an immediate
success escape rather than a meaningful task plan.

**Pre-built automaton load** (`load_processes_def.yaml`): skips synthesis entirely and loads
a previously saved automaton YAML directly into the FlexBE state-machine generator.
Useful for re-generating the behavior after a state-name or mapping change without re-running slugs.
Six usable pre-built automata are distributed in `slugs/`; see [Pre-built automaton modes](#pre-built-automaton-modes)
for the full table and launch command.
The load launcher derives `system_name`, `spec_path`, `capabilities_path`, and `automaton_path`
from the `prefix` argument unless those launch arguments are explicitly overridden.
