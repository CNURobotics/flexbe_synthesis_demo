# pyrobosim_flexbe_behaviors

FlexBE behavior state machines for the PyRoboSim synthesis demonstration.

Behaviors are generated and maintained by the FlexBE WebUI.
Manual additions belong only inside `[MANUAL]` tags (`[MANUAL_IMPORT]`, `[MANUAL_INIT]`,
`[MANUAL_CREATE]`, `[MANUAL_FUNC]`) — code outside these tags may be overwritten by
the WebUI generator.

For setup, build instructions, and step-by-step demo execution, see the
[PyRoboSim demo README](../README.md).

To see the synthesis demonstration, see [Synthesis](../pyrobosim_flexbe_synthesis/README.md).

---

## Running the Demos

Each component runs in a separate terminal.  Start the simulator first, then the onboard engine,
then the OCS, then the UI.

```bash
# Terminal 1 — PyRoboSim simulator
ros2 run pyrobosim_flexbe_problems run --ros-args -p problem_number:=1

# Terminal 2 — FlexBE onboard behavior engine (runs on the robot)
ros2 launch flexbe_onboard behavior_onboard.launch.py

# Terminal 3 — FlexBE OCS (operator control station, headless)
ros2 launch flexbe_webui flexbe_ocs.launch.py headless:=true

# Terminal 4 — FlexBE WebUI  (wait for "Begin behavior mirror processing..." in terminal 3)
ros2 run flexbe_webui webui_client
# add --qt_software if rendering is blank

# Terminal 5 — Input action server (required for DetectSelect object selection)
ros2 run flexbe_input input_action_server
```

Load a behavior from the FlexBE **Behavior Dashboard** using **Load Behavior**, then switch to
**Runtime Control** and press **Start Behavior**.

> In `Low` autonomy, click the highlighted outcome label to confirm each transition.
> In `High` or `Full` autonomy, transitions occur automatically without operator confirmation.
>
> **Warning:** An operator can preempt an active state — wait for the state to finish before
> confirming its outcome.

## Demonstrations

The behaviors are listed in order of increasing complexity and correspond to different problem "worlds" in PyRoboSim.

### Simple navigation and manipulation (world 1)

Start the simulator with `problem_number:=1`.

| Behavior | Description |
|---|---|
| `test navigate` | Single navigation to a fixed location |
| `Test Plan Path` | Plan a path then follow it; demonstrates userdata passing |
| `test pick place` | Navigate, pick, navigate, place using behavior parameters |
| `DetectSelect` | Detect objects, let operator select one via `input_action_server`, pick and place |
| `Go Beh` | Navigate to a parameterized target location |

### Door handling, traversal, and patrol (world 2)

Start the simulator with `problem_number:=2`.

| Behavior | Description |
|---|---|
| `delib_ws_p2` | Open doors via behavior parameters, then run `DetectSelect` |
| `delib_ws_p2_sm` | Hierarchical SM: operator selects which door to open, then runs `DetectSelect` |
| `Through Door` | Navigate through a doorway, opening it first if closed |
| `Traverse` | Navigate to any target room, opening intermediate doors using `NextRoomState` planning |
| `Patrol` | Visit every room in sequence using `Traverse`; set to `Full` autonomy after initial guided run |

### Battery management and concurrency (world 4)

Start the simulator with `problem_number:=4`.

| Behavior | Description |
|---|---|
| `PatrolCharge` | Runs `Patrol` inside a `ConcurrencyContainer` alongside a battery monitor; a `PriorityContainer` preempts patrol to recharge when battery drops below 30% |


## Behaviors

The hand-crafted behavior demos are based on the
[ROSCon '24 Deliberation Technologies Workshop](https://github.com/ros-wg-delib/roscon24-workshop).


| Behavior (`self.name`) | File | Description |
|---|---|---|
| `test navigate` | `test_navigate_sm.py` | Single navigation to a fixed location; validates `NavigateActionState` |
| `Test Plan Path` | `test_plan_path_sm.py` | Plans a path then follows it; demonstrates `PlanPathState` → `FollowPathState` userdata chaining |
| `test pick place` | `test_pick_place_sm.py` | Navigate, pick, navigate, place using behavior parameters |
| `DetectSelect` | `detectselect_sm.py` | Detect objects at a location, let the operator select one via `input_action_server`, then pick and place |
| `Go Beh` | `go_beh_sm.py` | Navigate to a parameterized target location; sub-behavior used by `Traverse` |
| `delib_ws_p2` | `delib_ws_p2_sm.py` | Open doors via behavior parameters, then embed `DetectSelect` to pick and place an item |
| `delib_ws_p2_sm` | `delib_ws_p2_sm_sm.py` | Hierarchical variant: nested sub-SM lets operator choose which door to open before running `DetectSelect` |
| `Through Door` | `through_door_sm.py` | Navigate to a doorway; open it if closed, then pass through |
| `Traverse` | `traverse_sm.py` | Navigate to any room across the world, opening intermediate doors as needed; uses `NextRoomState` for BFS planning |
| `Patrol` | `patrol_sm.py` | Visit every room in sequence using `Traverse`; run in `Full` autonomy after initial operator-guided warm-up |
| `PatrolCharge` | `patrolcharge_sm.py` | `ConcurrencyContainer` running `Patrol` alongside a battery monitor; a `PriorityContainer` preempts patrol to recharge when battery drops below 30% |

## Generic FlexBE states used

The `flexbe_states` package (part of `flexbe_behavior_engine`) provides generic states
demonstrated in these behaviors:

| State | Purpose |
|---|---|
| `LogState` | Print a fixed message to terminal and FlexBE WebUI |
| `LogKeyState` | Print a message containing a userdata value |
| `OperatorDecisionState` | Present outcomes to the operator; auto-selects the suggested outcome in `Full` autonomy |
| `SelectionState` | Use `input_action_server` to let the operator select from a list |
| `InputState` | Use `input_action_server` to accept free-form input (string, number, list) |

## Loading a behavior in FlexBE

This package is defined as a `flexbe_behaviors` export in the `package.xml` definition.
In the FlexBE WebUI, use **Load Behavior** from the Behavior Dashboard toolbar to select by name.
