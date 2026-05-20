# pyrobosim_flexbe_states

Custom FlexBE `EventState` implementations that wrap PyRoboSim ROS 2 actions and services.

All states use `ProxyActionClient` / `ProxyServiceCaller` from `flexbe_core.proxy` —
shared singleton proxies that must be used instead of bare `rclpy` clients inside states.

All states in this package override every lifecycle method (`on_start`, `on_enter`, `execute`,
`on_exit`, `on_pause`, `on_resume`, `on_stop`) with extra logging for educational purposes.
This is not the norm for production states.

For setup, build instructions, and step-by-step demo execution, see the
[PyRoboSim demo README](../README.md).

---

## States

### CheckDoorState

Queries the pyrobosim `RequestWorldState` service (non-blocking) to check whether a named
openable (door, dumpster, fridge, pantry, …) is open or closed.

| | |
|---|---|
| **Parameters** | `call_timeout`, `wait_timeout`, `service_name`, `robot` |
| **Userdata in** | `name` — name of the openable (e.g. `'hall_trash_closet'`, `'fridge'`) |
| **Userdata out** | `msg` — result message |
| **Outcomes** | `open`, `closed`, `failed` |

---

### DetectLocalObjectsState

Invokes a pyrobosim `ExecuteTaskAction` to `detect` objects at the current location,
then queries `RequestWorldState` to retrieve the detected items.

| | |
|---|---|
| **Parameters** | `filter`, `action_topic`, `server_topic`, `robot_name`, `server_timeout`, `call_timeout` |
| **Userdata out** | `msg`, `items` — list of detected objects |
| **Outcomes** | `done`, `failed` |

---

### DetectObjectsState

Invokes the pyrobosim `DetectObjects` action directly to detect objects at the current location.

| | |
|---|---|
| **Parameters** | `filter`, `action_topic`, `timeout` |
| **Userdata out** | `msg`, `items` — list of detected objects |
| **Outcomes** | `done` (≥1 object found), `nothing` (0 objects), `failed` |

---

### DoorActionState

Performs an `open` or `close` action on the nearest openable at the robot's current location
via `ExecuteTaskAction`.

| | |
|---|---|
| **Parameters** | `action_type` (`'open'` or `'close'`), `robot_name`, `action_topic`, `timeout` |
| **Userdata out** | `msg` |
| **Outcomes** | `done`, `failed` |

---

### FollowPathState

Follows a pre-planned path (received from `PlanPathState`) using the pyrobosim `follow_path` action.

| | |
|---|---|
| **Parameters** | `action_topic`, `server_timeout` |
| **Userdata in** | `path` — `Path` object from a previous `PlanPathState` |
| **Userdata out** | `msg` |
| **Outcomes** | `done`, `failed` |

---

### MonitorBatteryState

Subscribes to the robot state topic and returns an outcome when the battery level
crosses a threshold.  Designed for use inside a `ConcurrencyContainer`.

| | |
|---|---|
| **Parameters** | `low_battery_level` (default 15%), `high_battery_level` (default 105%), `state_topic`, `timeout` |
| **Userdata out** | `battery_level` |
| **Outcomes** | `battery_level` (threshold crossed), `failed` |

---

### NavigateActionState

Navigates to a named target location in one step (combines path planning and following)
using pyrobosim's `ExecuteTaskAction` with action type `navigate`.

| | |
|---|---|
| **Parameters** | `target_location`, `robot_name`, `action_topic`, `server_timeout`, `navigate_timeout` |
| **Userdata out** | `msg` |
| **Outcomes** | `done`, `planning_failed`, `motion_failed`, `canceled`, `timeout` |

> Used directly by synthesis-generated state machines as the primary navigation primitive.

---

### NextRoomState

Reads the robot's current location from the state topic and uses
`WorldConfiguration` (BFS over the hallway graph) to determine the next intermediate
room on the path toward a goal room.

| | |
|---|---|
| **Parameters** | `world_definition`, `package_name`, `sub_folder`, `state_topic`, `timeout` |
| **Userdata in** | `goal` — final destination room name |
| **Userdata out** | `room` — next room to enter, `hallway` — connecting hallway name |
| **Outcomes** | `done`, `failed` |

---

### PickActionState

Picks a named object at the robot's current location via `ExecuteTaskAction`.

| | |
|---|---|
| **Parameters** | `robot_name`, `action_topic`, `timeout`, `object_name` (or `None` to use userdata) |
| **Userdata in** | `object` — object name (used when `object_name` parameter is `None`) |
| **Userdata out** | `msg` |
| **Outcomes** | `done`, `failed` |

---

### PlaceActionState

Places the currently held object at the robot's current location via `ExecuteTaskAction`.

| | |
|---|---|
| **Parameters** | `robot_name`, `action_topic`, `timeout` |
| **Userdata out** | `msg` |
| **Outcomes** | `done`, `failed` |

---

### PlanPathState

Requests a collision-free path to a named goal location via the pyrobosim `plan_path` action.
The resulting `Path` is written to userdata for `FollowPathState`.

| | |
|---|---|
| **Parameters** | `action_topic`, `timeout` |
| **Userdata in** | `goal` — location name or `Pose` |
| **Userdata out** | `msg`, `path` |
| **Outcomes** | `done`, `failed` |
