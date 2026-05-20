# pyrobosim_flexbe_problems

PyRoboSim world definitions and simulator runner for the FlexBE PyRoboSim demonstration.
Based on the [ROSCon '24 Deliberation Technologies Workshop](https://github.com/ros-wg-delib/roscon24-workshop).

The world is a five-room apartment — `dining`, `kitchen`, `trash`, `closet`, and `office` —
connected by hallways with openable doors.  Household objects (`bread0`, `butter0`, `waste0`,
`waste1`, `soda0`, `snacks0`) are distributed across the rooms for pick-and-place tasks.

For setup, build instructions, and step-by-step demo execution, see the
[PyRoboSim demo README](../README.md).

---

## Problem Worlds

```bash
ros2 run pyrobosim_flexbe_problems run --ros-args -p problem_number:=<0|1|2|3|4>
```

| Problem | World file | What changes | Typical behaviors |
|---|---|---|---|
| 0 | `world0.yaml` | P0 synthesis world; all doors, the pantry, and the fridge are open | Synthesis demos |
| 1 | `world1.yaml` | All doors open; no failures | `test navigate`, `Test Plan Path`, `test pick place`, `DetectSelect`, `Go Beh` |
| 2 | `world2.yaml` | Several hallways start **closed** | `delib_ws_p2`, `delib_ws_p2_sm`, `Through Door`, `Traverse`, `Patrol` |
| 3 | `world3.yaml` | Adds **action failure probabilities** (navigate 90%, pick/place 50%, open/close 75%, detect 80%) | Same as problem 2 |
| 4 | `world4.yaml` | Adds **battery consumption** per action (nav 0.5%/m, pick/place/open 5–8%) | `PatrolCharge` |

### Problem goals (for synthesis and hand-crafted behaviors)

| Problem | Goal | Initial conditions |
|---|---|---|
| 0 | Bread and butter on the dining table | Bread in pantry (open); butter in fridge (open) |
| 1 | Snacks on the dining table | Snacks in the kitchen pantry |
| 2 | Waste in the closed dumpster | Waste on office desk and bin; trash hallways closed |
| 3 | Bread and butter on the dining table | Bread in pantry (closed); butter in fridge (closed) |
| 4 | Bread and butter on the dining table; waste in the closed dumpster; battery not depleted | Same as problem 3; trash hallways closed |

## World layout

```
 trash ─── closet
   │           │
dining       office
   │           │
    ── kitchen ─
```

The robot starts in `dining`.  Key locations and their synthesis integer indices:

| Location | Room | Index (synthesis) | Notes |
|---|---|---|---|
| `dumpster` | trash | 1 | Openable bin |
| `charger` | closet | 2 | Charging dock |
| `table` | dining | 3 | Flat surface |
| `pantry` | kitchen | 4 | Openable pantry |
| `fridge` | kitchen | 5 | Openable fridge |
| `desk` | office | 6 | Flat surface |
| `bin` | office | 7 | Trash bin |
| `hall_trash_closet` | — | 8 | Hallway door |
| `hall_trash_dining` | — | 9 | Hallway door |
| `hall_trash_kitchen` | — | 10 | Hallway door |
| `hall_closet_dining` | — | 11 | Hallway door |
| `hall_closet_office` | — | 12 | Hallway door |
| `hall_dining_kitchen` | — | 13 | Hallway door |
| `hall_dining_office` | — | 14 | Hallway door |
| `hall_kitchen_office` | — | 15 | Hallway door |

Index 0 encodes the robot's hand (item held).
Hallway indices (8–15) appear in `move_dest` as navigation waypoints but have no
associated environment tracking variable.
Room indices 16–20: trash=16, closet=17, dining=18, kitchen=19, office=20.

## Shared data files

| File | Contents |
|---|---|
| `location_data.yaml` | Location names, types, and nav-pose offsets shared across all worlds |
| `object_data.yaml` | Object names and category definitions shared across all worlds |
