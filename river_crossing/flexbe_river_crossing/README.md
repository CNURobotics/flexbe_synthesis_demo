# flexbe_river_crossing

Meta-package that aggregates all packages in the river-crossing synthesis demonstration.

Installing or building `flexbe_river_crossing` pulls in the full set of packages needed
to run the demonstration:

| Package | Role |
|---|---|
| `flexbe_river_crossing_msgs` | `WGCFStatus` ROS 2 message definition |
| `flexbe_river_crossing_flexbe_states` | `MoveState` and `MoveState2R` FlexBE states |
| `flexbe_river_crossing_behaviors` | Hand-crafted and example synthesized FlexBE behaviors |
| `flexbe_river_crossing_synthesis` | Launch files, pipeline configs, specs, and request clients |
| `flexbe_river_crossing_visualization` | Pygame visualization node |

See the [river-crossing README](../README.md) for setup instructions and demo modes.
