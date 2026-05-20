# flexbe_river_crossing_msgs

ROS 2 messages for the Wolf-Goat-Corn-Farmer river-crossing synthesis demonstration.

## WGCFStatus

Published on `/wgcf_status` by `MoveState` (single river) and `MoveState2R` (two rivers)
during execution so the Pygame visualization can animate the crossing in real time.

```
string[] left        # characters currently on the left bank
string[] middle      # characters on the middle bank (two-rivers modes; empty for single-river)
string[] right       # characters currently on the right bank
string[] passengers0 # characters in boat 0 (left river: left↔middle)
string[] passengers1 # characters in boat 1 (right river: middle↔right; empty for single-river)
float32  position0   # boat 0 position  0.0 = left bank,   1.0 = middle bank
float32  position1   # boat 1 position  0.0 = middle bank, 1.0 = right bank
```

In single-river mode `middle`, `passengers1`, and `position1` are unused (empty / 0.0).
