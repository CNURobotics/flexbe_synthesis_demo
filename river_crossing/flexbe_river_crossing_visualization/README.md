# Pygame-based visualization for flexbe_synthesis_demo

Pygame-based visualization for the river-crossing synthesis demonstration.
Subscribes to `/wgcf_status` (`flexbe_river_crossing_msgs/WGCFStatus`) and animates
the Wolf-Goat-Corn-Farmer puzzle — characters on banks, boat in motion, and passengers
in transit — in real time as the FlexBE behavior executes.

## Running

```bash
# Single-river (WGCF) modes
ros2 run flexbe_river_crossing_visualization river_crossing

# Two-rivers modes
ros2 run flexbe_river_crossing_visualization river_crossing --two-rivers
```

`--two-rivers` switches to the three-bank background and enables the second boat
(middle↔right river).  Use it with any `two_rivers*` launch/request pair.

See the [River Crossing README](../README.md) for full execution instructions.

## Modules

| Module | Description |
|---|---|
| `river_crossing.py` | Entry point. `RiverCrossing` drives the Pygame main loop, polls the message queue, and renders each frame. |
| `msg_handler.py` | `MsgHandler` runs a background thread that spins a ROS 2 subscriber on `/wgcf_status` and feeds messages into a queue for the main loop. |
| `image_manager.py` | `ImageManager` loads character and background images, computes bank/boat positions, and draws each frame element. |
| `sound_manager.py` | `SoundManager` loads and plays sound effects (splash, success). |
