# pyrobosim_flexbe_utilities

Utility classes for the FlexBE PyRoboSim demonstration.

For setup, build instructions, and step-by-step demo execution, see the
[PyRoboSim demo README](../README.md).

---

## WorldConfiguration

A class-level singleton that loads a PyRoboSim world YAML file and exposes the world
structure to FlexBE states without needing live ROS 2 communication.

Used by `NextRoomState` to compute the next intermediate room when navigating toward
a distant goal.

### Construction

```python
from pyrobosim_flexbe_utilities import WorldConfiguration

wc = WorldConfiguration(
    world_definition='world4',        # YAML basename under sub_folder
    package_name='pyrobosim_flexbe_problems',
    sub_folder='worlds',
)
```

The instance is cached at the class level; subsequent calls with the same arguments return
the same object.

### Key methods

| Method | Returns | Description |
|---|---|---|
| `get_room_for_location(name)` | `str` | Room containing the named location |
| `get_next_room(current, goal)` | `(str, str)` | `(next_room, hallway_name)` — BFS shortest path step; hallway follows the naming convention `hall_{room_a}_{room_b}` |
| `is_location_in_room(loc, room)` | `bool` | True if the location polygon contains the given room's centroid |

### Internals

- Room polygons are loaded from the world YAML and stored as [Shapely](https://shapely.readthedocs.io/) `Polygon` objects for point-in-polygon queries.
- Hallways are modeled as a bidirectional graph; `get_next_room` runs BFS to find the
  shortest-hop path from the robot's current room to the goal room.
- Hallway names follow the convention `hall_{room_start}_{room_end}` (alphabetical order
  within the YAML; both orderings are indexed for lookup).
