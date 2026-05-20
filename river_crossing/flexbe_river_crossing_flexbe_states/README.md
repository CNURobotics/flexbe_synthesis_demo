# flexbe_river_crossing_flexbe_states

Custom FlexBE execution states for the river-crossing synthesis demonstration.
Both states publish `WGCFStatus` on `/wgcf_status` during execution so the
Pygame visualization can animate the crossing in real time.

## MoveState — single-river crossing

Used by WGCF demo modes (`wgcf_synthesis`, `wgcf_capabilities`, `wgcf_capabilities_parsed`).

**Parameters**
- `item` — character to move with the farmer (`'wolf'`, `'goat'`, `'corn'`, or `None` for farmer alone)
- `tracker` — character whose bank position determines direction (typically `'farmer'`)

**Userdata**
- Input/output: `left`, `right` — lists of characters on each bank

**Outcomes**
- `done` — crossing succeeded; passengers delivered to destination bank
- `canceled` — crossing interrupted; passengers returned to origin bank (retryable)
- `failed` — unrecoverable error

## MoveState2R — two-river crossing

Used by two-rivers demo modes (`two_rivers`, `two_rivers_parsed`, `two_rivers_capabilities`,
`two_rivers_capabilities_parsed`).

**Parameters**
- `item` — character to move, or `None` for farmer alone
- `tracker` — character whose bank position determines origin (typically `'farmer'`)
- `destination` — explicit target bank (`'left'`, `'middle'`, `'right'`), or `None` to infer

**Userdata**
- Input/output: `left`, `middle`, `right` — lists of characters on each bank

**Outcomes** — same as `MoveState` (`done`, `canceled`, `failed`)

**Note:** FlexBE serializes Python `None` as the string `'None'` in generated state machines;
`MoveState2R.__init__` normalizes both forms so generated code works without modification.

## WGCFStatus message

Both states publish on `/wgcf_status` (`flexbe_river_crossing_msgs/WGCFStatus`):

```
string[] left        # characters on the left bank
string[] middle      # characters on the middle bank (two-rivers only)
string[] right       # characters on the right bank
string[] passengers0 # characters in boat 0 (left river)
string[] passengers1 # characters in boat 1 (right river)
float32  position0   # boat 0 position  0.0 = left bank, 1.0 = middle bank
float32  position1   # boat 1 position  0.0 = middle bank, 1.0 = right bank
```
