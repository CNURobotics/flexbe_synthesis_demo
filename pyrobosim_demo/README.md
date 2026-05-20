# PyRoboSim Demonstration

This demonstration uses [PyRoboSim](https://github.com/sea-bass/pyrobosim), a 2D mobile-robot
simulator with a ROS 2 interface, to showcase [FlexBE](https://github.com/FlexBE) Hierarchical
Finite State Machine (HFSM) behaviors and LTL-based GR(1) synthesis of executable HFSMs.

A robot navigates a five-room apartment (dining room, kitchen, trash room, closet, and office),
opening doors as needed, picking up and placing household objects (bread, butter, waste, soda,
snacks), and recharging its battery when running low.  The synthesis demonstration generates a
reactive FlexBE HFSM controller from GR(1) LTL specifications that guarantees delivery of
specified items regardless of action failures.

The hand-crafted behavior [demonstrations](pyrobosim_flexbe_behaviors/README.md) are based on the
[ROSCon '24 Deliberation Technologies Workshop](https://github.com/ros-wg-delib/roscon24-workshop).


## Setup

We assume that you have set up the `flexbe_synthesis` and FlexBE system as described in the
[top-level README](../README.md).

This demonstration has been tested in ROS 2 Jazzy, Kilted, and Rolling on Ubuntu 24.04 with:
```bash
# FlexBE Behavior Engine and FlexBE WebUI 4.1.5+
git clone https://github.com/FlexBE/flexbe_behavior_engine.git
git clone https://github.com/FlexBE/flexbe_webui.git

# FlexBE synthesis framework (v0.0.1)
git clone https://github.com/CNURobotics/flexbe_synthesis.git

# PyRoboSim with ROS 2 interface v4.3.4
git clone https://github.com/sea-bass/pyrobosim.git
```

> **Tip:** To exclude this demonstration from the workspace build, place a `COLCON_IGNORE` file
> in the `pyrobosim_demo/` folder.

### Python setup for PyRoboSim

PyRoboSim requires a custom Python virtual environment.
You can follow the directions on their [docs](https://pyrobosim.readthedocs.io/en/latest/setup.html#local-setup).

If you have defined `WORKSPACE_ROOT` with an existing Python virtual environment that is in your system `PATH`, e.g.:

```bash
VENV_PATH="$WORKSPACE_ROOT/venv/lib/python3.12/site-packages"
if [[ ":$PYTHONPATH:" != *":$VENV_PATH:"* ]]; then
  echo "Add Python virtual environment to current PYTHONPATH"
  export PYTHONPATH="$PYTHONPATH:$VENV_PATH"
fi
```
then activate it and install PyRoboSim's Python dependencies into it:

```bash
source $WORKSPACE_ROOT/venv/bin/activate
cd $WORKSPACE_ROOT/src/pyrobosim
pip3 install setuptools
python3 pyrobosim/setup.py egg_info
pip3 install -r pyrobosim.egg-info/requires.txt
rm -rf pyrobosim.egg-info/
deactivate
cd $WORKSPACE_ROOT
```

### Build

Build the packages from your ROS 2 workspace root:

```bash
colcon build --symlink-install --packages-up-to pyrobosim_flexbe_synthesis
source install/setup.bash
```

Now you are ready to run PyRoboSim.

## Problem Worlds

Five worlds are provided with increasing complexity.
See [`pyrobosim_flexbe_problems/README.md`](pyrobosim_flexbe_problems/README.md) for details.

| Problem | Launch arg | Description |
|---|---|---|
| 0 | `problem_number:=0` | P0 synthesis world; all doors, the pantry, and the fridge are open |
| 1 | `problem_number:=1` | Basic navigation; all doors open, no failures |
| 2 | `problem_number:=2` | Adds closed doors and dumpster requiring open actions |
| 3 | `problem_number:=3` | Adds action failure probabilities (navigate 90%, pick/place 50%, open/close 75%, detect 80%) |
| 4 | `problem_number:=4` | Adds battery consumption; robot must recharge during patrol |

## Running the Demonstrations

To run demonstrations of manually crafted behaviors see [Behaviors](pyrobosim_flexbe_behaviors/README.md).

To run demonstrations of synthesis, see [Synthesis](pyrobosim_flexbe_synthesis/README.md).


## Related Publications

Please cite the following when using the synthesis demonstration:

  * D. C. Conner, J. Luzier, E. R. Faith, W. J. Doyle, A. B. Kooiker, A. J. Farney, and I. G. Conner,
    "Capability-based robot controller synthesis," in Proc. 2026 IEEE Int. Conf.
    Electro/Information Technology (EIT), La Crosse, WI, USA, May 21–23, 2026, to be published.

This work is based on an earlier paper:

  * J. Luzier and D. C. Conner, "Solving the Farmer's Dilemma with FlexBE Using GR(1) Synthesis," SoutheastCon 2024, Atlanta, GA, USA, 2024, pp. 1189–1196, doi: [10.1109/SoutheastCon52093.2024.10500189](https://doi.org/10.1109/SoutheastCon52093.2024.10500189).


Please cite the following when using FlexBE:

  * J. Zutell, D. C. Conner, and P. Schillinger,
    ["ROS 2-Based Flexible Behavior Engine for Flexible Navigation,"](http://dx.doi.org/10.1109/SoutheastCon48659.2022.9764047)
    IEEE SouthEastCon, April 2022.

  * P. Schillinger, S. Kohlbrecher, and O. von Stryk,
    ["Human-Robot Collaborative High-Level Control with Application to Rescue Robotics,"](http://dx.doi.org/10.1109/ICRA.2016.7487442)
    ICRA 2016.

### BibTeX

```bibtex
@inproceedings{FlexBESynthesis26,
  author    = {Conner, David C. and Luzier, Joshua and Faith, Emma R. and Doyle, William J. and Kooiker, Aubrie B. and Farney, Andrew J. and Conner, Ian G.},
  title     = {Capability-based Robot Controller Synthesis},
  booktitle = {Proceedings of the 2026 IEEE International Conference on Electro/Information Technology (EIT)},
  address   = {La Crosse, WI, USA},
  month     = may,
  year      = {2026},
  note      = {To be published}
}
```
