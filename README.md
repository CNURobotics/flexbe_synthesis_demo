# FlexBE Synthesis Demonstrations

This repository provides three demonstrations of Hierarchical Finite State Machine (HFSM) synthesis as described in:

  * D. C. Conner, J. Luzier, E. R. Faith, W. J. Doyle, A. B. Kooiker, A. J. Farney, and I. G. Conner, "Capability-based robot controller synthesis," in Proc. 2026 IEEE Int. Conf. Electro/Information Technology (EIT), La Crosse, WI, USA, May 21–23, 2026, to be published.

The demonstrations use the capability-based synthesis pipeline from
[`flexbe_synthesis`](https://github.com/CNURobotics/flexbe_synthesis.git)
to generate executable [FlexBE](https://github.com/FlexBE) Hierarchical Finite State Machine
behaviors from GR(1) Linear Temporal Logic specifications.

Each demonstration subfolder can be excluded from the workspace build by placing a `COLCON_IGNORE`
file in its directory.

* **[River-crossing demonstrations](river_crossing/README.md)**

  Synthesis and execution of a FlexBE HFSM controller that solves the classic
  Wolf-Goat-Corn-Farmer puzzle, with single-river and two-river variants spanning
  four specification styles.

* **[PyRoboSim demonstration](pyrobosim_demo/README.md)**

  Capability-based synthesis applied to a mobile manipulation scenario in the
  [PyRoboSim](https://github.com/sea-bass/pyrobosim) simulator.

* **[Flexible-drones demonstration](drone_demo/README.md)**

  Synthesis of multi-step aerial behaviors using the portable
  [`flexible_drones`](https://github.com/CNURobotics/flexible_drones) interfaces,
  with support for Crazyflie and PiHawk/ArduCopter deployments.

## Setup

This repository has been tested on ROS 2 Jazzy, Kilted, and Rolling.

From an empty ROS 2 workspace, first clone this repository so the vcs manifest is
available, then import the remaining source dependencies:

```bash
mkdir -p $WORKSPACE_ROOT/src
git clone https://github.com/CNURobotics/flexbe_synthesis_demo.git $WORKSPACE_ROOT/src/flexbe_synthesis_demo
cd $WORKSPACE_ROOT
vcs import --recursive < src/flexbe_synthesis_demo/flexbe_synthesis_demo.repos
```

Or clone the repositories into the `src` folder individually:

| Repository | URL | Branch |
|---|---|---|
| `flexbe_behavior_engine` | https://github.com/FlexBE/flexbe_behavior_engine.git | `ros2-devel` |
| `flexbe_webui` | https://github.com/FlexBE/flexbe_webui.git | `main` |
| `flexbe_synthesis` | https://github.com/CNURobotics/flexbe_synthesis.git | `main` |
| `flexbe_synthesis_demo` | https://github.com/CNURobotics/flexbe_synthesis_demo.git | `main` |

Install ROS and system dependencies:

```bash
cd $WORKSPACE_ROOT
rosdep install --from-paths src --ignore-src -r -y
```

Install the `slugs` GR(1) synthesis binary (once per machine):

```bash
cd $WORKSPACE_ROOT/src/flexbe_synthesis/flexbe_synthesis_slugs/scripts
./install_slugs.sh
```

See [`flexbe_synthesis_slugs/README.md`](https://github.com/CNURobotics/flexbe_synthesis/blob/main/flexbe_synthesis_slugs/README.md)
for custom install locations and environment configuration.

From the workspace root folder, build and source:

```bash
colcon build --symlink-install --packages-up-to flexbe_synthesis_demo
source install/setup.bash
```

The `flexbe_synthesis_demo/` directory contains a lightweight ROS 2 aggregator
package. It contains no nodes or launch files of its own; it exists so
`colcon build --packages-up-to flexbe_synthesis_demo` builds the demonstration
packages and their dependencies as a coherent target.

Install `pygame` for the river crossing visualization:

```bash
sudo apt install python3-pygame
```

Alternatively, install it with `pip3 install pygame`.

For the PyRoboSim demonstration, additional prerequisites are required; see the
[PyRoboSim demo README](pyrobosim_demo/README.md#setup) for setup instructions.


## Related Publications

Please cite the following when using this demonstration:

  * D. C. Conner, J. Luzier, E. R. Faith, W. J. Doyle, A. B. Kooiker, A. J. Farney, and I. G. Conner, "Capability-based robot controller synthesis," in Proc. 2026 IEEE Int. Conf. Electro/Information Technology (EIT), La Crosse, WI, USA, May 21–23, 2026, to be published.

This synthesis work builds on:

  * J. Luzier and D. C. Conner, "Solving the Farmer's Dilemma with FlexBE Using GR(1) Synthesis," SoutheastCon 2024, Atlanta, GA, USA, 2024, pp. 1189–1196, doi: [10.1109/SoutheastCon52093.2024.10500189](https://doi.org/10.1109/SoutheastCon52093.2024.10500189).

  * J. W. M. Hayhurst and D. C. Conner, ["Towards Capability-Based Synthesis of Executable Robot Behaviors,"](http://dx.doi.org/10.1109/SECON.2018.8479047) IEEE SoutheastCon 2018, St. Petersburg, FL, USA.

  * S. Maniatopoulos, P. Schillinger, V. Pong, D. C. Conner and H. Kress-Gazit, ["Reactive high-level behavior synthesis for an Atlas humanoid robot,"](http://dx.doi.org/10.1109/ICRA.2016.7487613) ICRA 2016, Stockholm, Sweden.

  * R. Ehlers and V. Raman, [Slugs: Extensible GR(1) Synthesis](https://doi.org/10.1007/978-3-319-41540-6_19), CAV 2016.

Please cite the following when using FlexBE:

  * J. Zutell, D. C. Conner and P. Schillinger, ["ROS 2-Based Flexible Behavior Engine for Flexible Navigation,"](http://dx.doi.org/10.1109/SoutheastCon48659.2022.9764047) IEEE SouthEastCon, April 2022.

  * P. Schillinger, S. Kohlbrecher, and O. von Stryk, ["Human-Robot Collaborative High-Level Control with Application to Rescue Robotics,"](http://dx.doi.org/10.1109/ICRA.2016.7487442) ICRA 2016.

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

## License

The repository is released primarily under the Apache License 2.0; see
[LICENSE](LICENSE). Some PyRoboSim demonstration files are derived from or
compatible with BSD-3-Clause code and carry `SPDX-License-Identifier:
BSD-3-Clause`; see [LICENSE-BSD-3-Clause](LICENSE-BSD-3-Clause). Individual
source files and package manifests identify the applicable license.
