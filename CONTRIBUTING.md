# Contributing

Thank you for your interest in contributing to `flexbe_synthesis_demo`!

## How to contribute

1. Fork the repository and create a feature branch from `main`.
2. Make your changes, following the code style described below.
3. Run linting before opening a pull request:
   ```bash
   colcon test --packages-select <package_name>
   colcon test-result --verbose
   ```
4. Open a pull request against `main` with a clear description of the change and why it is needed.

## Code style

The repository uses ROS 2 ament linting conventions for Python packages:

- `ament_flake8` — style and error checking (135-character line limit; see `.flake8`)
- `ament_pep257` — docstring conventions
- `ament_copyright` — Apache-2.0 header on every source file

Additional style notes:
- YAML specs: follow the variable naming convention documented in
  [`flexbe_river_crossing_synthesis/README.md`](river_crossing/flexbe_river_crossing_synthesis/README.md)
  and [`pyrobosim_flexbe_synthesis/README.md`](pyrobosim_demo/pyrobosim_flexbe_synthesis/README.md)
  (same suffixes apply to both demos).
- Add inline comments to spec YAML files for any non-obvious LTL constraints.

## License

Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~
