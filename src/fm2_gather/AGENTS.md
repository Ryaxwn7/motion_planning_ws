# Repository Guidelines

## Project Structure & Module Organization
- `src/`: C++ ROS nodes and library sources (e.g., `fm2_gather_node.cpp`, `move_base_controller.cpp`).
- `include/`: public headers and the `fastmarching/` library tree.
- `msg/`: custom ROS messages (`*.msg`) generated via catkin.
- `launch/`: ROS launch files (`gather.launch`, `map_combine.launch`, `record.launch`).
- `config/`: runtime parameters (e.g., `config/gather_param.yaml`).
- `script/`: helper Python scripts (e.g., `script/path_record.py`).
- `data/`: data assets used by demos or runs.

## Build, Test, and Development Commands
Run these from the ROS workspace root (the folder that contains `src/`):
- `catkin_make`: build the whole workspace, including `fm2_gather`.
- `catkin_make --pkg fm2_gather`: build only this package.
- `catkin build fm2_gather`: alternative if you use `catkin_tools`.
- `roslaunch fm2_gather gather.launch`: start the main node with defaults.
- `roslaunch fm2_gather map_combine.launch`: run the map combine pipeline.
- `roslaunch fm2_gather record.launch`: record path data.

## Coding Style & Naming Conventions
- C++17 is required (see `CMakeLists.txt`); keep code compatible with ROS Kinetic+.
- Follow existing formatting in `src/` and `include/` (braces on the same line, compact spacing).
- File naming favors lowercase with underscores (e.g., `fm2_gather_node.cpp`).
- Public headers live under `include/` and should be included with the package prefix.

## Testing Guidelines
- No automated tests are defined yet (the gtest section in `CMakeLists.txt` is commented out).
- If you add tests, place them under `test/` and register with `catkin_add_gtest`.
- Use descriptive test names like `test_<feature>_<behavior>.cpp`.

## Commit & Pull Request Guidelines
- Git history shows no established convention beyond `init`; use clear, imperative messages
  like `Add map combine node` or `Fix goal selection`.
- PRs should include: a short description, how to run the change, and any launch
  files or parameters used. Add screenshots or rosbag notes when behavior changes.

## Configuration & Runtime Notes
- Parameters live in `config/gather_param.yaml`; update this when adding tunables.
- New ROS messages must be listed in `msg/` and added to `add_message_files` in
  `CMakeLists.txt` and dependencies in `package.xml`.
