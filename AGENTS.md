# Repository Guidelines

## Project Structure & Module Organization
This repository is a ROS Noetic catkin workspace. Top-level `src/` holds source packages; `build/` and `devel/` are generated outputs and should not be edited manually. The main planner code lives in `src/ros_motion_planning`, with algorithms under `src/core/`, simulation assets under `src/sim_env/`, plugins under `src/plugins/`, and user-facing config in `src/user_config/`. Robot bring-up and custom hardware packages live in `src/turn_on_ws/src/`, including `my_planner`, `turn_on_wheeltec_robot`, and LiDAR drivers.

## Build, Test, and Development Commands
From the workspace root:

- `source /opt/ros/noetic/setup.bash && catkin_make`: build all packages in this workspace.
- `catkin_make run_tests && catkin_test_results build/test_results`: run registered catkin tests and summarize results.

From `src/ros_motion_planning/scripts/`:

- `./build.sh`: install Conan-managed third-party dependencies from `../3rd/` and run `catkin_make`.
- `./main.sh`: regenerate launch and XML files from `src/user_config/user_config.yaml` and start `roslaunch sim_env main.launch`.
- `./killpro.sh`: stop the launched simulation processes quickly.
- `doxygen` from `src/ros_motion_planning/`: rebuild API docs into `docs/html/`.

## Coding Style & Naming Conventions
Use ROS package conventions: package, launch, and YAML names stay `lowercase_with_underscores`. C++ headers belong in `include/<package>/`; implementations stay in `src/`. The tracked formatter is `src/ros_motion_planning/.clang-format`: Google-based, 2-space indentation, 120-column limit, no tabs. Run `clang-format -style=file` on touched C++ files before submitting.

## Testing Guidelines
There are currently few tracked unit-test directories, so validate changes with both `catkin_make` and an affected launch flow. For planner changes, smoke-test with `./main.sh`; for hardware packages, launch the relevant file under `src/turn_on_ws/src/turn_on_wheeltec_robot/launch/`. New tests should live with the owning package and be registered with `catkin_add_gtest` or `rostest`.

## Commit & Pull Request Guidelines
Recent history uses short imperative subjects such as `review theta_star and lazy_theta_star planner`, `feat(build): ...`, and `fix(...)`. Keep commits focused by package or feature. PRs should state affected packages, config files changed, validation commands run, and include RViz or Gazebo screenshots when behavior or visualization changes. Prefer editing `src/user_config/*.yaml` over generated launch files, because `main.sh` rewrites them.
