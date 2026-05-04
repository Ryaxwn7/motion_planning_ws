# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Multi-robot navigation and formation (shape assembly) system built on ROS Noetic (Ubuntu 20.04) for Wheeltec `mini_mec` robots. The repo is a catkin workspace with separate host-side and robot-side build and launch flows.

## Build

Workspace root; always source ROS and the local devel first:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

**Full workspace build:**
```bash
catkin_make
```

**Host-side only** (formation_host, fm2_gather, move_base_client, sim_env, formation_msgs):
```bash
./build_host.sh
```

**Robot-side only** (my_planner, my_ware, turn_on_wheeltec_robot, LiDAR drivers, core planners):
```bash
./build_robot.sh
```

The build scripts set `CATKIN_WHITELIST_PACKAGES` to skip the other half. C++ code uses `src/ros_motion_planning/.clang-format` (Google-based, 2-space indent, 120-col, no tabs).

## Run

### Real robots

Host (run once):
```bash
./start_host.sh
```

Each robot (unique `agent_id`):
```bash
./start_robot.sh agent_id:=1
```

Host defaults to manual gather mode. Trigger the gather center computation:
```bash
rostopic pub -1 /gather_signal std_msgs/UInt8 '{data: 2}'
```

### Simulation (Gazebo)

```bash
cd src/ros_motion_planning/scripts
./shape_assembly_5x10_d2.sh agent_number:=4
```

### Configuration files

- `config/host_start.conf` — host-side defaults (master URI, robot IDs, shape type, gather params)
- `config/robot_start.conf` — robot-side defaults (agent id, planners, shape_source)
- `src/ros_motion_planning/src/sim_env/config/shape_assembly*.yaml` — swarm controller params
- `src/fm2_gather/config/gather_param*.yaml` — FM2 gather planner params
- `src/ros_motion_planning/src/user_config/*.yaml` — simulation environment config (world, map, robot count/types)

Parameters use a three-layer priority: CLI launch arg → YAML config → launch file built-in default.

## Architecture

### Host ↔ Robot split

- **Host** runs `roscore`, `map_server`, `formation_host` (Python task supervisor UI), `fm2_gather` (C++ node that computes the optimal gathering center via Fast Marching Method), and optional `move_base_client` monitoring scripts.
- **Robots** each run chassis drivers, LiDAR, AMCL localization, `move_base` (with FM2 global planner + `my_planner` local planner), and `shape_task_bridge` (formation_robot) to bridge shape assembly goals into move_base.

### Key packages

| Package | Role |
|---------|------|
| `fm2_gather` | C++ node computing multi-robot gathering center using FM2 on the costmap. Input: robot poses + global costmap. Output: `/gather_center` PoseStamped. Supports event-based and periodic replanning with cooldown logic. |
| `formation_host` | Python-based host UI (`host_control_ui.py`) and `shape_task_supervisor.py` that orchestrates shape assembly task distribution. |
| `formation_msgs` | Custom ROS messages: `ShapeTask.msg`, `RobotFormationStatus.msg`. |
| `formation_robot` | `shape_task_bridge.py` — runs on each robot, receives shape assembly targets and translates them into move_base goals. |
| `move_base_client` | C++ utilities (`c_plan_monitor` monitors plan execution, `vr_path_transformer` transforms paths) + Python scripts (`start_gather.py` triggers gather, `plan_monitor.py`). |
| `my_planner` | Custom local planner (PID-based) implementing `nav_core::BaseLocalPlanner`. |
| `sim_env` | Launch files, YAML configs, and Gazebo worlds for simulation. |
| `turn_on_wheeltec_robot` | Launch files for robot bring-up: chassis, LiDAR, AMCL, move_base, shape assembly. |

### Planner modules (under `src/ros_motion_planning/src/core/`)

- **Global planners**: FM2 (fast marching), graph_planner, evolutionary_planner, sample_planner, lazy_planner
- **Local planners**: apf, dwa, lqr, mpc, orca, pid, rpp, sfm, static

### Launch file chain (real robot flow)

`motion_navigate_multi4.launch` → starts map_server, chassis, LiDAR, AMCL, move_base, and optionally `shape_assembly_host.launch` (host) / `shape_assembly_agent.launch` (per robot). Each robot runs in its own namespace (`robot1`, `robot2`, ...).

### Shape assembly flow

1. Host publishes a shape image (rectangle, triangle, etc.) via `shape_task_supervisor`
2. `fm2_gather` computes the optimal center point near all robots
3. Robots navigate to staging positions via `move_base` (`use_center_as_goal:=true`)
4. Once near the center, `shape_assembly` swarm controller takes over for precision formation

### Config variants

Files suffixed `_real_simaligned` are for real robot operation. Files suffixed `_simtest` are for Gazebo simulation. The `_d2` suffix (e.g., `user_config_5x10_d2_simtest.yaml`) indicates docking station #2 variant.

## Notes

- `src/ros_motion_planning/scripts/env.sh` must be **sourced** (not executed) before any ROS commands — it cleans inherited ROS overlay paths and sources the workspace setup.
- `src/ros_motion_planning/scripts/main.sh` regenerates generated launch/XML files from `user_config/*.yaml` — prefer editing the YAML, not the generated files.
- The startup scripts use a `.conf` sourcing mechanism: bash-sourced config files set variables, which can be overridden by CLI `key:=value` args.
- Robot `agent_id` determines the namespace (`robot1`, `robot2`, etc.) and must be unique per robot.
- Time synchronization on real robots uses NTP with the host as time server (see `start_robot.sh`).
