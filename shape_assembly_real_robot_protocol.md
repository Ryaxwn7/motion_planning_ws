# Real-Robot Shape Assembly Integration

> Update: the current canonical split is documented in [host_robot_code_split.md](./host_robot_code_split.md).
> The runtime protocol has moved from "host pushes per-robot goals" to "host publishes shared `ShapeTask`, robot computes local staging goal".

## Goal

This workspace now supports a mixed navigation/formation pipeline:

1. Host publishes `/gather_signal` with `std_msgs/UInt8(data=2)` to request a new gather-center computation.
2. Host-side `fm2_gather` computes the gather center from the current robot poses and publishes it on `/gather_center`.
3. Host chooses `shape_heading` automatically from available map space and publishes it in `ShapeTask`.
4. Each robot receives only the shared `ShapeTask`, then computes its own local staging goal from the task geometry.
5. Each robot keeps using its own local `move_base` until it enters the target shape neighborhood.
6. After entry, the local distributed `shape_assembly` agent cancels that robot's `move_base` and takes over `cmd_vel`.
7. When a new center is published, each robot re-evaluates whether it is still inside the new shape.
8. Robots outside the new shape automatically release back to `move_base`; robots still inside remain in shape mode.

`/shape_assembly/center_goal_cmd` is still supported as an optional external center override, but it is no longer the default way to start a gather run.


## Shape Image Storage

Both host and every robot must store a local `shape_images` directory.

- Host uses local shape images for shape-task generation and automatic heading selection.
- Each robot uses local shape images for local staging-goal computation and local distributed shape control.
- Runtime shape selection still follows the host-published `ShapeTask.shape_type`; local launch `shape_type` is only a startup fallback before the first task arrives.

## Roles

### Host

- Package: `fm2_gather`
- Package: `move_base_client`
- Launch: `src/turn_on_ws/src/turn_on_wheeltec_robot/launch/shape_assembly_host.launch`

Responsibilities:

- Accept a gather-start signal on `/gather_signal` and compute the shared gather center
- Publish the active reference center to `/gather_center`
- Publish the shared `ShapeTask` with `center + shape_type + shape_heading + shape_scale`
- Monitor navigation and trigger gather-center recompute when replanning is needed
- Avoid assigning control authority; high-frequency execution stays on the robots
- Optionally accept a manual center override from `/shape_assembly/center_goal_cmd`

### Robot

- Package: `sim_env`
- Node: `shape_assembly_swarm.py`
- Package: `formation_robot`
- Node: `shape_task_bridge.py`
- Launch include: `src/turn_on_ws/src/turn_on_wheeltec_robot/launch/include/shape_assembly_agent.launch`

Responsibilities:

- Subscribe to the shared `ShapeTask`
- Compute this robot's local staging goal from shared task geometry and local robot id
- Send that goal to the local `move_base`
- Subscribe to all robots' odometry
- Compute local neighbor set from communication radius `r_sense`
- Decide locally whether this robot is in `NAVIGATE` or `SHAPE_ACTIVE`
- Publish only its own `/<robot>/cmd_vel`
- Cancel only its own `/<robot>/move_base/cancel`

## Communication Protocol

### Host -> Robot

- `/gather_signal`
  - Type: `std_msgs/UInt8`
  - Producer: host UI / operator / helper script / host monitor
  - Consumer: `fm2_gather`
  - Meaning: request gather-center computation; `data=2` starts or recomputes gather-center selection

- `/gather_started`
  - Type: `std_msgs/UInt8`
  - Producer: `fm2_gather`
  - Consumer: host monitor / tools
  - Meaning: gather execution state; `0` means idle or recomputing, `1` means current gather task is active

- `/shape_assembly/center_goal_cmd`
  - Type: `geometry_msgs/PoseStamped`
  - Producer: host UI / operator / upper-level task node
  - Consumer: `fm2_gather`
  - Meaning: optional external center override for the next formation task

- `/gather_center`
  - Type: `geometry_msgs/PoseStamped`
  - Producer: `fm2_gather`
  - Consumer: `shape_task_supervisor` and every `shape_assembly_swarm` agent
  - Meaning: authoritative formation reference center used for shared task publication and shape switching

- `/shape_assembly/task`
  - Type: `formation_msgs/ShapeTask`
  - Producer: `shape_task_supervisor`
  - Consumer: robot-local `shape_task_bridge` and `shape_assembly_swarm`
  - Meaning: shared strategic task containing center, shape type, heading, scale, and replan flag

### Robot -> Host

- `/<robot>/shape_assembly/status`
  - Type: `formation_msgs/RobotFormationStatus`
  - Producer: robot-local `shape_assembly_swarm`
  - Consumer: host monitor / tools
  - Meaning: local phase, takeover state, convergence state, and pose summary for each robot

- `/shape_assembly/active_robot_ids`
  - Type: ROS param
  - Producer: marker-owner `shape_assembly_swarm`
  - Consumer: `fm2_gather`, `c_plan_monitor`
  - Meaning: compatibility state used to skip host replanning for robots already in distributed shape control

- `/shape_assembly/stop_path_planning`
  - Type: ROS param
  - Producer: marker-owner `shape_assembly_swarm`
  - Consumer: `fm2_gather`, `c_plan_monitor`
  - Meaning: compatibility stop flag, only asserted when all robots are already in shape mode and `stop_path_planning_on_shape_takeover=true`

- `/<robot>/odom_combined`
  - Type: `nav_msgs/Odometry`
  - Producer: robot state estimation
  - Consumer: `shape_assembly_swarm`, `c_plan_monitor`
  - Meaning: shared state for neighbor discovery, host monitoring, and replan gating

- `/<robot>/mv_state`
  - Type: `std_msgs/UInt8`
  - Producer: `fm2_gather` move-base controller wrapper
  - Consumer: `c_plan_monitor`
  - Meaning: monitor navigation success/abort/preempted state

### Robot -> Robot

There is no explicit peer-to-peer custom message in v1.

Robots communicate implicitly through shared ROS topics:

- Every agent subscribes to every `/<robot>/odom_combined`
- Each agent computes its local neighbor set using `r_sense`
- Agents outside communication range are ignored by the formation controller

This matches the simulation behavior while keeping the real deployment simple.

## State Machine

### `NAVIGATE`

- `move_base` owns motion
- `shape_assembly` does not publish non-zero velocity
- Robot keeps receiving host goals until it enters the target shape neighborhood

### `SHAPE_ACTIVE`

- `shape_assembly` cancels that robot's `move_base`
- A short `cancel_grace_period` is applied before non-zero formation velocity is published
- Robot participates in distributed formation with nearby robots

### Transition: `NAVIGATE -> SHAPE_ACTIVE`

Triggered when either condition becomes true:

- robot gray-value is inside the target shape threshold
- robot is within the reference-shape radius neighborhood

### Transition: `SHAPE_ACTIVE -> NAVIGATE`

Triggered when a new center arrives and the robot is no longer inside the new target shape neighborhood.

## Efficiency Decisions

- `publish_markers=false` by default on real robots
- `monitor_report_enabled=false` by default
- `fm2_gather` disables map combine by default in the real-robot host launch
- Host replanning now skips robots already in `active_robot_ids`
- External center dispatch waits briefly before goal publication so robots can refresh shape state first
- Robot-local staging goals are geometry-aware and derived from `shape_type + shape_heading + shape_scale`; host `staging_radius` is now only an optional override

## Launch Usage

### Start host once

Use on the machine that runs the single ROS master:

```bash
roslaunch turn_on_wheeltec_robot shape_assembly_host.launch agent_number:=4
```

### Start each robot

For each robot:

```bash
roslaunch turn_on_wheeltec_robot motion_navigate_multi4.launch \
  agent_number:=4 agent_id:=1 enable_shape_assembly:=true
```

For the first robot you may also start host from the same launch if needed:

```bash
roslaunch turn_on_wheeltec_robot motion_navigate_multi4.launch \
  agent_number:=4 agent_id:=1 enable_shape_assembly:=true launch_shape_assembly_host:=true
```

### Trigger gather-center computation

```bash
rosrun move_base_client start_gather.py --wait-started 5.0
```

Equivalent raw topic command:

```bash
rostopic pub -1 /gather_signal std_msgs/UInt8 '{data: 2}'
```

### Optional: override the center manually

```bash
rostopic pub -1 /shape_assembly/center_goal_cmd geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}'
```

## Files Added Or Changed

- `src/formation_msgs/msg/ShapeTask.msg`
- `src/formation_msgs/msg/RobotFormationStatus.msg`
- `src/formation_host/scripts/shape_task_supervisor.py`
- `src/formation_host/launch/formation_host.launch`
- `src/formation_robot/scripts/shape_task_bridge.py`
- `src/formation_robot/launch/formation_robot.launch`
- `src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py`
- `src/ros_motion_planning/src/sim_env/config/shape_assembly.yaml`
- `src/ros_motion_planning/src/sim_env/launch/app/shape_assembly.launch.xml`
- `src/ros_motion_planning/src/sim_env/CMakeLists.txt`
- `src/ros_motion_planning/src/sim_env/package.xml`
- `src/fm2_gather/src/fm2_gather_node.cpp`
- `src/fm2_gather/config/gather_param.yaml`
- `src/fm2_gather/launch/laun.launch`
- `src/fm2_gather/CMakeLists.txt`
- `src/move_base_client/src/c_plan_monitor.cpp`
- `src/turn_on_ws/src/turn_on_wheeltec_robot/launch/shape_assembly_host.launch`
- `src/turn_on_ws/src/turn_on_wheeltec_robot/launch/include/shape_assembly_agent.launch`
- `src/turn_on_ws/src/turn_on_wheeltec_robot/launch/motion_navigate_multi4.launch`
