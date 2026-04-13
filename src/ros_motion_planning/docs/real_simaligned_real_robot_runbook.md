# Real Simaligned 真机运行手册

本文档对应当前工作空间的真实机器人默认链路。

目标：

- 主机和机器人默认行为尽量对齐 `simtest`
- 主机负责聚集点计算、地图融合、任务发布、可视化
- 机器人负责 `move_base` 导航、`shape_assembly` 自主切换和速度控制
- 聚集启动改为人工发布 `/gather_signal = 2`

## 1. 默认入口

主机：

- [start_host.sh](/home/yxw/motion_planning_ws/start_host.sh)
- 默认配置：[config/host_start.conf](/home/yxw/motion_planning_ws/config/host_start.conf)

机器人：

- [start_robot.sh](/home/yxw/motion_planning_ws/start_robot.sh)
- 默认配置：[config/robot_start.conf](/home/yxw/motion_planning_ws/config/robot_start.conf)

回退旧真实参数：

- 主机：[config/host_start.legacy.conf](/home/yxw/motion_planning_ws/config/host_start.legacy.conf)
- 机器人：[config/robot_start.legacy.conf](/home/yxw/motion_planning_ws/config/robot_start.legacy.conf)

## 2. 当前默认链路

### 2.1 主机侧

`start_host.sh` 默认启动：

- `shape_assembly_real_robot.sh`
- `map_server`
- `shape_assembly_host.launch`
- `formation_host.launch`
- `fm2_gather/laun.launch`
- `fm2_map_combine_node`
- `fm2_gather_node`
- `c_plan_monitor`
- `shape_task_supervisor.py`

主机默认负责：

- `/combined_map` 地图融合
- `/gather_center` 聚集中心发布
- `/shape_assembly/task` 编队任务发布
- `/shape_assembly/target_markers` 目标形状 `MarkerArray`
- 自动 `shape_heading` 选择
- `LONG_PATH / GOAL_OCCUPIED / ABORTED` 类重规划触发

### 2.2 机器人侧

`start_robot.sh` 默认启动：

- `turn_on_wheeltec_robot_multi.launch`
- `wheeltec_lidar.launch`
- `amcl_multi.launch`
- `move_base.launch.xml`
- `shape_assembly_agent.launch`
- `formation_robot.launch`
- `shape_task_bridge.py`
- `shape_assembly_swarm.py`

机器人默认负责：

- `AMCL` 定位
- 本机 `move_base`
- 本机 `MyPlanner`
- 接收 `/shape_assembly/task`
- 基于 `/gather_center` 和队形条件切换到 `shape_assembly`
- 计算本机 `cmd_vel`
- 发布 `/<robot>/shape_assembly/status`
- 仅由 marker owner 机器人发布 `/shape_assembly/markers`

## 3. 默认配置文件

主机默认使用：

- [gather_param_real_simaligned.yaml](/home/yxw/motion_planning_ws/src/fm2_gather/config/gather_param_real_simaligned.yaml)

机器人默认使用：

- [shape_assembly_real_simaligned.yaml](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/config/shape_assembly_real_simaligned.yaml)
- [my_local_planner_params_real_simaligned.yaml](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/config/planner/my_local_planner_params_real_simaligned.yaml)
- [global_costmap_params_real_simaligned.yaml](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/config/costmap/global_costmap_params_real_simaligned.yaml)
- [local_costmap_params_real_simaligned.yaml](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/config/costmap/local_costmap_params_real_simaligned.yaml)

配置特点：

- `robot_detect_topic_suffix=/odom_combined`
- `robot_odom_topic_suffix=/odom_combined`
- `use_combined_map=true`
- `publish_combined_occupancy=true`
- `use_center_as_goal=true`
- `control_strategy=move_base_then_shape`
- `shape_target_mode=reference`
- `switch_gray_threshold=0.8`
- `switch_reference_radius_enable=false`
- `auto_shape_heading=true`
- `cmd_smooth_use_odom=true`
- `publish_markers=true`

## 4. 最小启动顺序

### 4.1 主机

在主机 PC：

```bash
cd /home/yxw/motion_planning_ws
./start_host.sh
```

如果要显式指定配置：

```bash
./start_host.sh config:=/home/yxw/motion_planning_ws/config/host_start.conf
```

### 4.2 机器人

在每台机器人树莓派：

```bash
cd /home/yxw/motion_planning_ws
./start_robot.sh
```

如需切换机器人配置文件：

```bash
./start_robot.sh config:=/home/yxw/motion_planning_ws/config/robot_start.conf
```

### 4.3 人工触发聚集

主机和所有机器人都 ready 后，在主机执行：

```bash
rostopic pub -1 /gather_signal std_msgs/UInt8 '{data: 2}'
```

也可以用辅助脚本：

```bash
rosrun move_base_client start_gather.py --wait-started 5.0
```

## 5. 推荐检查项

### 5.1 主机侧

启动后先检查：

```bash
rostopic list | egrep '/combined_map|/gather_center|/shape_assembly/task|/shape_assembly/target_markers'
```

人工触发聚集前，建议确认：

```bash
rostopic echo -n 1 /map
rostopic info /combined_map
```

人工触发聚集后，建议确认：

```bash
rostopic echo -n 1 /gather_started
rostopic echo -n 1 /gather_center
rostopic echo -n 1 /shape_assembly/task
```

### 5.2 机器人侧

以 `robot4` 为例：

```bash
rostopic echo -n 1 /robot4/amcl_pose
rostopic echo -n 1 /robot4/move_base/status
rostopic echo -n 1 /robot4/shape_assembly/staging_goal
rostopic echo -n 1 /robot4/shape_assembly/status
```

如果要确认 `shape_assembly` 已经接管：

```bash
rostopic echo /robot4/shape_assembly/status
```

关注：

- `phase`
- `shape_active`
- `inside_rate`
- `cover_rate`

### 5.3 RViz 观察

主机侧建议观察：

- `/gather_center`
- `/shape_assembly/task`
- `/shape_assembly/target_markers`
- `/shape_assembly/markers`
- `/robotN/move_base/GraphPlanner/plan`
- `/robotN/shape_assembly/status`

说明：

- `/shape_assembly/target_markers` 由主机发布，表示目标队形
- `/shape_assembly/markers` 由 marker owner 机器人发布，表示当前编队状态

## 6. 当前默认参数与旧真实链路的差异

### 6.1 主机侧差异

- 默认 `AUTO_START_GATHER=false`
- 默认开启 `enable_map_combine=true`
- `gather_config` 切到 `gather_param_real_simaligned.yaml`
- 默认发布 `/shape_assembly/target_markers`
- `r_avoid` 从旧链路常见的 `0.60` 收紧到 `0.35`
- `use_combined_map=true`
- `publish_combined_occupancy=true`

影响：

- 主机启动后先待命，不会抢先开始聚集
- 聚集中心计算更接近 `simtest`
- 目标形状可以直接在主机 RViz 中观察

### 6.2 机器人侧差异

- 默认 `shape_assembly_config` 切到 `shape_assembly_real_simaligned.yaml`
- 默认 `move_base` 配置切到 real_simaligned 版本
- 默认 `use_center_as_goal=true`
- `switch_gray_threshold=0.8`
- `switch_reference_radius_enable=false`
- `auto_shape_heading=true`
- `cmd_smooth_use_odom=true`
- `publish_markers=true`

影响：

- 机器人先直接去聚集中心，而不是默认走 staging 点
- 切换到 shape control 的时机更接近仿真
- 队形角度和目标形状由主机统一决定
- 机器人侧局部控制和可视化更接近 `simtest`

## 7. 兼容与回退

如果需要回到旧真实参数链路：

主机：

```bash
./start_host.sh config:=/home/yxw/motion_planning_ws/config/host_start.legacy.conf
```

机器人：

```bash
./start_robot.sh config:=/home/yxw/motion_planning_ws/config/robot_start.legacy.conf
```

也可以直接覆盖单个参数，例如：

```bash
./start_robot.sh use_center_as_goal:=false
./start_host.sh enable_map_combine:=false
```

## 8. 常见问题定位

### 8.1 主机一直没有 `/gather_center`

优先检查：

- 是否已经人工发布 `/gather_signal=2`
- `/combined_map` 是否有 publisher
- `/robotN/odom_combined` 是否存在
- `robot_ids` 和 `agent_number` 是否一致

### 8.2 机器人没有进入编队控制

优先检查：

- `/shape_assembly/task` 是否已发布
- `/gather_center` 是否已发布
- `/robotN/move_base/status` 是否正常
- `/robotN/shape_assembly/status` 中 `shape_active` 是否始终为 `false`

### 8.3 RViz 里只有目标队形没有当前队形

优先检查：

- `/shape_assembly/target_markers` 是否有消息
- `/shape_assembly/markers` 是否有消息
- marker owner 机器人是否在线
- `publish_markers` 是否被显式关掉

### 8.4 机器人远离队伍

优先检查：

- `move_base` 是否反复重规划
- `/shape_assembly/status` 中是否发生反复 `release`
- 主机是否持续重发新的 `/shape_assembly/task`
- AMCL 位姿是否稳定

## 9. 相关文件

- [start_host.sh](/home/yxw/motion_planning_ws/start_host.sh)
- [start_robot.sh](/home/yxw/motion_planning_ws/start_robot.sh)
- [shape_assembly_real_robot.sh](/home/yxw/motion_planning_ws/src/ros_motion_planning/scripts/shape_assembly_real_robot.sh)
- [shape_assembly_host.launch](/home/yxw/motion_planning_ws/src/turn_on_ws/src/turn_on_wheeltec_robot/launch/shape_assembly_host.launch)
- [motion_navigate_multi4.launch](/home/yxw/motion_planning_ws/src/turn_on_ws/src/turn_on_wheeltec_robot/launch/motion_navigate_multi4.launch)
- [formation_host.launch](/home/yxw/motion_planning_ws/src/formation_host/launch/formation_host.launch)
- [formation_robot.launch](/home/yxw/motion_planning_ws/src/formation_robot/launch/formation_robot.launch)
- [shape_task_supervisor.py](/home/yxw/motion_planning_ws/src/formation_host/scripts/shape_task_supervisor.py)
- [shape_task_bridge.py](/home/yxw/motion_planning_ws/src/formation_robot/scripts/shape_task_bridge.py)
- [shape_assembly_swarm.py](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py)
