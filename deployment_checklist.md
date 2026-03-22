# 部署文档

## 1. 文档目标

这份文档用于指导当前工作空间的实际部署。

目标是把系统分成两部分：

- 主机：负责聚集中心选择、队形任务发布、全局监控、重规划触发
- 机器人：负责本机导航、本机避障、本机编队控制、本机任务执行

---

## 2. 部署原则

### 2.1 主机职责

主机只负责低频、全局、战略层逻辑：

- 运行 `roscore`
- 发布地图
- 运行 `fm2_gather`
- 自动选择 `shape_heading`
- 发布共享 `ShapeTask`
- 监控各机器人导航状态
- 触发重新规划

主机不参与高频控制，不直接输出各机器人的 `cmd_vel`。

### 2.2 机器人职责

每台机器人负责本机高频执行层逻辑：

- 底盘驱动
- 雷达驱动
- AMCL
- 本机 `move_base`
- 本机局部路径规划
- 本机 `shape_task_bridge`
- 本机 `shape_assembly_swarm`

机器人直接执行主机发布的共享任务，但本机独立完成导航与最终队形形成。

---

## 3. 主机部署清单

### 3.1 主机必须放置的包

主机工作空间中至少需要包含以下包：

- `formation_host`
- `fm2_gather`
- `move_base_client`
- `formation_msgs`
- `sim_env`
- `turn_on_wheeltec_robot`

### 3.2 主机必须有的资源

- 地图文件
- `shape_images`

默认 `shape_images` 路径：

- `src/ros_motion_planning/src/sim_env/shape_images`

说明：

- 主机本地必须有这份目录
- 主机不会去读取机器人文件系统中的 `shape_images`

### 3.3 主机侧核心文件

- [shape_assembly_host.launch](/home/yxw/motion_planning_ws/src/turn_on_ws/src/turn_on_wheeltec_robot/launch/shape_assembly_host.launch)
- [formation_host.launch](/home/yxw/motion_planning_ws/src/formation_host/launch/formation_host.launch)
- [shape_task_supervisor.py](/home/yxw/motion_planning_ws/src/formation_host/scripts/shape_task_supervisor.py)
- [fm2_gather_node.cpp](/home/yxw/motion_planning_ws/src/fm2_gather/src/fm2_gather_node.cpp)
- [c_plan_monitor.cpp](/home/yxw/motion_planning_ws/src/move_base_client/src/c_plan_monitor.cpp)

### 3.4 主机侧默认建议参数

- `agent_number:=4`
- `shape_source:=mat`
- `shape_type:=rectangle`
- `shape_scale:=1.0`
- `shape_library_root:=$(find sim_env)/shape_images`
- `staging_radius:=0.0`
- `auto_shape_heading:=true`
- `auto_shape_heading_map_topic:=/map`
- `auto_shape_heading_angle_step_deg:=15.0`
- `auto_shape_heading_obstacle_threshold:=80`
- `auto_shape_heading_unknown_is_obstacle:=true`
- `auto_shape_heading_shape_stride:=2`
- `auto_shape_heading_min_improve:=0.01`
- `auto_shape_heading_yaw_bias:=0.02`
- `auto_shape_heading_oob_is_obstacle:=true`
- `publish_center_goal_only:=true`
- `robot_namespace_prefix:=robot`
- `robot_detect_topic_suffix:=/odom_combined`
- `robot_odom_topic_suffix:=/odom_combined`
- `external_center_goal_topic:=/shape_assembly/center_goal_cmd`（仅在需要手动覆盖中心时使用）
- `external_center_sync_wait:=0.5`
- `replan_mode:=event`
- `periodic_interval:=5.0`
- `cooldown:=0.5`
- `per_robot_cooldown:=0.5`
- `max_len_growth_ratio:=1.4`
- `long_path_replan_min_length:=3.0`
- `format_radius:=0.3`
- `enable_stable_len_replan:=false`
- `use_goal_occupied_peer_fallback:=true`
- `goal_occupied_radius:=0.35`
- `loop_hz:=10.0`

### 3.5 主机启动命令

```bash
roslaunch turn_on_wheeltec_robot shape_assembly_host.launch \
  agent_number:=4 \
  shape_type:=rectangle \
  shape_source:=mat \
  shape_scale:=1.0 \
  shape_library_root:=$(rospack find sim_env)/shape_images \
  auto_shape_heading:=true
```

---

## 4. 机器人部署清单

### 4.1 每台机器人必须放置的包

每台机器人工作空间中至少需要包含以下包：

- `turn_on_wheeltec_robot`
- `ros_motion_planning`
- `my_planner`
- `my_ware`
- `formation_robot`
- `formation_msgs`
- `lslidar_driver`
- `lslidar_msgs`

### 4.2 每台机器人必须有的资源

每台机器人本地也必须有一份 `shape_images`：

- `src/ros_motion_planning/src/sim_env/shape_images`

说明：

- 机器人运行时不会去读取主机磁盘中的 `shape_images`
- 机器人本机计算 `staging goal` 和本机运行 `shape_assembly` 都依赖本地 shape 模型

### 4.3 机器人侧核心文件

- [motion_navigate_multi4.launch](/home/yxw/motion_planning_ws/src/turn_on_ws/src/turn_on_wheeltec_robot/launch/motion_navigate_multi4.launch)
- [shape_assembly_agent.launch](/home/yxw/motion_planning_ws/src/turn_on_ws/src/turn_on_wheeltec_robot/launch/include/shape_assembly_agent.launch)
- [formation_robot.launch](/home/yxw/motion_planning_ws/src/formation_robot/launch/formation_robot.launch)
- [shape_task_bridge.py](/home/yxw/motion_planning_ws/src/formation_robot/scripts/shape_task_bridge.py)
- [shape_assembly_swarm.py](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py)
- [shape_assembly.yaml](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/config/shape_assembly.yaml)

### 4.4 机器人侧默认建议参数

#### 导航入口参数

- `map_started:=true`
- `agent_number:=4`
- `agent_id:=1`
- `global_planner:=fm2`
- `local_planner:=my`
- `robot:=mini_mec`
- `enable_shape_assembly:=true`
- `shape_source:=mat`
- `shape_type:=rectangle`
- `use_center_as_goal:=false`

说明：

- `use_center_as_goal:=false` 是默认推荐值
- 这表示机器人先导航到本机计算出的 `staging goal`
- 如果你希望机器人直接把聚集中心当作导航目标，再改成 `true`

#### `formation_robot` 相关参数

- `robot_namespace:=robot1`
- `agent_id:=1`
- `agent_number:=4`
- `distributed_marker_owner:=robot1`
- `shape_source:=mat`
- `shape_type:=rectangle`
- `shape_library_root:=$(find sim_env)/shape_images`
- `shape_resolution:=80`
- `ring_inner_ratio:=0.25`
- `ring_outer_ratio:=0.4`
- `gray_width:=4`
- `task_topic:=/shape_assembly/task`
- `robot_status_topic:=shape_assembly/status`
- `use_center_as_goal:=false`

#### `shape_assembly` 相关参数

这些来自 [shape_assembly.yaml](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/config/shape_assembly.yaml)，第一次部署建议先保持默认：

- `r_avoid: 0.35`
- `r_sense: 1.5`
- `shape_source: mat`
- `shape_type: rectangle`
- `formation_task_topic: /shape_assembly/task`
- `robot_status_topic: shape_assembly/status`
- `reference_center_topic: /gather_center`
- `reference_center_wait: true`
- `control_strategy: move_base_then_shape`
- `shape_target_mode: reference`
- `cancel_move_base_on_switch: true`
- `cancel_grace_period: 0.35`
- `stop_path_planning_on_shape_takeover: false`
- `switch_use_reference_shape: true`
- `switch_reference_radius_enable: true`
- `switch_reference_radius_margin: 0.25`
- `switch_reference_radius_min: 0.0`
- `use_local_costmap_avoid: true`
- `auto_shape_heading: false`
- `publish_markers: false`
- `shape_black_threshold: 1.0e-6`

### 4.5 机器人启动命令

```bash
roslaunch turn_on_wheeltec_robot motion_navigate_multi4.launch \
  map_started:=true \
  agent_number:=4 \
  agent_id:=1 \
  global_planner:=fm2 \
  local_planner:=my \
  robot:=mini_mec \
  enable_shape_assembly:=true \
  shape_source:=mat \
  shape_type:=rectangle \
  use_center_as_goal:=false
```

如果你要测试“直接去聚集中心”模式：

```bash
roslaunch turn_on_wheeltec_robot motion_navigate_multi4.launch \
  map_started:=true \
  agent_number:=4 \
  agent_id:=1 \
  enable_shape_assembly:=true \
  use_center_as_goal:=true
```

---

## 5. 运行时 shape 选择规则

这是部署时最容易混淆的一点。

### 5.1 实际运行规则

机器人运行时实际使用的 shape，以主机下发的 `ShapeTask.shape_type` 为准。

也就是说：

- 主机决定本次任务用什么队形
- 机器人收到任务后，按主机下发的 `shape_type` 切换

### 5.2 本地 launch 里的 `shape_type` 是什么作用

机器人本地 launch 里的 `shape_type` 只作为：

- 节点刚启动、但还没收到第一个 `ShapeTask` 之前的回退值

它不是最终权威来源。

---

## 6. `shape_images` 部署规则

主机和每台机器人都必须各自保存本地 `shape_images`。

### 6.1 为什么主机也需要

主机需要本地 `shape_images` 用于：

- 自动选取 `shape_heading`
- 生成与评估 shape 任务

### 6.2 为什么机器人也需要

机器人需要本地 `shape_images` 用于：

- 本机计算 `staging goal`
- 本机运行 `shape_assembly` 分布式控制

### 6.3 结论

不要假设机器人能直接读取主机磁盘中的 shape 资源。

必须保证：

- 主机有本地 `shape_images`
- 每台机器人也有本地 `shape_images`
- 各机器上的 `shape_images` 内容一致

---

## 7. 推荐部署顺序

### 第一步：主机部署

确认主机具备：

- 工作空间完整
- `shape_images` 完整
- 地图文件完整
- `shape_assembly_host.launch` 可启动

### 第二步：单机器人部署

先只启动 1 台机器人，验证：

- 能连接 ROS master
- 能收到 `/shape_assembly/task`
- 能收到 `/gather_center`
- 能正确发本机 `move_base` 目标
- 能启动本机 `shape_assembly`

### 第三步：双机器人部署

再上 2 台机器人，验证：

- 命名空间无冲突
- 两台都能收到主机任务
- 模式切换正常
- 机器人之间 odom 可见

### 第四步：四机器人部署

最后再扩大到实际规模，重点看：

- 重规划是否稳定
- 进入 shape 的切换是否平滑
- `move_base` 和 `shape_assembly` 是否出现抢控制

---

## 8. 上线前检查清单

### 主机检查

- [ ] 工作空间已编译
- [ ] `shape_images` 存在
- [ ] 地图文件存在
- [ ] `shape_assembly_host.launch` 参数已确认
- [ ] `auto_shape_heading:=true`
- [ ] `agent_number` 与机器人数量一致
- [ ] ROS master 正常

### 每台机器人检查

- [ ] 工作空间已编译
- [ ] `shape_images` 存在
- [ ] 雷达驱动可启动
- [ ] 底盘驱动可启动
- [ ] `motion_navigate_multi4.launch` 参数已确认
- [ ] `agent_id` 唯一
- [ ] `robot_namespace` 唯一
- [ ] 能看到主机的 `/shape_assembly/task`
- [ ] 能看到主机的 `/gather_center`

---

## 9. 首次部署推荐配置

如果你是第一次把这套系统真正放到现场，建议直接用下面这组：

### 主机

- `shape_type:=rectangle`
- `shape_source:=mat`
- `shape_scale:=1.0`
- `auto_shape_heading:=true`
- `staging_radius:=0.0`
- `replan_mode:=event`

### 机器人

- `global_planner:=fm2`
- `local_planner:=my`
- `enable_shape_assembly:=true`
- `use_center_as_goal:=false`

原因：

- `rectangle` 最容易观察队形朝向是否合理
- `auto_shape_heading:=true` 能先利用主机自动适配可用空间
- `use_center_as_goal:=false` 更符合当前分层设计，能减少多机器人直接拥到中心的风险

如果现场空间很宽、你只想先做最简单验证，再临时改成：

- `use_center_as_goal:=true`

---

## 10. 相关文档

- [host_robot_code_split.md](/home/yxw/motion_planning_ws/host_robot_code_split.md)
- [shape_assembly_real_robot_protocol.md](/home/yxw/motion_planning_ws/shape_assembly_real_robot_protocol.md)


---

## 11. 完整多机器人实验流程

下面给出一套可直接执行的 `4` 机器人实验流程。

约定：

- ROS Master 在主机
- 主机 IP 写作 `<HOST_IP>`
- 机器人 IP 分别写作 `<ROBOT1_IP>`、`<ROBOT2_IP>`、`<ROBOT3_IP>`、`<ROBOT4_IP>`
- 所有机器都已经完成编译并执行过 `source devel/setup.bash`

### 11.1 主机终端 1：启动 `roscore`

```bash
export ROS_MASTER_URI=http://<HOST_IP>:11311
export ROS_IP=<HOST_IP>
source /opt/ros/noetic/setup.bash
cd /home/yxw/motion_planning_ws
source devel/setup.bash
roscore
```

### 11.2 主机终端 2：启动地图

```bash
export ROS_MASTER_URI=http://<HOST_IP>:11311
export ROS_IP=<HOST_IP>
source /opt/ros/noetic/setup.bash
cd /home/yxw/motion_planning_ws
source devel/setup.bash
rosrun map_server map_server $(rospack find turn_on_wheeltec_robot)/map/exp_d2.yaml
```

### 11.3 主机终端 3：启动主机战略层

```bash
export ROS_MASTER_URI=http://<HOST_IP>:11311
export ROS_IP=<HOST_IP>
source /opt/ros/noetic/setup.bash
cd /home/yxw/motion_planning_ws
source devel/setup.bash

roslaunch turn_on_wheeltec_robot shape_assembly_host.launch \
  agent_number:=4 \
  shape_type:=rectangle \
  shape_source:=mat \
  shape_scale:=1.0 \
  shape_library_root:=$(rospack find sim_env)/shape_images \
  auto_shape_heading:=true \
  replan_mode:=event
```

### 11.4 机器人终端：分别启动 `robot1` 到 `robot4`

`robot1`：

```bash
export ROS_MASTER_URI=http://<HOST_IP>:11311
export ROS_IP=<ROBOT1_IP>
source /opt/ros/noetic/setup.bash
cd /home/yxw/motion_planning_ws
source devel/setup.bash

roslaunch turn_on_wheeltec_robot motion_navigate_multi4.launch \
  map_started:=true \
  agent_number:=4 \
  agent_id:=1 \
  global_planner:=fm2 \
  local_planner:=my \
  robot:=mini_mec \
  enable_shape_assembly:=true \
  shape_source:=mat \
  shape_type:=rectangle \
  use_center_as_goal:=false
```

`robot2`：

```bash
export ROS_MASTER_URI=http://<HOST_IP>:11311
export ROS_IP=<ROBOT2_IP>
source /opt/ros/noetic/setup.bash
cd /home/yxw/motion_planning_ws
source devel/setup.bash

roslaunch turn_on_wheeltec_robot motion_navigate_multi4.launch \
  map_started:=true \
  agent_number:=4 \
  agent_id:=2 \
  global_planner:=fm2 \
  local_planner:=my \
  robot:=mini_mec \
  enable_shape_assembly:=true \
  shape_source:=mat \
  shape_type:=rectangle \
  use_center_as_goal:=false
```

`robot3`：

```bash
export ROS_MASTER_URI=http://<HOST_IP>:11311
export ROS_IP=<ROBOT3_IP>
source /opt/ros/noetic/setup.bash
cd /home/yxw/motion_planning_ws
source devel/setup.bash

roslaunch turn_on_wheeltec_robot motion_navigate_multi4.launch \
  map_started:=true \
  agent_number:=4 \
  agent_id:=3 \
  global_planner:=fm2 \
  local_planner:=my \
  robot:=mini_mec \
  enable_shape_assembly:=true \
  shape_source:=mat \
  shape_type:=rectangle \
  use_center_as_goal:=false
```

`robot4`：

```bash
export ROS_MASTER_URI=http://<HOST_IP>:11311
export ROS_IP=<ROBOT4_IP>
source /opt/ros/noetic/setup.bash
cd /home/yxw/motion_planning_ws
source devel/setup.bash

roslaunch turn_on_wheeltec_robot motion_navigate_multi4.launch \
  map_started:=true \
  agent_number:=4 \
  agent_id:=4 \
  global_planner:=fm2 \
  local_planner:=my \
  robot:=mini_mec \
  enable_shape_assembly:=true \
  shape_source:=mat \
  shape_type:=rectangle \
  use_center_as_goal:=false
```

### 11.5 主机终端 4：触发一次聚集中心计算

```bash
export ROS_MASTER_URI=http://<HOST_IP>:11311
export ROS_IP=<HOST_IP>
source /opt/ros/noetic/setup.bash
cd /home/yxw/motion_planning_ws
source devel/setup.bash

rosrun move_base_client start_gather.py --wait-started 5.0
```

等价的底层话题命令：

```bash
rostopic pub -1 /gather_signal std_msgs/UInt8 '{data: 2}'
```

如果你这次实验明确要人为指定中心，而不是让 `fm2_gather` 自己算，才使用：

```bash
rostopic pub -1 /shape_assembly/center_goal_cmd geometry_msgs/PoseStamped \
'{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}'
```

### 11.6 实验过程中建议观察

主机看：

```bash
rostopic echo /gather_center
rostopic echo /shape_assembly/task
```

看 `robot1` 的本机状态：

```bash
rostopic echo /robot1/shape_assembly/staging_goal
rostopic echo /robot1/shape_assembly/status
```

### 11.7 预期运行流程

- 主机发布 `/gather_signal`，请求开始聚集中心计算
- `fm2_gather` 根据当前机器人位姿计算聚集中心
- 主机自动选择 `shape_heading`
- 主机发布 `/shape_assembly/task`
- 每台机器人本机 `move_base` 导航到自己的 `staging goal`
- 机器人靠近目标区域后，本机 `shape_assembly` 接管
- 多机器人分布式完成最终队形形成

### 11.8 结束实验

建议按这个顺序停止：

1. 先停所有机器人 launch
2. 再停主机 `shape_assembly_host.launch`
3. 再停 `map_server`
4. 最后停 `roscore`

### 11.9 可选对照实验

如果你要测试“直接导航到聚集中心”模式，把机器人侧启动参数改成：

- `use_center_as_goal:=true`

如果你要测试“人为指定聚集中心覆盖 `fm2_gather` 自动计算”的模式，则额外发布：

```bash
rostopic pub -1 /shape_assembly/center_goal_cmd geometry_msgs/PoseStamped \
'{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}'
```

其它参数先保持不变即可。
