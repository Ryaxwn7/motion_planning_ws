# Simtest 聚集与 Shape Assembly 工程流程说明

## 1. 文档范围

本文描述当前工作区里 `simtest` 链路的完整运行流程，覆盖以下内容：

- 启动入口和 launch 组织方式
- FM2 聚集中心计算流程
- `move_base` 聚集导航与重规划逻辑
- `shape_assembly_swarm` 的初始化、控制项、切换条件和接管逻辑
- 关键 topic、参数和状态机

本文主要对应当前仿真入口：

- `src/ros_motion_planning/scripts/shape_assembly_5x10_d2.sh`
- `src/ros_motion_planning/src/sim_env/launch/shape_assembly_with_main_simtest.launch`

真实机器人链路 `./start_robot.sh` 在附录中单独说明差异。

## 2. 总体链路

当前 `simtest` 的运行链可概括为：

1. `shape_assembly_5x10_d2.sh` 生成并启动仿真 launch。
2. `main_simtest.launch` 拉起 Gazebo、机器人模型、`move_base`、RViz。
3. `laun_simtest.launch` 拉起：
   - `fm2_map_combine_node`
   - `fm2_gather_node`
   - `c_plan_monitor`
4. `shape_assembly_simtest.launch.xml` 拉起 `shape_assembly_swarm.py`。
5. 外部向 `/gather_signal` 发布 `std_msgs/UInt8(data=2)`。
6. `fm2_gather_node` 计算聚集中心 `/gather_center`，并向各机器人发送 `move_base` 目标。
7. 机器人先由 `move_base` 靠近聚集区。
8. `shape_assembly_swarm.py` 根据灰度进入条件或参考半径条件，将机器人从 `move_base` 切换到 shape control。
9. shape assembly 控制项叠加后持续输出 `cmd_vel`，直到队形收敛。

## 3. 启动链路

### 3.1 脚本入口

入口脚本为 `shape_assembly_5x10_d2.sh`。

它做四件事：

1. 清理旧的 Gazebo / robot 节点。
2. 调用 `main_generate_simtest.py`，用 `user_config_5x10_d2_simtest.yaml` 生成仿真 launch。
3. 从 `shape_assembly_simtest.yaml` 读取关键参数，并透传到 launch。
4. 启动 `shape_assembly_with_main_simtest.launch`。

当前脚本默认透传的关键聚集参数包括：

- `use_fm2_gather`
- `enable_map_combine`
- `publish_center_goal_only`
- `gather_center_topic`
- `fm2_auto_detect_robots`
- `fm2_robot_detect_topic_suffix`
- `replan_mode`
- `periodic_interval`
- `cooldown`
- `per_robot_cooldown`
- `max_len_growth_ratio`
- `long_path_replan_min_length`
- `format_radius`
- `use_goal_occupied_peer_fallback`
- `goal_occupied_radius`

### 3.2 顶层 launch 组织

`shape_assembly_with_main_simtest.launch` 负责把仿真环境、FM2 gather 和 shape assembly 串起来：

- `main_simtest.launch`
  负责 Gazebo 世界、地图、机器人、`move_base_simtest.launch.xml`
- `laun_simtest.launch`
  负责 `fm2_map_combine_node`、`fm2_gather_node`、`c_plan_monitor`
- `shape_assembly_simtest.launch.xml`
  负责 `shape_assembly_swarm.py`

当前 simtest 里 `enable_map_combine` 默认已经为 `true`，因为 `fm2_gather_node` 使用的是 `/combined_map`，没有这个节点就不会进入聚集中心计算。

## 4. 聚集流程的触发条件

### 4.1 `/gather_signal`

`fm2_gather_node` 订阅 `/gather_signal`。

当前约定：

- `data = 2`
  启动一次“全局聚集中心计算”
- `data = 10 + robot_id`
  触发某台机器人单独重规划

代码中：

- `signal_callback()` 在收到 `2` 时将 `start_compute = true`
- 主循环检测到 `start_compute` 后进入 `gather()`

### 4.2 `/gather_started`

`fm2_gather_node` 还会发布 `/gather_started`：

- `0`
  当前还没有进入“聚集导航已下发”的状态
- `1`
  本轮聚集中心和导航目标已经下发

`c_plan_monitor` 会用这个状态判断是否已经进入 gather 阶段。

### 4.3 为什么会卡在 “waiting for /gather_center”

`shape_assembly_swarm.py` 当前配置里 `reference_center_wait: true`，所以在收到 `/gather_center` 之前不会初始化 shape assembly。

这意味着：

- 如果没有发布 `/gather_signal = 2`，会一直等待
- 如果 `/combined_map` 没有 publisher，`fm2_gather_node` 即使收到了 `2`，也不会真正开始计算

## 5. FM2 聚集中心计算

### 5.1 输入

`gather()` 的输入主要有：

- 机器人当前位姿 `robot_poses_`
- 地图 `occ.occupancyGrid`
- 合并地图 `/combined_map` 或基础地图 `/map`
- 每个机器人局部 costmap 提供的动态障碍信息
- 任务类型 `mission`

当前支持三类 mission：

- `fastest`
- `energy`
- `space`

### 5.2 地图准备

计算前会做这些预处理：

1. 把机器人世界坐标转为网格坐标。
2. 在机器人当前站位附近清空一小块区域，避免把机器人自己误判成障碍。
3. 用 `planning_occupancy` 初始化 FM2 网格。
4. 额外叠加动态障碍速度层或规划层。

### 5.3 每个机器人自己的 FM2 时间场

对每台机器人，都会建立一个 FM2 求解器，计算从该机器人出发到各栅格点的到达代价。

关键参数：

- `v_max`
- `velocity_alpha`
- `velocity_dmax`
- `robot_radius`
- `velocity_mode`
- `velocity_sigmoid_k`
- `velocity_sigmoid_b`
- `space_inflation_radius`

其中：

- `computeFM2_gather()` 计算基础时间场与速度场
- `computeFM2_gather_v()` 在基础速度图上叠加机器人局部动态障碍的约束后再计算

### 5.4 三种 mission 的代价场合成

设第 `i` 台机器人对栅格点 `p` 的到达代价为 `T_i(p)`。

#### `energy`

使用加权平均：

`J(p) = sum_i( w_i * T_i(p) ) / sum_i(w_i)`

对应代码里对每个栅格做加权求和再除以 `weight_sum`。

#### `fastest`

使用最大到达时间：

`J(p) = max_i T_i(p)`

含义是最小化“最后一台机器人到达聚集点”的时间。

#### `space`

`space` 分两步：

1. 先在基础速度图上找“通行能力最强”的主导速度区域。
2. 只在这些 dominant cells 上再合成多机器人代价。

它本质上更偏向“大空间、通行能力强”的聚集中心。

### 5.5 聚集中心选取

在得到总成本图 `J(p)` 后，还会再做一次聚集半径膨胀：

- 半径使用 `gather_radius`
- 若 `gather_radius <= 0`，则用 `hypotenuse + 0.1`

随后取总代价图中的最小代价值点：

`p* = argmin_p J(p)`

再把网格坐标转回世界坐标：

- `goal_centre_[0] = gx`
- `goal_centre_[1] = gy`

并发布为 `/gather_center`。

## 6. 聚集目标的生成与下发

### 6.1 `publish_center_goal_only = true`

当前 simtest 默认是 `publish_center_goal_only: true`。

因此 FM2 gather 计算完中心后，会把所有机器人的目标都设成同一个中心点：

`goal_i = goal_centre`

也就是说，当前这条链是：

- 先用 FM2 求一个公共聚集中心
- 所有机器人先导航到中心附近
- 再由 shape assembly 接管形成最终队形

### 6.2 `publish_center_goal_only = false`

如果关闭这个参数，`Gather::GenerateGoals()` 会围绕聚集中心生成一个半径为 `hypotenuse` 的圆形目标集：

`g_j = center + hypotenuse * [cos(2*pi*j/N), sin(2*pi*j/N)]`

然后用匈牙利算法做最小代价分配：

`cost(i,j) = || robot_i - g_j ||`

把机器人与环形目标一一匹配。

### 6.3 目标占用 fallback

当 `c_plan_monitor` 发现某台机器人的目标被其他机器人占住时，会发 `10 + robot_id` 的信号。

此时 `fm2_gather_node` 会调用 `publish_goal_occupied_staging_goal()`，给该机器人发送一个围绕聚集中心的过渡目标：

- 半径 `staging_radius`
- 方向是“当前相对中心方向”和“该机器人理论 slot 角度”的混合

代码中的混合方式是：

- `slot_angle = 2*pi*robot_index / robot_count`
- `radial_angle = atan2(py-cy, px-cx)`
- `target_angle = blend(radial_angle, slot_angle)`

其中 `blend = 0.25`。

## 7. `c_plan_monitor` 的重规划逻辑

`c_plan_monitor.cpp` 负责监视每个机器人当前全局路径、`move_base` 结果、目标点和里程计。

### 7.1 监视输入

对每台机器人，它订阅：

- `move_base/GraphPlanner/plan`
- `move_base/result`
- `move_base/current_goal`
- `odom`
- `mv_state`
- `/gather_started`

### 7.2 触发条件

当前 event / periodic 混合逻辑主要有这些触发器：

#### 周期重规划

若 `replan_mode` 含 `periodic`，则每过 `periodic_interval` 且满足 `cooldown`，发布：

- `/gather_signal = 2`

#### `ABORTED`

若某机器人 `move_base` 失败：

- 若目标被同伴占用，则发布 `10 + robot_id`
- 否则直接发布 `2`，重新计算聚集中心

#### `STABLE_LEN`

若路径长度长期稳定在 `[stable_len_min, stable_len_max]` 范围内，并且连续变化小于 `stable_len_epsilon`，超过 `stable_len_count` 次后触发单机重规划。

#### `LONG_PATH`

若当前路径长度相较上一次显著增长：

`plan_length > last_plan_length * max_len_growth_ratio`

且：

`plan_length >= max(2*pi*format_radius, long_path_replan_min_length)`

则触发 `/gather_signal = 2`。

### 7.3 与 shape takeover 的耦合

`shape_assembly_swarm.py` 会维护两个参数：

- `/shape_assembly/active_robot_ids`
- `/shape_assembly/stop_path_planning`

当 shape assembly 已经接管某些机器人后：

- `c_plan_monitor` 不再对这些机器人触发重规划
- 若配置 `stop_path_planning_on_shape_takeover=true` 且全部机器人都已接管，则停止继续发布新的 gather replan

## 8. Shape Assembly 初始化

`shape_assembly_swarm.py` 的初始化入口是 `_maybe_init()`。

初始化前必须满足：

1. 所有机器人 odom 都已收到
2. 若 `reference_center_wait=true`，则 `/gather_center` 已收到

初始化阶段会完成：

- 设置 `swarm_size`
- 生成 `refer_state`
- 生成 `shape_state`
- 选 leader 集合 `inform_index`
- 载入 shape matrix
- 初始化 `shape_ctrl_active`

### 8.1 `refer_state`

`refer_state` 是整个 shape assembly 的全局参考状态，包括：

- `pos_x, pos_y`
- `head`
- `vel_x, vel_y`
- `hvel`

如果有 `/gather_center`，中心默认来自它；否则会退化为机器人平均位置。

### 8.2 `shape_state`

`shape_state` 是每个机器人自己维护的“协同形状状态”，包括：

- `pos_x[i], pos_y[i]`
- `vel_x[i], vel_y[i]`
- `head[i], hvel[i]`

它不是机器人真实位置，而是算法内部维护的目标队形状态。

## 9. Shape Assembly 的协同状态更新

### 9.1 邻接关系

邻接矩阵由感知半径 `r_sense` 决定：

- `a_ij = 1`，若 `||p_j - p_i|| <= r_sense`
- 否则 `a_ij = 0`

同时保存：

- 距离矩阵 `d_ij`
- 相对向量 `r_ij = p_j - p_i`

### 9.2 位置协同 `negotiate_position`

先计算邻居平均误差：

`err_x[i] = avg_j( shape_x[j] - shape_x[i] )`

`err_y[i] = avg_j( shape_y[j] - shape_y[i] )`

共识项采用幂律形式：

`u_cons_x[i] = kappa_conse_pos * sign(err_x[i]) * |err_x[i]|^alpha`

`u_cons_y[i] = kappa_conse_pos * sign(err_y[i]) * |err_y[i]|^alpha`

参考跟踪项：

`u_track_x[i] = kappa_track_pos * (ref_x - shape_x[i]) + ref_vx`

`u_track_y[i] = kappa_track_pos * (ref_y - shape_y[i]) + ref_vy`

非 leader 机器人不直接跟踪参考中心，而是跟踪邻居平均速度：

`u_align_x[i] = avg_j( shape_vx[j] )`

`u_align_y[i] = avg_j( shape_vy[j] )`

最终：

- leader:
  `u_x = -u_cons_x + u_track_x`
  `u_y = -u_cons_y + u_track_y`
- follower:
  `u_x = -u_cons_x + u_align_x`
  `u_y = -u_cons_y + u_align_y`

再限幅到 `shape_vel_max`。

### 9.3 朝向协同 `negotiate_orientation`

与位置逻辑完全对应，只是量从位置换成 heading：

`err_h[i] = avg_j( head[j] - head[i] )`

`u_cons_h[i] = kappa_conse_head * sign(err_h[i]) * |err_h[i]|^alpha`

`u_track_h[i] = kappa_track_head * (ref_head - head[i]) + ref_hvel`

`u_align_h[i] = avg_j( hvel[j] )`

最后同样分 leader/follower：

- leader:
  `u_h = -u_cons_h + u_track_h`
- follower:
  `u_h = -u_cons_h + u_align_h`

并限幅到 `hvel_max`。

## 10. Shape 控制项

每个周期真正的 shape control 由四项叠加：

`u = u_enter + u_explore + u_interact + u_local_obs`

### 10.1 Entering 项

该项的目标是让机器人从 shape 外部进入目标形状内部，或者在灰度带内继续向更“黑”的区域深入。

若机器人当前在 shape 外部：

- 找最近的 shape 内合法点 `goal_out`

若机器人已经进入灰度区但还不够深：

- 在附近搜索灰度更低的点 `goal_in`

然后形成 entering 控制：

`dx = goal_x - x_i`

`dy = goal_y - y_i`

`dist = sqrt(dx^2 + dy^2)`

`scale = gray_value`

`u_enter_x = kappa_enter * dx * scale / dist + shape_vel_x[i]`

`u_enter_y = kappa_enter * dy * scale / dist + shape_vel_y[i]`

这里的关键约定是：

- `gray = 1` 表示在 shape 外部
- `gray` 越小，表示越靠近形状内部黑区
- 黑区 `gray ~= 0`

### 10.2 Exploration 项

该项由两部分组成：

- `gf`: 面向“未覆盖形状区域”的填充目标
- `ge`: 面向“邻居未覆盖区域”的探索目标

控制形式：

`u_explore_x = kappa_explore_1 * (gf_x - x_i) + kappa_explore_2 * (ge_x - x_i)`

`u_explore_y = kappa_explore_1 * (gf_y - y_i) + kappa_explore_2 * (ge_y - y_i)`

### 10.3 Interaction 项

Interaction 项同时负责：

- 机器人间软避障
- 机器人间硬避障
- 速度一致性共识

软避障基于 `r_avoid`：

当 `d_ij < r_avoid` 时，产生排斥项。

硬避障基于 `hard_dist = max(r_safe, 2*r_body)`：

当 `d_ij < hard_dist` 时，额外施加强排斥屏障。

速度共识项：

`u_cons_vel_x = avg_j( v_x[i] - v_x[j] )`

`u_cons_vel_y = avg_j( v_y[i] - v_y[j] )`

最终 interaction 项：

`u_interact_x = kappa_avoid * u_avoid_x + kappa_hard_avoid * u_hard_x - kappa_consensus * u_cons_vel_x`

`u_interact_y = kappa_avoid * u_avoid_y + kappa_hard_avoid * u_hard_y - kappa_consensus * u_cons_vel_y`

### 10.4 Local costmap 障碍项

若启用 `use_local_costmap_avoid=true`，每台机器人都会从自己的 `move_base/local_costmap/costmap` 中采样障碍栅格。

对于每个障碍点，若距离机器人满足：

- `dist > self_ignore_radius`
- `dist <= avoid_radius`

则形成局部障碍排斥：

`closeness = (avoid_radius - dist) / avoid_radius`

`u_obs += closeness * unit_away`

若 `dist < hard_radius`，再加硬排斥：

`u_hard_obs += (hard_radius - dist) / dist * unit_away`

最终：

`u_local_obs_x = local_costmap_avoid_gain * sum_x + local_costmap_hard_gain * hard_x`

`u_local_obs_y = local_costmap_avoid_gain * sum_y + local_costmap_hard_gain * hard_y`

## 11. 安全屏障、平滑和发布

控制项叠加后还会经过三层处理：

### 11.1 安全屏障

若机器人间距离小于 `safe_dist = max(r_safe, 2*r_body)`：

- 去掉继续朝向邻居靠近的速度分量
- 叠加反向推力
- 若距离小于更小的 `hard_dist`，直接进入“紧急分离模式”

紧急分离模式下：

`u = vel_max * normalize(push)`

### 11.2 限幅和平滑

先限幅到 `vel_max`，再做一阶平滑：

- 若 `cmd_smooth_use_odom=true`

`u_smooth = ratio * u_now + (1-ratio) * v_odom`

- 否则

`u_smooth = ratio * u_now + (1-ratio) * u_prev`

再乘以 `cmd_scale`，然后再次做安全屏障和限幅。

### 11.3 坐标系转换

若 `cmd_in_map_frame=true`，最终会把 map 系速度转换到机器人本体系后再发 `cmd_vel`。

## 12. `move_base -> shape control` 切换逻辑

这是当前链路最关键的状态切换。

### 12.1 开关前提

仅当：

- `control_strategy = move_base_then_shape`

时，才存在切换逻辑。

若是 `shape_only`，则一开始所有机器人都由 shape assembly 直接控制。

### 12.2 灰度触发

对每个机器人，都计算它在目标形状中的灰度值：

`gray_i = gray( robot_i ; reference_shape_or_local_shape )`

若：

`gray_i < switch_gray_threshold`

则触发切换。

### 12.3 参考半径触发

若启用 `switch_reference_radius_enable=true`，还会计算一个参考半径：

`switch_reference_radius = max( switch_reference_radius_min, max_dist_of_shape + switch_reference_radius_margin )`

其中：

- `max_dist_of_shape` 是目标 shape 任意有效栅格到中心的最大距离

若机器人满足：

`||p_i - ref_center|| <= switch_reference_radius`

也会触发切换。

### 12.4 最终切换判据

每个机器人：

`shape_active_i = gray_trigger_i OR radius_trigger_i`

当 `shape_active_i` 从 `False -> True` 时：

- 发布 `move_base/cancel`
- 后续 `cmd_vel` 由 `shape_assembly_swarm.py` 直接发送

当 `shape_active_i` 从 `True -> False` 时：

- 释放回 `move_base`

### 12.5 takeover 状态参数

切换后会更新：

- `/shape_assembly/active_robot_ids`
- `/shape_assembly/stop_path_planning`

若配置 `stop_path_planning_on_shape_takeover=true` 且所有机器人都已被接管，则 FM2 gather 和 `c_plan_monitor` 将停止继续发新的 path planning 请求。

## 13. 收敛指标

`compute_swarm_metric()` 会计算一组工程指标，用于日志和监视：

- `cover_rate`
  目标形状被机器人覆盖的比例
- `inside_rate`
  机器人进入 shape 区域的比例
- `enter_rate`
  机器人进入黑区的比例
- `dist_var`
  邻接最小距离的离散程度
- `vel_align`
  速度方向一致性
- `neigh_mean`
  平均邻居数量
- `cmd_sat_rate`
  命令饱和比例
- `shape_center_err`
  当前 shape center 到参考中心的误差
- `min_pair_dist`
  最近机器人间距
- `collision_pairs`
  碰撞对数量

## 14. 当前 simtest 的关键 topic

### 14.1 聚集相关

- `/gather_signal`
  启动聚集计算或触发重规划
- `/gather_started`
  gather 导航状态
- `/gather_center`
  FM2 输出的聚集中心
- `/gather_replan_event`
  重规划原因

### 14.2 地图相关

- `/map`
  基础地图
- `/combined_map`
  合并后的全局规划地图
- `/combined_map_occ`
  合并后的可视化 occupancy map

### 14.3 机器人导航相关

- `/robotN/move_base/goal`
- `/robotN/move_base/result`
- `/robotN/move_base/current_goal`
- `/robotN/move_base/GraphPlanner/plan`
- `/robotN/move_base/local_costmap/costmap`

### 14.4 Shape assembly 相关

- `/shape_assembly/task`
  可选的运行时任务输入
- `/shape_assembly/status`
  机器人局部状态
- `/shape_assembly/markers`
  RViz 可视化
- `/shape_assembly/active_robot_ids`
  当前已接管机器人
- `/shape_assembly/stop_path_planning`
  是否停止 path planning

## 15. 当前 simtest 的关键参数文件

- `src/ros_motion_planning/src/user_config/user_config_5x10_d2_simtest.yaml`
  世界、地图、机器人初始位姿、Gazebo / RViz 开关
- `src/fm2_gather/config/gather_param_simtest.yaml`
  FM2 gather 参数
- `src/ros_motion_planning/src/sim_env/config/shape_assembly_simtest.yaml`
  shape assembly 参数
- `src/ros_motion_planning/src/sim_env/config/costmap/global_costmap_params_simtest.yaml`
  全局 costmap
- `src/ros_motion_planning/src/sim_env/config/costmap/local_costmap_params_simtest.yaml`
  局部 costmap
- `src/ros_motion_planning/src/sim_env/config/planner/my_local_planner_params_simtest.yaml`
  `MyPlanner` 参数

## 16. 最小运行步骤

### 16.1 启动仿真

```bash
bash src/ros_motion_planning/scripts/shape_assembly_5x10_d2.sh
```

### 16.2 启动聚集

```bash
rostopic pub -1 /gather_signal std_msgs/UInt8 "{data: 2}"
```

### 16.3 检查是否已经进入聚集流程

```bash
rostopic echo -n 1 /gather_started
rostopic echo -n 1 /gather_center
```

## 17. 附录：与 `start_robot.sh` 的关系

`start_robot.sh` 是真实机器人链路，不等于当前 simtest 集成链。

主要差异：

- `start_robot.sh` 默认只起真机导航和 agent，不自动起集中式 FM2 gather host。
- simtest 用的是 `shape_assembly_with_main_simtest.launch`，把 Gazebo、FM2 gather、shape assembly 一次性串起来。
- simtest 使用：
  - `/odom`
  - `laser_link`
  - `/combined_map`
  - Gazebo `map_to_odom`
- 真机链保留原始 real-robot 参数和坐标系假设。

因此当前文档描述的“聚集切换逻辑、`/gather_signal=2` 触发、`/combined_map` 地图合成、move_base_then_shape 切换”主要对应 simtest 仿真链，而不是原始真机默认链。
