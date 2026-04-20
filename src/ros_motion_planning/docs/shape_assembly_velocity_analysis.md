# Shape Assembly 机器人速度计算分析

## 1. 文档范围

本文分析当前仓库中 `shape_assembly_swarm.py` 的机器人速度计算链路，重点回答四件事：

- 机器人最终发布的 `cmd_vel` 是如何逐步计算出来的
- 每一项速度分量的数学形式、物理意义和调参含义是什么
- 位置控制、朝向控制、避障、平滑、限幅之间如何耦合
- 当前运行参数从哪里来，哪些是代码默认值，哪些通常被 YAML 覆盖

本文对应主实现：

- [shape_assembly_swarm.py](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:1)
- [shape_assembly.launch.xml](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/launch/app/shape_assembly.launch.xml:1)

当前工程里最常用的两套参数文件是：

- [shape_assembly_real_simaligned.yaml](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/config/shape_assembly_real_simaligned.yaml:1)
- [shape_assembly_simtest.yaml](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/config/shape_assembly_simtest.yaml:1)

## 2. 总体控制链

`_on_timer()` 是主控制循环入口，当前代码的主链是：

1. 读取机器人真实状态 `robot_state`
2. 计算邻接关系 `neigh`
3. 更新内部“目标形状状态” `shape_state`
4. 生成当前时刻的动态形状投影 `shape_dyn`
5. 分别计算四类平移控制项
   - entering
   - exploration
   - interaction
   - local obstacle avoidance
6. 对四类平移控制项求和
7. 做安全屏障、限速、平滑、缩放、死区处理
8. 根据 `move_base_then_shape` 切换逻辑决定谁由 shape assembly 接管
9. 计算 yaw 误差和角速度命令
10. yaw 误差较大时抑制线速度
11. 将地图系速度转换到机器人基座系
12. 发布 `cmd_vel`

对应代码主链在：

- [_on_timer](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:3693)

可以把最终控制律概括成：

`u_i = sat( u_enter,i + u_explore,i + u_interact,i + u_obs,i )`

然后再经过：

- 安全屏障修正
- 一阶平滑
- 全局缩放
- 线速度死区
- yaw 修正后的线速度抑制
- 坐标系变换

最终得到：

- `twist.linear.x`
- `twist.linear.y`
- `twist.angular.z`

## 3. 状态量和坐标系

### 3.1 三类核心状态

代码里有三套状态：

1. `RobotState`
   - 来自真实 odom
   - 表示机器人实际位置、速度、yaw
2. `ShapeState`
   - 是 shape assembly 内部维护的“目标形状参考状态”
   - 每个机器人都有一个自己的形状锚点 `(pos_x, pos_y, head)`
3. `ReferState`
   - 当 `shape_target_mode=reference` 时，整队共享一个参考中心和参考 heading

定义位置见：

- [RobotState / ReferState / ShapeState](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:125)

### 3.2 形状坐标系

目标形状首先以灰度图 / mat 形式定义，再被转换成离散的形状局部坐标：

- 每个栅格中心有一个 `(base_x, base_y)`
- 黑色区域表示目标占据区
- 灰色区域表示缓冲过渡区
- 白色区域表示形状外部

变换过程见：

- [init_form_shape](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:377)

其核心是：

- `grid`：每个形状栅格在世界中的实际长度
- `base_x/base_y`：形状局部点坐标

`grid` 的计算是：

`grid = sqrt((pi / 4) * (N / black_num)) * r_avoid * scale`

其中：

- `N` 是机器人数量
- `black_num` 是形状中黑色栅格数量
- `r_avoid` 是机器人期望间距尺度
- `scale` 是全局形状缩放

这个式子的直觉是：按“总机器人数”和“目标黑色面积”估计一个形状采样间距，让机器人密度与目标形状面积相匹配。

### 3.3 动态形状投影

每个机器人都有一份“当前时刻属于自己的目标形状投影”：

`[rx, ry]^T = R(head_i) [base_x, base_y]^T + [pos_x_i, pos_y_i]^T`

其中 `R(head_i)` 是二维旋转矩阵。

代码见：

- [get_dyn_formation](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:427)

这一步的作用是：把抽象形状模板投影到每台机器人当前的局部参考系下，后面的 entering / exploration 都是在这个投影上做。

## 4. 邻接图与多机器人耦合

机器人间耦合由邻接矩阵 `a_mtr` 和距离矩阵 `d_mtr` 决定。

当两台机器人距离 `dist <= r_sense` 时：

- `a_mtr[i][j] = 1`
- 认为 `j` 是 `i` 的邻居

同时记录：

- `d_mtr[i][j] = ||p_j - p_i||`
- `rx_mtr[i][j] = x_j - x_i`
- `ry_mtr[i][j] = y_j - y_i`

代码见：

- [get_neighbor_set](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:452)

从控制理论看，这一步是在构造时变通信图 `G(t)`。后面的共识项、本地一致性项都依赖这个图。

## 5. 内部形状状态更新

在真正给底盘发速度前，算法会先更新内部 `shape_state`，即“理想编队锚点”本身也会演化。

### 5.1 位置协商 `negotiate_position`

对每个机器人，先计算邻域位置误差均值：

`e_x,i = (1 / |N_i|) * sum_{j in N_i} (x_j^s - x_i^s)`

`e_y,i = (1 / |N_i|) * sum_{j in N_i} (y_j^s - y_i^s)`

然后得到共识项：

`u_cons,x = kappa_conse_pos * sign(e_x) * |e_x|^alpha`

`u_cons,y = kappa_conse_pos * sign(e_y) * |e_y|^alpha`

以及参考跟踪项：

`u_track,x = kappa_track_pos * (x_ref - x_i^s) + v_ref,x`

`u_track,y = kappa_track_pos * (y_ref - y_i^s) + v_ref,y`

未被 inform 的机器人不直接跟踪参考中心，而是跟踪邻居平均速度：

`u_align = average(v_j^s)`

最终：

`u_shape,x = -u_cons,x + inform_i * u_track,x + (1 - inform_i) * u_align,x`

`u_shape,y = -u_cons,y + inform_i * u_track,y + (1 - inform_i) * u_align,y`

代码见：

- [negotiate_position](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:468)

理论上，这是典型的 leader-follower 一致性控制：

- `-u_cons` 负责收缩邻域分歧
- `u_track` 将有信息的 agent 拉向全局参考中心
- `u_align` 让无信息 agent 通过局部速度一致性传播参考信息

其中 `sign(e) |e|^alpha` 在 `0 < alpha < 1` 时属于非线性有限时间收敛风格的反馈，比纯线性项在小误差区域更“硬”。

### 5.2 姿态协商 `negotiate_orientation`

姿态部分完全同构，只是把位置换成 heading：

`e_h,i = (1 / |N_i|) * sum_{j in N_i} (h_j^s - h_i^s)`

`u_cons,h = kappa_conse_head * sign(e_h) * |e_h|^alpha`

`u_track,h = kappa_track_head * (h_ref - h_i^s) + omega_ref`

`u_align,h = average(hvel_j^s)`

`u_h = -u_cons,h + inform_i * u_track,h + (1 - inform_i) * u_align,h`

再限幅到：

`|u_h| <= hvel_max`

代码见：

- [negotiate_orientation](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:545)

这一步更新的是内部 `shape_state.head/hvel`，不是底盘实际 `twist.angular.z`。实际角速度要到后面的 yaw control 才发给机器人。

## 6. 平移速度四个主分量

真正给机器人平移速度的主要四项是：

`u_xy = u_enter + u_explore + u_interact + u_obs`

### 6.1 Entering 项

目标：把机器人先拉进目标形状区域内部。

首先把机器人当前世界坐标投到形状局部坐标里：

- 若落在白区，去最近的非白色目标点
- 若已落在灰区，去邻域内更黑的区域

核心逻辑在：

- [trans_goal_to_local](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:611)
- [get_local_target_out](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:640)
- [get_local_target_in](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:662)
- [cal_entering_cmd](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:684)

控制式为：

`dx = goal_x - x_i`

`dy = goal_y - y_i`

`dist = ||[dx, dy]||`

`scale = gray_color`

`u_enter,x = kappa_enter * dx * scale / dist + v_shape,x`

`u_enter,y = kappa_enter * dy * scale / dist + v_shape,y`

解释：

- `dx/dist, dy/dist` 是归一化方向，保证进入项主要控制方向而非距离平方爆炸
- `gray_color` 是灰度值，越接近黑区越小，意味着机器人进入目标区域后，entering 力自动减弱
- `+ shape_state.vel` 提供一个参考速度前馈，避免 purely reactive 导致抖动

理论上它相当于一个“形状区域吸引场”，而灰度因子构成了空间变增益。

### 6.2 Exploration 项

目标：在形状内部补空洞，并优先探索尚未被邻居覆盖的区域。

它由两个点构成：

1. `gf`：fill point，偏向填补附近空白目标单元
2. `ge`：exploration point，偏向尚未被邻居覆盖的目标单元

代码见：

- [get_mean_point_fill](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:734)
- [get_mean_point_expl](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:763)
- [cal_exploration_cmd](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:809)

最终控制式：

`u_explore,x = kappa_explore_1 * (gf_x - x_i) + kappa_explore_2 * (ge_x - x_i)`

`u_explore,y = kappa_explore_1 * (gf_y - y_i) + kappa_explore_2 * (ge_y - y_i)`

这里本质是双目标线性吸引：

- `gf` 负责“补满局部”
- `ge` 负责“避免机器人都盯着同一块区域”

而 `get_mean_point_fill/get_mean_point_expl` 又不是简单取最近点，而是加权平均：

`w(d) = 1, d < 2s`

`w(d) = 0, d > r`

`w(d) = (1 + cos(pi * (d - 2s)/(r - 2s))) / 2, otherwise`

这是一个余弦窗权重函数，作用是让局部目标点对距离更平滑，不会像最近点法那样跳变明显。

### 6.3 Interaction 项

目标：处理机器人间排斥和速度一致性。

代码见：

- [cal_interaction_cmd](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:831)

这部分有三项。

#### 6.3.1 常规排斥项

当邻居距离小于 `r_avoid` 时：

`temp = r_avoid - d_ij`

`spr = temp / dis`

`u_rep,ij = spr * (-r_ij / dis)`

总和后得到 `cmd_avoid_x/y`。

直观上这是一个基于剩余安全间距的排斥场，距离越近，排斥越强。

#### 6.3.2 硬排斥项

当距离小于 `hard_dist = max(r_safe, 2 * r_body)` 时，再加一层短程强屏障：

`over = hard_dist - d_ij`

`gain = over / d_ij`

`u_hard,ij = gain * (-r_ij / d_ij)`

这项是为了解决低速挤压和穿模，属于更硬的短程 barrier。

#### 6.3.3 速度一致性项

对邻居平均速度差：

`u_cons_vel,x = (1 / |N_i|) * sum_{j in N_i} (v_i,x - v_j,x)`

`u_cons_vel,y = (1 / |N_i|) * sum_{j in N_i} (v_i,y - v_j,y)`

总 interaction 项：

`u_interact,x = kappa_avoid * u_rep,x + kappa_hard_avoid * u_hard,x - kappa_consensus * u_cons_vel,x`

`u_interact,y = kappa_avoid * u_rep,y + kappa_hard_avoid * u_hard,y - kappa_consensus * u_cons_vel,y`

理论上这是“人工势场 + 速度共识”混合控制：

- 势场负责避免碰撞
- 速度共识负责局部群体运动协调

### 6.4 Local costmap obstacle 项

目标：把 `move_base` 的局部 costmap 作为补充障碍感知层，增强对非机器人障碍物的实时规避。

代码见：

- [_cal_local_obstacle_cmd](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:2254)

逻辑是：

1. 遍历本机器人 local costmap 占据栅格
2. 将栅格点变换到 odom / map 坐标系
3. 对距离小于 `avoid_radius` 的障碍施加排斥
4. 若距离小于 `hard_radius`，再叠加硬排斥

具体形式：

`closeness = (avoid_radius - dist) / avoid_radius`

`u_obs_soft += closeness * unit_away`

若 `dist < hard_radius`：

`hard = (hard_radius - dist) / dist`

`u_obs_hard += hard * unit_away`

最后：

`u_obs = local_costmap_avoid_gain * u_obs_soft + local_costmap_hard_gain * u_obs_hard`

这部分和 interaction 很像，但 interaction 针对“其他机器人”，而这里针对“地图障碍物”。

## 7. 安全屏障、限速、平滑与死区

### 7.1 安全屏障 `enforce_safety_barrier`

在四项求和后，代码还会再额外做一次强制安全修正：

- 如果当前命令还在朝邻居方向继续推进，就把朝向邻居的速度分量投影掉
- 再沿远离邻居方向加一个二次 closeness 推力
- 若已经非常近，则直接进入紧急模式，只保留“分离方向”速度

代码见：

- [enforce_safety_barrier](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:910)

这一步的理论意义更接近 control barrier function 的工程近似版：

- 先移除违反安全约束的“内向速度分量”
- 再加入外向修正

虽然它不是严格 CBF 求解器，但思路类似“先满足安全，再谈性能”。

### 7.2 限速 `limit_speed`

对每个机器人：

`speed = sqrt(cmd_x^2 + cmd_y^2)`

若 `speed > vel_max`：

`cmd <- (vel_max / speed) * cmd`

代码见：

- [limit_speed](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:961)

这是标准向量归一化限幅，保持方向不变，只裁剪模长。

### 7.3 命令平滑

如果 `cmd_smooth_ratio > 0`，控制命令会做一阶低通：

若 `cmd_smooth_use_odom=true`：

`cmd_new = ratio * cmd_raw + (1 - ratio) * v_odom`

若 `false`：

`cmd_new = ratio * cmd_raw + (1 - ratio) * cmd_prev`

代码见：

- [_on_timer 平滑段](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:3754)

从信号处理角度，这是指数平滑 / 一阶惯性滤波：

- ratio 越小，越保守
- ratio 越大，越接近原始命令

使用 odom 作为平滑参考时，等价于“让命令追随实际底盘速度”，通常比追随上一次命令更稳。

### 7.4 全局缩放与线速度死区

平滑后还会乘 `cmd_scale`：

`cmd <- cmd_scale * cmd`

然后再做死区：

若 `||cmd|| <= deadzone`，直接置零；

否则：

`cmd <- ((||cmd|| - deadzone) / ||cmd||) * cmd`

代码见：

- [apply_linear_deadzone](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:970)

这个式子不是简单截断，而是“削去一个固定半径”，保持小命令不发，大命令连续。

## 8. 接管逻辑与 yaw 修正

### 8.1 `move_base_then_shape` 切换

如果当前策略是 `move_base_then_shape`，并不是所有机器人一开始都由 shape assembly 接管。

切换条件见：

- [_update_control_switch](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:2650)

主要判据有两个：

1. 灰度判据
   - 机器人进入参考形状的灰/黑区域
   - `gray_value < switch_gray_threshold`
2. 参考半径判据
   - 若开启 `switch_reference_radius_enable`
   - 机器人进入参考中心附近某个半径

满足后：

- shape assembly 开始接管该机器人
- 可选地给对应 `move_base` 发 cancel

### 8.2 yaw 误差控制

当前实现已经包含 yaw 修正：

- [_compute_angular_cmds](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:2948)
- [_apply_yaw_linear_slowdown](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:2929)

若机器人处于 shape control：

`yaw_err = wrap(target_head - yaw_robot)`

若 `|yaw_err| <= yaw_tolerance`，角速度置零。

否则：

`omega = kappa_track_head * yaw_err + target_hvel`

并限幅：

`|omega| <= hvel_max`

这是一条标准的 P + feedforward 角速度控制律。

### 8.3 yaw 误差过大时抑制线速度

如果 yaw 误差大于 `yaw_linear_slowdown_start`，线速度开始衰减；

若大于 `yaw_linear_stop_threshold`，线速度直接置零。

本质是：

- 朝向差很大时优先转头
- 避免机器人一边横着冲、一边急转

这等价于在平移控制和旋转控制之间引入一个姿态相关的调度器。

## 9. 坐标系变换与最终发布

内部 `cmd_x/cmd_y` 默认是在地图系 / odom 系里计算的。

如果 `cmd_in_map_frame=true`，发布前会做二维旋转变换到机器人基座系：

`v_body,x = cos(yaw) * v_map,x + sin(yaw) * v_map,y`

`v_body,y = -sin(yaw) * v_map,x + cos(yaw) * v_map,y`

代码见：

- [_on_timer 坐标系变换段](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py:3784)

然后发布：

- `twist.linear.x = pub_cmd_x`
- `twist.linear.y = pub_cmd_y`
- `twist.angular.z = angular_cmd`

若该机器人尚未切到 shape control，则此轮不发 shape 命令，由 `move_base` 继续控制。

## 10. 当前参数来源与优先级

参数优先级见：

- [shape_assembly.launch.xml](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/launch/app/shape_assembly.launch.xml:1)

顺序是：

1. launch 命令行显式覆盖
2. YAML 文件
3. launch 内置 fallback
4. Python 中 `SimParam` 的默认初值

因此 Python 里的默认值只是最后兜底，实际运行通常看 YAML。

### 10.1 `real_simaligned` 当前关键值

参考：

- [shape_assembly_real_simaligned.yaml](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/config/shape_assembly_real_simaligned.yaml:1)

当前常用值是：

- `vel_max = 0.7`
- `shape_vel_max = 1.0`
- `kappa_enter = 4.0`
- `kappa_explore_1 = 1.0`
- `kappa_explore_2 = 8.0`
- `kappa_avoid = 8.0`
- `kappa_hard_avoid = 10.0`
- `kappa_consensus = 1.2`
- `kappa_conse_pos = 1.6`
- `kappa_track_pos = 3.0`
- `kappa_conse_head = 1.6`
- `kappa_track_head = 3.0`
- `hvel_max = 1.57`
- `cmd_smooth_ratio = 0.32`
- `cmd_scale = 0.5`
- `cmd_deadzone = 0.04`
- `yaw_tolerance = 0.05`
- `local_costmap_avoid_gain = 7.0`
- `local_costmap_hard_gain = 9.0`

### 10.2 `simtest` 当前关键值

参考：

- [shape_assembly_simtest.yaml](/home/yxw/motion_planning_ws/src/ros_motion_planning/src/sim_env/config/shape_assembly_simtest.yaml:1)

与 `real_simaligned` 大体一致，但主要差异有：

- `cmd_scale = 1.0`
- `robot_detect_topic_suffix = /odom`
- 某些可视化与 Gazebo 相关参数不同

因此真机和仿真看起来“算法一样但速度感不同”，一个直接原因就是 `cmd_scale` 和底层导航参数不同。

## 11. 数学和理论依据总结

当前 `shape_assembly` 不是单一算法，而是几类控制思想的叠加。

### 11.1 一致性控制 / 编队协同

对应：

- `negotiate_position`
- `negotiate_orientation`
- interaction 中的速度一致性项

本质是多智能体系统中的 consensus / leader-follower 控制。

### 11.2 势场法 / 人工势场

对应：

- entering 吸引
- exploration 吸引
- interaction 排斥
- local costmap 排斥

这类方法优点是反应快、实现直接，缺点是可能有局部极值，因此代码里又用灰度场、邻域均值、共识项来缓解。

### 11.3 安全屏障近似

对应：

- `enforce_safety_barrier`
- hard avoid

它并非严格二次规划形式的 CBF，但已经体现出“移除危险速度分量 + 添加安全修正”的思路。

### 11.4 一阶低通滤波

对应：

- `cmd_smooth_ratio`

这是典型的控制输入整形，用来提高可执行性并减少过冲和振荡。

### 11.5 姿态闭环与调度

对应：

- yaw P 控制
- yaw error 触发的线速度抑制

这一步相当于把平移控制和姿态控制解耦后，再通过误差门控重新耦合。

## 12. 对调参与行为的直接解释

### 12.1 为什么会过冲

若这些参数偏大：

- `kappa_enter`
- `kappa_explore_2`
- `kappa_avoid`
- `cmd_scale`
- `vel_max`

而这些参数偏小：

- `cmd_smooth_ratio`
- `cmd_deadzone`
- `yaw_linear_slowdown_start`

就容易出现：

- 接近目标区时速度仍偏大
- 一边大平移一边修 heading
- 局部障碍 / 邻居排斥叠加过猛

### 12.2 为什么会抖动

常见原因：

- `r_avoid` 太小，机器人太挤
- `kappa_avoid` 和 `kappa_hard_avoid` 太大
- `switch_gray_threshold` 太高，过早切到 shape control
- `cmd_smooth_ratio` 太大，命令过于贴近原始势场

### 12.3 为什么会形成队伍后还在微动

常见原因：

- `cmd_deadzone` 太小
- `yaw_tolerance` 太小
- `cmd_scale` 偏大
- `local_costmap` 持续有占据波动

## 13. 结论

当前 `shape_assembly` 的速度控制可以概括为：

- 用一致性控制维护编队参考状态
- 用 entering / exploration 保证机器人进入并填满目标形状
- 用 interaction / obstacle 避免机器人间和机器人与环境间的碰撞
- 用安全屏障、限速、平滑、yaw 修正把理论速度变成可执行速度

因此它并不是单纯的“几何跟踪器”，而是一个多项控制叠加的群体控制器。调参时应当把参数分组看待：

- 形状进入：`kappa_enter`
- 形状分布：`kappa_explore_1/2`
- 群体排斥：`kappa_avoid / kappa_hard_avoid / r_avoid`
- 编队参考：`kappa_conse_* / kappa_track_*`
- 可执行性：`vel_max / cmd_scale / cmd_smooth_ratio / cmd_deadzone`
- 姿态收敛：`kappa_track_head / hvel_max / yaw_*`

如果只盯一个参数，很容易误判问题来源。
