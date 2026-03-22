# 主机代码与机器人代码划分说明

## 1. 目标

当前工作空间已经按“主机战略层 + 机器人执行层”完成重构。

这份文档只回答一个问题：

- 哪些代码应该部署在主机上运行
- 哪些代码应该部署在机器人上运行
- 各自负责什么
- 新代码以后应该放到哪里

---

## 2. 总体原则

### 主机负责的内容

主机只负责低频、全局、战略层任务：

- 选取聚集中心
- 根据可用空间自动选择 `shape_heading`
- 生成并发布共享队形任务 `ShapeTask`
- 监控各机器人导航状态
- 在需要时触发重新规划

主机**不负责**高频控制，不直接参与每个机器人本地的控制闭环。

### 机器人负责的内容

每个机器人本机负责高频执行层任务：

- 本机传感器驱动
- 本机定位与导航
- 本机 `move_base`
- 本机局部避障
- 本机 `shape_assembly` 分布式控制
- 根据主机下发的共享任务，独立计算自己的 staging goal

---

## 3. 主机侧代码

### 3.1 `formation_host`

主机侧新增的战略层任务发布包。

- 包路径：`src/formation_host`
- 主要 launch：`src/formation_host/launch/formation_host.launch`
- 主要脚本：`src/formation_host/scripts/shape_task_supervisor.py`

职责：

- 订阅 `/gather_center`
- 发布共享任务 `/shape_assembly/task`
- 在任务中填充：
  - `shape_type`
  - `shape_heading`
  - `shape_scale`
  - `staging_radius`
- 默认根据 `/map` 的可用空间自动计算 `shape_heading`
- 手动传入的 `shape_heading` 只作为回退初值，不再是主模式

### 3.2 `fm2_gather`

主机侧聚集中心与重规划决策的核心包。

- 包路径：`src/fm2_gather`
- 主要 launch：`src/fm2_gather/launch/laun.launch`
- 主要节点：`src/fm2_gather/src/fm2_gather_node.cpp`

职责：

- 计算全局聚集中心
- 接收监控侧的重规划触发
- 在战略层模式下只发布 `/gather_center`
- 与 `formation_host` 配合，由后者把中心转换成共享 `ShapeTask`

当前约定：

- 主机侧默认使用 `publish_goal:=false`
- 这表示 `fm2_gather` 不再作为“直接给每台机器人发导航目标”的主路径
- 它现在是“聚集中心选择器 + 战略层重规划入口”

### 3.3 `move_base_client`

主机侧监控与重规划信号包。

- 包路径：`src/move_base_client`
- 主要节点：`src/move_base_client/src/c_plan_monitor.cpp`

职责：

- 监控各机器人 `move_base` 状态
- 监控路径长度变化、失败、卡住等情况
- 向主机战略层发出重规划事件
- 跳过已经进入 `shape_assembly` 接管状态的机器人

### 3.4 主机侧兼容入口

为了保持你原来的启动习惯，保留了一个主机侧包装 launch：

- `src/turn_on_ws/src/turn_on_wheeltec_robot/launch/shape_assembly_host.launch`

说明：

- 这个文件现在只是兼容入口
- 它本身不再承载核心逻辑
- 真正的主机逻辑已经转移到 `formation_host` 和 `fm2_gather`

---

## 4. 机器人侧代码

### 4.1 `formation_robot`

机器人侧新增的任务桥接包。

- 包路径：`src/formation_robot`
- 主要 launch：`src/formation_robot/launch/formation_robot.launch`
- 主要脚本：`src/formation_robot/scripts/shape_task_bridge.py`

职责：

- 订阅共享任务 `/shape_assembly/task`
- 运行时实际 shape 以主机发送的 `ShapeTask.shape_type` 为准
- 本地 launch 里的 `shape_type` 只作为尚未收到首个任务前的启动回退值
- 根据以下共享信息，独立计算本机器人的 staging goal：
  - `center`
  - `shape_type`
  - `shape_heading`
  - `shape_scale`
  - `robot_id`
- 将结果发送给本机 `/<robot>/move_base`
- 发布本机 staging goal 预览：
  - `/<robot>/shape_assembly/staging_goal`

当前实现特点：

- staging goal 已经不是简单固定半径圆环
- 机器人会根据队形几何本身来推导本地进入点
- `staging_radius` 现在只作为主机侧可选覆盖参数


### 4.1.1 `staging goal` 示意图

默认模式下，机器人不会直接把聚集中心当作 `move_base` 目标，而是先导航到队形外沿附近的一个进入点：

```text
                         robot i 的 staging goal
                                   *
                                .-   -.
                             .-'       '-.
                           .'   staging    '.
                          /      ring        \
                         ;                    ;
                         |       ######       |
                         |     ### shape ###  |
                         |       ######       |
                         ;                    ;
                          \       C          /
                           '.   gather      .'
                             '-. center  .-'
                                '-.___.-'
```

含义：

- `C` 是主机发布的聚集中心 `gather center`
- `shape` 是最终要形成的目标队形
- `staging ring` 不是固定圆，而是机器人根据 `shape_type + shape_heading + shape_scale` 计算出的外沿进入区
- 每个机器人按自己的 `robot_id` 分配一个入场方向，再生成自己的 `staging goal`

### 4.1.2 直接导航到聚集中心的配置开关

如果你不想让机器人先去 staging 点，而是直接把聚集中心当作导航目标，可以打开：

- launch 参数：`use_center_as_goal:=true`

作用：

- `false`：默认模式，机器人导航到本机计算出的 `staging goal`
- `true`：机器人直接把 `ShapeTask.center` 作为 `move_base` 目标

示例：

```bash
roslaunch turn_on_wheeltec_robot motion_navigate_multi4.launch \
  map_started:=true \
  agent_number:=4 \
  agent_id:=1 \
  enable_shape_assembly:=true \
  use_center_as_goal:=true
```

### 4.2 `sim_env / shape_assembly_swarm.py`

机器人侧分布式编队控制核心实现。

- 包路径：`src/ros_motion_planning/src/sim_env`
- 主要脚本：`src/ros_motion_planning/src/sim_env/scripts/shape_assembly_swarm.py`
- 主要配置：`src/ros_motion_planning/src/sim_env/config/shape_assembly.yaml`
- 主要 launch：`src/ros_motion_planning/src/sim_env/launch/app/shape_assembly.launch.xml`

职责：

- 订阅共享任务 `/shape_assembly/task`
- 运行时实际 shape 以主机发送的 `ShapeTask.shape_type` 为准
- 本地 launch 里的 `shape_type` 只作为启动回退值
- 在运行时更新：
  - `reference center`
  - `shape type`
  - `shape heading`
- 根据邻居与局部状态，决定本机是否从 `move_base` 切换到 `shape_assembly`
- 在进入编队控制后取消本机 `move_base`
- 只发布本机自己的 `cmd_vel`
- 发布本机状态：
  - `/<robot>/shape_assembly/status`

说明：

- 这里依然是分布式控制主体
- 它运行在每台机器人本机
- 主机不参与这个高频控制环

### 4.3 `turn_on_wheeltec_robot`

机器人本机导航与硬件 bringup 包。

- 包路径：`src/turn_on_ws/src/turn_on_wheeltec_robot`
- 主要入口：`src/turn_on_ws/src/turn_on_wheeltec_robot/launch/motion_navigate_multi4.launch`

职责：

- 底盘驱动
- 雷达驱动
- AMCL
- 本机 `move_base`
- 调用机器人侧 `shape_assembly` 包装入口

### 4.4 机器人侧兼容入口

保留一个机器人侧包装 launch：

- `src/turn_on_ws/src/turn_on_wheeltec_robot/launch/include/shape_assembly_agent.launch`

说明：

- 这个文件现在只是兼容入口
- 真正的机器人侧编队入口已经迁移到 `formation_robot.launch`

---


### 4.5 `shape_images` 的存储约束

主机和机器人都必须各自保存一份本地 `shape_images` 目录。

原因：

- 主机在自动选角和 shape 任务生成时需要读取 shape 模型
- 机器人在本机计算 `staging goal` 和本机运行 `shape_assembly` 时也需要读取同样的 shape 模型
- 机器人运行时不会去读取主机文件系统中的 `shape_images`，只能读取自己机器上的本地目录

当前默认路径：

- 主机：`$(find sim_env)/shape_images`
- 机器人：`$(find sim_env)/shape_images`

说明：

- 这表示主机工作空间和每台机器人工作空间里都要有 `src/ros_motion_planning/src/sim_env/shape_images`
- 如果部署目录不同，可以分别通过 `shape_library_root` 参数覆盖

## 5. 共享代码与通信接口

### 5.1 `formation_msgs`

主机和机器人之间的共享协议放在单独的消息包中。

- 包路径：`src/formation_msgs`
- 消息文件：
  - `src/formation_msgs/msg/ShapeTask.msg`
  - `src/formation_msgs/msg/RobotFormationStatus.msg`

职责：

- 统一定义主机和机器人之间的任务协议
- 统一定义机器人回传给主机的状态协议

### 5.2 关键话题

主机发布：

- `/shape_assembly/center_goal_cmd`
  - 操作员或上层系统发布共享中心点
- `/gather_center`
  - `fm2_gather` 发布当前聚集中心
- `/shape_assembly/task`
  - `formation_host` 发布共享队形任务

机器人发布：

- `/<robot>/shape_assembly/staging_goal`
  - 本机计算出的 staging goal 预览
- `/<robot>/shape_assembly/status`
  - 本机编队状态

兼容参数接口：

- `/shape_assembly/active_robot_ids`
- `/shape_assembly/stop_path_planning`

说明：

- 这两个参数接口目前仍保留，用于兼容现有监控与重规划链
- 但新的主路径已经是 `ShapeTask` 和 `RobotFormationStatus`

---

## 6. 启动方式

### 6.1 主机启动

```bash
roslaunch turn_on_wheeltec_robot shape_assembly_host.launch \
  agent_number:=N \
  shape_type:=rectangle \
  auto_shape_heading:=true
```

机器人侧默认使用 `use_center_as_goal:=false`，即先导航到 staging goal。

### 6.2 机器人启动

```bash
roslaunch turn_on_wheeltec_robot motion_navigate_multi4.launch \
  map_started:=true \
  agent_number:=N \
  agent_id:=K \
  enable_shape_assembly:=true
```

---

## 7. 后续代码放置规范

### 7.1 应该放在主机侧的代码

如果新功能属于以下类型，应放在主机侧包中：

- 聚集中心选择
- 全局 shape 选择
- `shape_heading` 自动选角
- 多机器人全局监控
- 重规划判定
- 运维监控、日志、可视化调度

推荐位置：

- `formation_host`
- `fm2_gather`
- `move_base_client`

### 7.2 应该放在机器人侧的代码

如果新功能属于以下类型，应放在机器人侧包中：

- 本机 `move_base` 目标生成
- 本机分布式编队控制
- 本机传感器融合
- 本机局部避障
- 本机状态回传

推荐位置：

- `formation_robot`
- `sim_env`
- `turn_on_wheeltec_robot`

### 7.3 共享协议应该放哪里

凡是主机和机器人都要使用的数据结构，不要先塞进 launch 参数。

统一规则：

- 先定义到 `formation_msgs`
- 再在主机和机器人节点里同时接入

---

## 8. 当前完成状态

已完成：

- 主机战略层代码和机器人执行层代码已经分离
- 主机自动选 `shape_heading` 已接入
- 机器人本机已能根据共享 `ShapeTask` 独立计算 staging goal
- 机器人本机继续运行 `move_base + shape_assembly`
- 兼容旧启动方式的 wrapper launch 已保留

尚未完成的部分不是代码划分，而是联调验证：

- 真机多机器人网络负载验证
- 真机控制频率验证
- takeover 半径与 `r_sense` 参数整定
- 重规划效果与稳定性验证
