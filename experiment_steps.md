# 多机器人实验步骤清单

这份清单基于工作空间根目录新增的两个脚本：

- [start_host.sh](/home/yxw/motion_planning_ws/start_host.sh)
- [start_robot.sh](/home/yxw/motion_planning_ws/start_robot.sh)

默认配置文件：

- 主机：[config/host_param.yaml](/home/yxw/motion_planning_ws/config/host_param.yaml)
- 机器人：[config/robot_param.yaml](/home/yxw/motion_planning_ws/config/robot_param.yaml)

默认目标：

- 主机启动 `roscore`、`map_server`、`shape_assembly_host.launch`
- 主机默认进入待命状态，等待操作人通过 `/gather_signal` 手动触发 `fm2_gather` 计算聚集中心
- 每台机器人运行自己的 `move_base + shape_assembly`

详细真机运行手册见：

- [real_simaligned_real_robot_runbook.md](/home/yxw/motion_planning_ws/src/ros_motion_planning/docs/real_simaligned_real_robot_runbook.md)

## 1. 实验前检查

- [ ] 主机和机器人都已执行编译
- [ ] 主机和机器人都有本地 `shape_images`
- [ ] 主机地图文件可用
- [ ] 每台机器人底盘串口和雷达设备正常
- [ ] 每台机器人 `agent_id` 唯一
- [ ] 所有机器网络互通

## 2. 先编辑配置文件

主机编辑：

- [config/host_param.yaml](/home/yxw/motion_planning_ws/config/host_param.yaml)

至少确认：

- `ros.master_uri`
- `ros.ip`
- `shape_assembly_host.agent_number`
- `shape_assembly_host.robot_ids`
- `formation_host.shape_type`

每台机器人分别编辑自己的本地文件：

- [config/robot_param.yaml](/home/yxw/motion_planning_ws/config/robot_param.yaml)

至少确认：

- `ros.master_uri`
- `ros.ip`
- `motion_navigate_multi4.agent_id`
- `motion_navigate_multi4.agent_number`
- `motion_navigate_multi4.robot_ids`

说明：

- 每台机器人都有自己本地工作空间，所以都可以用同一个路径 `config/robot_param.yaml`
- 但每台机器人里面的 `ros.ip` 和 `motion_navigate_multi4.agent_id` 应该不同

## 3. 主机启动

配置文件准备好后，主机终端直接执行：

```bash
cd /home/yxw/motion_planning_ws
./start_host.sh
```

如果要临时覆盖配置文件，例如使用另一份主机配置：

```bash
./start_host.sh config:=/home/yxw/motion_planning_ws/config/host_param.yaml
```

如果只想临时覆盖单个参数，也可以直接附加：

```bash
./start_host.sh auto_start_gather:=false shape_type:=triangle
```

## 4. 机器人启动

每台机器人各开一个终端，配置文件已经各自写好后，直接执行：

```bash
cd /home/yxw/motion_planning_ws
./start_robot.sh
```

如果你在同一台电脑上想切换不同机器人配置，可以额外指定配置文件：

```bash
./start_robot.sh config:=/home/yxw/motion_planning_ws/config/robot_param.yaml
```

如果只想临时覆盖单个参数，也可以直接附加：

```bash
./start_robot.sh agent_id:=2 use_center_as_goal:=true
```

## 5. 运行时观察

主机侧建议观察：

```bash
rostopic echo /gather_center
rostopic echo /shape_assembly/task
```

机器人侧建议观察，以 `robot1` 为例：

```bash
rostopic echo /robot1/shape_assembly/staging_goal
rostopic echo /robot1/shape_assembly/status
```

## 6. 手动控制选项

当前默认就是手动触发聚集中心计算。主机启动完成后，手动触发：

```bash
rosrun move_base_client start_gather.py --wait-started 5.0
```

或直接发布底层信号：

```bash
rostopic pub -1 /gather_signal std_msgs/UInt8 '{data: 2}'
```

如果你要人工覆盖 `fm2_gather` 计算出的中心，则额外发布：

```bash
rostopic pub -1 /shape_assembly/center_goal_cmd geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}'
```

## 7. 结束实验

建议停止顺序：

1. 先停所有机器人终端
2. 再停主机 `start_host.sh`
3. 如果单独开过其它主机终端，再停那些辅助终端

## 8. 常用变体

默认 `real_simaligned` 真实链路已经使用 `use_center_as_goal:=true`。

如果要整体回退到旧真实参数链路：

```bash
./start_host.sh config:=/home/yxw/motion_planning_ws/config/host_start.legacy.conf
./start_robot.sh config:=/home/yxw/motion_planning_ws/config/robot_start.legacy.conf
```

如果要切回“先去 staging goal 再切换编队”的模式，可以临时覆盖：

```bash
./start_robot.sh use_center_as_goal:=false
```

如果要关闭编队控制、只跑导航链：

- 在 `ROBOT_LAUNCH_ARGS` 里把 `enable_shape_assembly:=true` 改成 `false`

或者临时覆盖：

```bash
./start_robot.sh enable_shape_assembly:=false
```
