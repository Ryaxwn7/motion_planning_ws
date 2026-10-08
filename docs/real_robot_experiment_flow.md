# Real Robot Experiment Flow

本文档记录一次实机聚集实验的推荐流程，包括主机、机器人、VRPN 动捕、轨迹发布、rosbag 录制、聚集启动和中断聚集。

## 0. 实验前检查

主机和机器人都需要使用同一个 ROS master。确认：

- 主机、机器人、动捕主机在同一网络内。
- 主机和机器人互相能 `ping` 通。
- 每台机器人 `ROS_IP` 指向本机可被主机访问的网卡地址。
- 机器人配置文件中的 `agent_id` 和机器人编号一致。
- Motive/VRPN 中刚体名称和轨迹发布节点配置一致。

主机工作区：

```bash
cd /home/bsrl-ubuntu/motion_planning_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

## 1. 主机启动 Host UI

主机执行：

```bash
cd /home/bsrl-ubuntu/motion_planning_ws
./start_host_ui.sh
```

在 Host UI 中确认：

- `robot_ids`: 例如 `[3,4,5,6]`
- `Shape`: 队形类型
- `Scale`: 队形尺度
- `Gather Goal`: `fastest`、`energy` 或 `space`
- host 是否启动地图、聚集节点和监控节点

点击：

```text
Start Host
```

## 2. 启动机器人

每台机器人分别执行 `start_robot.sh`。示例：

```bash
cd /home/bsrl-ubuntu/motion_planning_ws
./start_robot.sh config:=config/robot_start.robot3.conf
```

robot4 到 robot6 分别使用自己的配置：

```bash
./start_robot.sh config:=config/robot_start.robot4.conf
./start_robot.sh config:=config/robot_start.robot5.conf
./start_robot.sh config:=config/robot_start.robot6.conf
```

`start_robot.sh` 会启动底盘、雷达、定位、`move_base`、队形控制等机器人侧节点，并会尝试从主机参数服务器同步 `/robot_ids`、`/shape_type`、`/shape_scale` 和 `/shape_source`。

推荐先启动 host，再启动 robot。

## 3. 启动 VRPN 动捕和 TF

主机执行：

```bash
cd /home/bsrl-ubuntu/motion_planning_ws
./start_vrpn_client.sh server:=192.168.1.102
```

如需打开 TF 调整窗口：

```bash
./start_vrpn_tf_ui.sh server:=192.168.1.102
```

检查 VRPN pose：

```bash
rostopic list | grep vrpn
rostopic hz /vrpn_client_node/vrobot3/pose
```

检查 `world -> map`：

```bash
rosrun tf tf_echo world map
```

如果动捕主机 IP 不是 `192.168.1.102`，启动时替换 `server`。

## 4. 发布 VRPN 机器人轨迹

主机执行：

```bash
roslaunch move_base_client vrpn_traj.launch robot_ids:="[3,4,5,6]" save_dir:=/tmp/vrpn_traj
```

默认输入：

```text
/vrpn_client_node/vrobot3/pose
/vrpn_client_node/vrobot4/pose
/vrpn_client_node/vrobot5/pose
/vrpn_client_node/vrobot6/pose
```

默认输出：

```text
/vrpn_client_node/vrobot3/traj
/vrpn_client_node/vrobot4/traj
/vrpn_client_node/vrobot5/traj
/vrpn_client_node/vrobot6/traj
```

检查轨迹：

```bash
rostopic echo /vrpn_client_node/vrobot3/traj/header
```

如果实际刚体名是 `robot3` 而不是 `vrobot3`，需要修改轨迹节点命名规则，或使用旧版 `move_base_client laun.launch` 并提供 `/pose_throttle` 输入。

## 5. 录制 ROS Bag

确认 VRPN pose、轨迹、机器人导航和聚集话题正常后开始录包。

可使用已有录包 launch：

```bash
roslaunch fm2_gather record.launch output_bag:=/sda2/exp_bags/$(date +%Y%m%d_%H%M%S)_gather.bag
```

如果要明确录制 VRPN 轨迹，建议使用或扩展下面的命令：

```bash
rosbag record -O /sda2/exp_bags/$(date +%Y%m%d_%H%M%S)_gather_vrpn.bag \
  /tf /tf_static \
  /map /map_updates \
  /gather_signal /gather_center \
  /fm2_gather/preview_center \
  /fm2_gather/estimated_gather_cost \
  /shape_assembly/task \
  /vrpn_client_node/vrobot3/pose /vrpn_client_node/vrobot3/traj \
  /vrpn_client_node/vrobot4/pose /vrpn_client_node/vrobot4/traj \
  /vrpn_client_node/vrobot5/pose /vrpn_client_node/vrobot5/traj \
  /vrpn_client_node/vrobot6/pose /vrpn_client_node/vrobot6/traj \
  /robot3/move_base/GraphPlanner/plan \
  /robot4/move_base/GraphPlanner/plan \
  /robot5/move_base/GraphPlanner/plan \
  /robot6/move_base/GraphPlanner/plan
```

## 6. 启动聚集

在 Host UI 中先点击：

```text
Preview Gather
```

检查 RViz 中的预计算结果：

- `/fm2_gather/preview_center`
- `/fm2_gather/preview_path/robot<N>`
- `/fm2_gather/arrival_time/combined`
- `/shape_assembly/target_markers`

确认安全后点击：

```text
Send Gather=2
```

也可以使用倒计时：

```text
Countdown(s) -> Start Countdown
```

不用 UI 时可手动发布：

```bash
rostopic pub -1 /gather_signal std_msgs/UInt8 '{data: 3}'
rostopic pub -1 /gather_signal std_msgs/UInt8 '{data: 2}'
```

其中：

- `3`: 只预计算，不发送真实导航目标。
- `2`: 启动真实聚集。

## 7. 保存轨迹 CSV

实验结束前，如需保存 VRPN 轨迹 CSV：

```bash
rosservice call /vrpn_traj_publisher/save_traj
```

默认输出目录：

```text
/tmp/vrpn_traj
```

服务返回会包含每台机器人轨迹长度和总长度。

## 8. 中断聚集

需要中断聚集时，按这个顺序操作：

1. 在 Host UI 点击 `Stop Host`。
2. 在 Host UI 点击 `Force MoveBase ON`。

`Force MoveBase ON` 会对当前 `robot_ids` 执行：

- 停止 gather replanning。
- 取消各机器人当前 `move_base` goal。
- 发布 `/robotN/shape_assembly/force_move_base_mode = true`。
- 尝试通过 dynamic reconfigure 设置 `force_move_base_mode=true`。

这样机器人会退出当前聚集/队形控制逻辑，回到强制 `move_base` 模式，便于人工重新发送导航目标或安全处理。

恢复正常聚集/队形控制前，点击：

```text
Force MoveBase OFF
```

## 9. 停止实验

推荐停止顺序：

1. 停止聚集：`Stop Host`。
2. 如需要人工接管：`Force MoveBase ON`。
3. 停止 rosbag 录制。
4. 调用 `/vrpn_traj_publisher/save_traj` 保存 CSV。
5. 停止各机器人 `start_robot.sh`。
6. 停止 VRPN/TF 终端。
7. 关闭 Host UI。

## 10. 常用检查命令

```bash
rostopic echo /gather_center
rostopic echo /fm2_gather/estimated_gather_cost
rostopic echo /shape_assembly/task
rostopic echo /robot3/shape_assembly/status
rostopic echo /robot3/move_base/GraphPlanner/plan
rostopic hz /vrpn_client_node/vrobot3/pose
rostopic echo /vrpn_client_node/vrobot3/traj/header
```

FM2 计算调试数据默认保存到：

```text
~/.ros/fm2_gather_debug/run_*
```

如果设置了 `FM2_GATHER_DEBUG_DIR`，则保存到该环境变量指定的目录。
