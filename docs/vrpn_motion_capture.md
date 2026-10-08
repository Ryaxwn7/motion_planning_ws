# VRPN Motion Capture And Trajectory

本文档说明本工作空间中 VRPN 动捕接入、坐标系发布、机器人轨迹发布和轨迹保存的相关功能。

## 相关入口

- `start_vrpn_client.sh`: 启动 `vrpn_client_ros`，同时发布 `world -> map` TF。
- `start_vrpn_tf_ui.sh`: `start_vrpn_client.sh tf_backend:=ui` 的快捷入口，启动后打开 TF 调整窗口。
- `move_base_client/launch/vrpn_traj.launch`: 根据 VRPN pose 发布机器人轨迹。
- `move_base_client/src/vrpn_traj_publisher.cpp`: 轨迹发布节点源码。
- `move_base_client/src/vr_path_transformer.cpp`: 旧版轨迹发布节点，使用 `robotN/pose_throttle` 输入。

## 启动 VRPN 客户端

主机上执行：

```bash
cd /home/bsrl-ubuntu/motion_planning_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./start_vrpn_client.sh server:=192.168.1.102
```

`server` 是 VRPN server 地址，通常是运行 Motive/动捕服务的主机 IP。脚本默认使用：

```text
server:=192.168.1.102
parent_frame:=world
child_frame:=map
tf_backend:=ui
```

如果只想打开等价的 TF 调整 UI：

```bash
./start_vrpn_tf_ui.sh server:=192.168.1.102
```

## TF 标定

`start_vrpn_client.sh` 会同时发布 `world -> map`。常用参数：

```bash
./start_vrpn_client.sh \
  server:=192.168.1.102 \
  parent_frame:=world \
  child_frame:=map \
  x:=0.0 y:=0.0 z:=0.0 \
  roll:=0.0 pitch:=0.0 yaw:=0.0 \
  tf_backend:=ui
```

`tf_backend` 支持：

- `ui`: 打开 `scripts/tf_adjust_ui.py`，可实时调整并保存/加载 TF。
- `live`: 后台发布动态 TF，可通过 `/live_map_world_tf/*` 参数在线修改。
- `tf2`: 使用 `tf2_ros static_transform_publisher` 发布静态 TF。
- `tf`: 使用旧版 `tf static_transform_publisher` 周期发布 TF。

检查 TF：

```bash
rosrun tf tf_echo world map
```

## 检查 VRPN 话题

启动 VRPN 后检查话题：

```bash
rostopic list | grep vrpn
rostopic echo /vrpn_client_node/vrobot3/pose
```

话题名取决于 Motive/VRPN 中刚体名称。若刚体名是 `vrobot3`，通常会发布：

```text
/vrpn_client_node/vrobot3/pose
/vrpn_client_node/vrobot4/pose
/vrpn_client_node/vrobot5/pose
/vrpn_client_node/vrobot6/pose
```

若刚体名是 `robot3`，话题会变成：

```text
/vrpn_client_node/robot3/pose
```

后续轨迹发布节点必须和实际刚体名称匹配。

## 发布机器人轨迹

推荐使用 `vrpn_traj_publisher`：

```bash
roslaunch move_base_client vrpn_traj.launch robot_ids:="[3,4,5,6]"
```

默认参数下，它订阅：

```text
/vrpn_client_node/vrobot3/pose
/vrpn_client_node/vrobot4/pose
/vrpn_client_node/vrobot5/pose
/vrpn_client_node/vrobot6/pose
```

发布：

```text
/vrpn_client_node/vrobot3/traj
/vrpn_client_node/vrobot4/traj
/vrpn_client_node/vrobot5/traj
/vrpn_client_node/vrobot6/traj
```

轨迹消息类型为 `nav_msgs/Path`。

常用参数：

```bash
roslaunch move_base_client vrpn_traj.launch \
  robot_ids:="[3,4,5,6]" \
  vrpn_prefix:=/vrpn_client_node \
  input_suffix:=pose \
  output_suffix:=traj \
  fixed_frame:=world \
  use_msg_frame:=true \
  min_distance:=0.01 \
  max_poses:=0 \
  save_dir:=/tmp/vrpn_traj
```

参数说明：

- `robot_ids`: 机器人编号列表。
- `input_suffix`: 输入 pose 后缀，默认 `pose`。
- `output_suffix`: 输出轨迹后缀，默认 `traj`。
- `min_distance`: 相邻轨迹点距离小于该值时跳过，单位米。
- `max_poses`: 最大保留点数，`0` 表示不限制。
- `save_dir`: 调用保存服务时 CSV 输出目录。

## 保存轨迹 CSV

轨迹发布节点提供保存服务：

```bash
rosservice call /vrpn_traj_publisher/save_traj
```

默认保存到：

```text
/tmp/vrpn_traj/vrobot3_traj.csv
/tmp/vrpn_traj/vrobot4_traj.csv
/tmp/vrpn_traj/vrobot5_traj.csv
/tmp/vrpn_traj/vrobot6_traj.csv
```

CSV 字段：

```text
stamp,frame_id,x,y,z,qx,qy,qz,qw
```

服务返回中会打印每台机器人轨迹点数、轨迹长度和总长度。

## 录制 ROS Bag

建议录制 VRPN pose、轨迹、TF、聚集结果和机器人导航相关话题。示例：

```bash
rosbag record -O /sda2/exp_bags/$(date +%Y%m%d_%H%M%S)_exp.bag \
  /tf /tf_static \
  /vrpn_client_node/vrobot3/pose /vrpn_client_node/vrobot3/traj \
  /vrpn_client_node/vrobot4/pose /vrpn_client_node/vrobot4/traj \
  /vrpn_client_node/vrobot5/pose /vrpn_client_node/vrobot5/traj \
  /vrpn_client_node/vrobot6/pose /vrpn_client_node/vrobot6/traj \
  /gather_signal /gather_center \
  /fm2_gather/preview_center \
  /fm2_gather/estimated_gather_cost \
  /shape_assembly/task
```

已有的 `src/fm2_gather/launch/record.launch` 主要录制地图、规划、costmap 和 `/robotN/path` 等话题；如需保存 VRPN 轨迹，建议按实际 VRPN 刚体名补充 `/vrpn_client_node/*/pose` 和 `/vrpn_client_node/*/traj`。

## 旧版节点

`move_base_client/launch/laun.launch` 会启动 `vr_path_transformer`。旧版节点监听：

```text
/vrpn_client_node/robotN/pose_throttle
```

发布：

```text
/vrpn_client_node/robotN/trajectory
```

它在 Ctrl+C 时保存：

```text
/home/bsrl-ubuntu/saved_paths/robot_N_path.csv
```

如果当前 VRPN 只发布 `/pose` 而不是 `/pose_throttle`，优先使用新版 `vrpn_traj.launch`。

## 常见检查

```bash
rostopic hz /vrpn_client_node/vrobot3/pose
rostopic echo /vrpn_client_node/vrobot3/traj/header
rosrun tf view_frames
```

如果没有轨迹输出，先确认：

- Motive/VRPN 中刚体名称是否为 `vrobotN`。
- `vrpn_traj.launch` 的 `robot_ids` 是否和刚体编号一致。
- 输入话题是否为 `/vrpn_client_node/vrobotN/pose`。
- `roscore` 和所有节点是否使用同一个 `ROS_MASTER_URI`。
