# Motion Planning Workspace

本仓库是我在 Raspberry Pi 4B 上维护的 ROS Noetic 工作空间，运行环境为 Ubuntu 20.04。工作空间基于 `ros_motion_planning`、`turn_on_wheeltec_robot` 等包进行了本地集成和二次开发，当前用于轮趣/Wheeltec 底盘的导航、定位、激光雷达接入与自定义路径规划实验。

## Environment

- Hardware: Raspberry Pi 4B
- OS: Ubuntu 20.04
- ROS: Noetic
- Build tool: `catkin_make`

## Workspace Layout

```text
motion_planning_ws/
├── src/
│   ├── ros_motion_planning/        # 全局/局部规划算法与 sim_env
│   └── turn_on_ws/src/
│       ├── turn_on_wheeltec_robot/ # 底盘、雷达、TF、导航启动文件
│       ├── my_planner/             # 自定义局部规划器
│       ├── my_ware/                # 辅助节点，例如 costmap_cleaner
│       ├── lsm10_ros/              # 传感器相关包
│       └── lsx10/                  # 镭神雷达驱动
├── build/                          # catkin 构建产物
└── devel/                          # catkin 开发环境
```

## Current Navigation Stack

当前多机器人导航入口主要使用：

- Launch file: `turn_on_wheeltec_robot/launch/motion_navigate_multi4.launch`
- Robot type: `mini_mec`
- Map: `exp_d2.yaml`
- Global planner: `fm2`
- Local planner: `my_planner/MyPlanner`
- Localization: `AMCL`
- Navigation framework: `move_base`

这套启动流程会拉起 `map_server`、底盘驱动、激光雷达、`robot_pose_ekf`、`amcl`、`move_base`，以及自定义的 `costmap_cleaner` 节点。

## Build

```bash
cd ~/motion_planning_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Run

启动当前默认的多机器人导航实例：

```bash
cd ~/motion_planning_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch turn_on_wheeltec_robot motion_navigate_multi4.launch
```

如果后续启动其他机器人实例，通常需要把 `map_started` 设为 `true`，避免重复启动 `map_server`。

## Real Robot Experiment

当前实机实验推荐走根目录脚本：

- 主机 UI：`./start_host_ui.sh`
- 主机命令行：`./start_host.sh`
- 机器人：`./start_robot.sh`
- 主机配置：`config/host_start.conf`
- 机器人配置：`config/robot_start.conf`、`config/robot_start.robot3.conf` 到 `config/robot_start.robot6.conf`

### 1. 编译

主机只需要构建 host 侧包：

```bash
cd /home/bsrl-ubuntu/motion_planning_ws
source /opt/ros/noetic/setup.bash
./build_host.sh
source devel/setup.bash
```

每台机器人构建 robot 侧包：

```bash
cd /home/bsrl-ubuntu/motion_planning_ws
source /opt/ros/noetic/setup.bash
./build_robot.sh
source devel/setup.bash
```

如果 `build_robot.sh` 提示缺少 ROS `serial` 包，需要先补齐底盘依赖，否则 `turn_on_wheeltec_robot` 可能无法编译。

### 2. 网络和定位检查

所有机器必须使用同一个 ROS master。当前默认 master 在 `config/host_start.conf` 和机器人配置中都是：

```text
ROS_MASTER_URI_VALUE='http://192.168.1.104:11311'
```

实验前确认：

- 主机和机器人互相能 `ping` 通。
- 每台机器人上的 `ROS_IP` 或配置文件 `ROS_IP_VALUE` 指向本机可被主机访问的网卡地址。
- 主机地图文件 `MAP_FILE_VALUE` 存在，当前配置为 `src/turn_on_ws/src/turn_on_wheeltec_robot/map/5x10_2026.yaml`。
- 如果使用动捕位姿，先启动 VRPN 客户端并完成 `world -> map` 标定：

```bash
cd /home/bsrl-ubuntu/motion_planning_ws
./start_vrpn_client.sh server:=192.168.1.117
```

### 3. 主机配置

主机侧编辑 `config/host_start.conf`。实机前至少确认这些项：

- `START_ROSCORE`：是否由脚本启动 roscore。当前为 `false`，表示需要外部 master 已经存在。
- `LAUNCH_MAP_SERVER`：是否启动地图。当前为 `true`。
- `ROS_MASTER_URI_VALUE`、`ROS_IP_VALUE`：ROS 网络配置。
- `MAP_FILE_VALUE`：实机地图。
- `HOST_LAUNCH_ARGS` 中的 `robot_ids` 和 `agent_number`：当前是 `[3,4,5,6]` 和 `4`。
- `shape_type`、`shape_scale`、`shape_source`：队形类型、尺度和形状来源。
- `mission`：FM2 聚集目标，支持 `fastest`、`energy`、`space`。
- `gather_config`：FM2 参数文件，当前指向 `src/fm2_gather/config/gather_param_real_simaligned.yaml`。

`mission` 的含义：

- `fastest`：选择最短完成时间的聚集中心。
- `energy`：选择总能耗更小的聚集中心。
- `space`：选择满足障碍物最小距离阈值后的最快聚集中心。

`space` 模式的障碍物距离阈值在 `src/fm2_gather/config/gather_param_real_simaligned.yaml` 中调整：

```yaml
space_min_obstacle_distance: 0.7
```

### 4. 机器人配置

每台机器人建议使用自己的配置文件：

```bash
./start_robot.sh config:=config/robot_start.robot3.conf
./start_robot.sh config:=config/robot_start.robot4.conf
./start_robot.sh config:=config/robot_start.robot5.conf
./start_robot.sh config:=config/robot_start.robot6.conf
```

每份机器人配置中至少确认：

- `ROS_MASTER_URI_VALUE`：指向主机 ROS master。
- `ROS_IP_VALUE`：指向该机器人本机 IP，留空时使用当前环境变量 `ROS_IP`。
- `agent_id`：必须和该机器人编号一致，例如 robot5 使用 `agent_id:=5`。
- `agent_number`：和主机机器人数量一致。
- `enable_shape_assembly:=true`：开启队形控制。
- `use_center_as_goal:=true`：先由 `move_base` 前往共享聚集中心，再切换到队形控制。
- `shape_assembly_config`：当前使用 `shape_assembly_real_simaligned.yaml`。
- `global_costmap_config`、`local_costmap_config`：实机 costmap 参数。

`start_robot.sh` 启动时会尝试从 ROS 参数服务器同步主机侧共享参数，包括 `/robot_ids`、`/shape_type`、`/shape_scale` 和 `/shape_source`。因此推荐先启动主机，再启动机器人。若机器人先启动，机器人会使用本地配置文件中的默认值。

### 5. 启动顺序

主机上启动 Host UI：

```bash
cd /home/bsrl-ubuntu/motion_planning_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./start_host_ui.sh
```

在 UI 中确认 `robot_ids`、`shape_type`、`shape_scale`、`mission` 后启动 host。也可以不用 UI，直接启动命令行主机：

```bash
./start_host.sh
```

然后分别在 robot3 到 robot6 上启动：

```bash
cd /home/bsrl-ubuntu/motion_planning_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./start_robot.sh config:=config/robot_start.robot5.conf
```

每台机器人把配置文件换成自己的 `robot_start.robotN.conf`。

### 6. 聚集和队形实验

Host UI 中当前聚集相关按钮：

- `Preview Gather`：发布 `/gather_signal = 3`，只提前计算 FM2 聚集中心、融合代价场和机器人路径，不给机器人发送真实导航目标。
- `Send Gather=2`：发布 `/gather_signal = 2`，启动真实聚集。
- `Countdown(s)` 和 `Start Countdown`：设置倒计时，到点后自动执行 `Send Gather=2`。

推荐实机流程：

1. 启动 host 和所有机器人。
2. 在 Host UI 设置机器人编号、队形类型、尺度和 `mission`。
3. 点击 `Preview Gather`，先在 RViz 中检查中心点、总代价场和路径。
4. 检查无误后点击 `Send Gather=2`，或用倒计时启动真实聚集。

不用 UI 时可以手动发布：

```bash
rostopic pub -1 /gather_signal std_msgs/UInt8 '{data: 3}'  # 只预计算
rostopic pub -1 /gather_signal std_msgs/UInt8 '{data: 2}'  # 真实聚集
```

### 7. RViz 和话题检查

主机侧常看：

```bash
rostopic echo /gather_center
rostopic echo /shape_assembly/task
rostopic echo /gather_replan_event
rostopic echo /fm2_gather/estimated_gather_cost
```

预计算结果可在 RViz 添加：

- `/fm2_gather/preview_center`
- `/fm2_gather/preview_path/robot<N>`
- `/fm2_gather/arrival_time/combined`
- `/shape_assembly/target_markers`

机器人侧以 robot5 为例：

```bash
rostopic echo /robot5/shape_assembly/status
rostopic echo /robot5/move_base/GraphPlanner/plan
rostopic hz /robot5/move_base/local_costmap/costmap
rostopic hz /robot5/move_base/local_costmap/costmap_updates
```

如果机器人进入 shape control 后需要使用本地静态/动态障碍物，确认 `shape_assembly_real_simaligned.yaml` 中的 `use_local_costmap_avoid` 已开启，并且机器人确实发布了完整 local costmap。只有 update 话题而没有完整 costmap 时，shape control 无法直接构建可用障碍物场。

### 8. FM2 计算结果保存

当前 `src/fm2_gather/launch/laun.launch` 默认设置了：

```xml
<param name="save_data" type="bool" value="true" />
```

每次 `Preview Gather` 或真实 `Send Gather=2` 触发 FM2 计算后，都会保存一个 run 文件夹。默认位置：

```text
~/.ros/fm2_gather_debug/run_*
```

如果设置了环境变量 `FM2_GATHER_DEBUG_DIR`，则保存到该目录。每个 run 目录用于检查本次聚集计算，包括总代价场、融合后的代价场、速度场、每个机器人的时间场和路径等调试数据。

### 9. 停止顺序

建议先停止所有机器人上的 `start_robot.sh`，再停止主机 UI 或 `start_host.sh`。如果单独启动过 VRPN、RViz、roscore 或其它辅助节点，最后再关闭这些进程。

### 10. 常见问题

- Host 仍监听 robot1：确认 Host UI 和 `config/host_start.conf` 中的 `robot_ids` 已改对；host 侧节点启动后才会按这些参数订阅对应 `/robotN/...` 话题。
- 机器人队形仍是旧 shape：先启动 host，再启动 robot；或在 Host UI 中重新发送聚集命令，使 `/shape_type`、`/shape_scale` 等共享参数更新。
- robot 到中心后没有进入 shape control：检查 `enable_shape_assembly`、`use_center_as_goal`、`/robotN/shape_assembly/status` 和 `/shape_assembly/task`。
- 没有重规划：检查 `/gather_started`、`/gather_replan_event`、每个机器人的 `GraphPlanner/plan` 和 `GraphPlanner/arrival_time` 是否正常发布。
- shape control 阶段不避障：检查机器人完整 local costmap 话题，而不只是 `costmap_updates`；同时确认 `shape_assembly_real_simaligned.yaml` 的 local costmap 话题后缀和实际命名一致。

## VRPN 与 TF 调整

根目录新增了 VRPN 客户端和 `world`/`map` TF 调整脚本，用于接入动作捕捉服务器并在实机实验时在线校准坐标系。

启动 VRPN 客户端并打开 TF 调整窗口：

```bash
cd ~/motion_planning_ws
./start_vrpn_client.sh server:=192.168.1.117
```

等价的 UI 快捷入口：

```bash
./start_vrpn_tf_ui.sh server:=192.168.1.117
```

常用参数：

- `server:=192.168.1.117`：VRPN 服务器地址。
- `parent_frame:=world`、`child_frame:=map`：发布的 TF 父/子坐标系，默认发布 `world -> map`。
- `x:=0.0 y:=0.0 z:=0.0`：平移量，单位为米。
- `roll:=0.0 pitch:=0.0 yaw:=0.0`：旋转量，单位为弧度。
- `tf_backend:=ui`：默认模式，打开 `scripts/tf_adjust_ui.py`，可用窗口实时微调并保存/加载 JSON 配置。
- `tf_backend:=live`：后台发布动态 TF，可用 `rosparam set /live_map_world_tf/x 0.1` 等命令实时修改参数。
- `tf_backend:=tf2`：使用 `tf2_ros static_transform_publisher` 发布静态 TF。
- `tf_backend:=tf`：使用 ROS 旧版 `tf static_transform_publisher` 按 `period_ms` 周期发布。

示例：

```bash
./start_vrpn_client.sh server:=192.168.1.117 parent_frame:=world child_frame:=map tf_backend:=ui
./start_vrpn_client.sh server:=192.168.1.117 tf_backend:=live yaw:=1.5708
```

## Notes

- 根目录的 `.gitignore` 已忽略 `build/`、`devel/`、嵌套 `build/devel`、IDE 配置和本地 zip 备份文件。
- 该仓库保留的是本地开发后的统一工作空间，不再保留原始嵌套 Git 历史。
- `src/ros_motion_planning/README.md` 仍然保留了上游规划模块的详细说明，可作为算法实现参考。
