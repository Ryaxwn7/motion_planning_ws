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
