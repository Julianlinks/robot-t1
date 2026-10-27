# robot-t1

ROS 2 机械臂仿真分选与贴片系统。

本工程实现四轴 SCARA 吸盘机械臂的固定工位作业：读取薄片类别和位姿，抓取源区薄片，按红/绿/蓝分选，对准同类槽位，下降放置，抬升并检查薄片的实际位置和朝向。默认场景包含六片薄片、六个槽位和一张工作台。

**交付状态：Python 离线任务演示及 16 项单元/资源检查已通过；当前 Windows 环境没有 ROS 2，C++ 插件编译、ROS 通信及 Gazebo 物理运行尚未实测。必须在下述 Ubuntu 环境构建并执行端到端验收，才能确认仿真集成通过。**

## 已确定的方案

| 项目 | 配置 |
|---|---|
| 操作系统 | Ubuntu 24.04 |
| ROS | ROS 2 Jazzy |
| 仿真器 | Gazebo Harmonic / gz-sim8 |
| 机械臂 | 工程内自定义 SCARA，R-R-P-R 四轴，臂长 0.32 m + 0.28 m |
| 控制 | gz_ros2_control + JointTrajectoryController |
| 运动生成 | 解析逆解，固定肘型，五次多项式关节轨迹 |
| 工具 | 吸盘近距离捕获 + Gazebo 可拆固定约束 |
| 分选依据 | Gazebo 模型类别和物理位姿真值，接口允许替换为视觉节点 |
| 贴片定义 | 薄片按指定 XY、Z、yaw 对位下放，短暂停留，释放并验收 |
| 默认验收阈值 | 位置误差 ≤ 4 mm，偏航角误差 ≤ 0.08 rad，薄片已脱离吸盘 |

Jazzy 与 Harmonic 的组合及控制插件用法依据 [gz_ros2_control 官方文档](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)。

这是一套固定平面工位的仿真工程。类别来自模型命名 `part_<class>_<index>`，没有宣称实现相机识别。当前贴片过程没有胶层、真空流量、焊接、压力传感器或力控模型；4 mm 是演示验收阈值，不能作为电子元件贴装精度指标。新增任意障碍物需要补充碰撞规划。当前没有集成 MoveIt 2：四轴固定工位使用解析逆解直接控制，可在保留任务接口的前提下替换运动后端。

## 工程结构

```text
src/
  sort_place_interfaces/   Part / PartArray、Suction 服务、SortPlace 动作
  sort_place_core/         任务状态机、SCARA 正逆解、限速轨迹、ROS 动作服务器
  sort_place_sim/          URDF、Gazebo 世界、吸附 C++ 插件、控制器和启动配置
scripts/
  generate_scene.py        从 cell.json 生成工作台、薄片与槽位
  smoke_test.py            启动独立无界面场景并检查六片物理放置结果
tests/                    不依赖 ROS 的 16 项测试
```

```mermaid
flowchart LR
    G[Gazebo 物理世界] -->|实际薄片位姿与类别| P[/sort_place/parts]
    P --> T[任务状态机]
    U[ROS Action 客户端] -->|/sort_place/run| T
    T --> K[SCARA 逆解与限速轨迹]
    K --> C[FollowJointTrajectory]
    C --> R[gz_ros2_control]
    R --> G
    T -->|Suction 服务| V[近距离吸附插件]
    V -->|可拆固定约束| G
    T --> S[/sort_place/status]
```

## 1. 安装和构建

在 Ubuntu 24.04 中操作。先按 [ROS 2 Jazzy 官方安装说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) 配置软件源并安装 ROS，然后将整个工程目录复制到 `~/ros2_sort_place_ws`。

```bash
sudo apt update
sudo apt install -y build-essential cmake python3-colcon-common-extensions \
  python3-rosdep ros-jazzy-desktop ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro

source /opt/ros/jazzy/setup.bash
cd ~/ros2_sort_place_ws
# 仅首次使用 rosdep 时初始化；已初始化则跳过这一行。
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
```

C++ 插件依赖 `gz_sim_vendor`、`gz_plugin_vendor`、`gz-sim8` 和 `gz-plugin2`。它们由上述 ROS/Gazebo 依赖提供。首次构建若失败，应先检查依赖解析和第一条编译错误；本交付没有把编译未执行当作编译通过。

## 2. 启动仿真

终端 A：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_sort_place_ws
source install/setup.bash
ros2 launch sort_place_sim cell.launch.py
```

启动顺序为 Gazebo → 生成机械臂 → joint_state_broadcaster → arm_controller → 任务服务器；任何生成/控制器加载进程返回非零都会结束启动链。仿真默认自动播放。Gazebo 显示工位和薄片；RViz 显示机器人与 TF。

终端 B：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_sort_place_ws
source install/setup.bash
ros2 action send_goal /sort_place/run sort_place_interfaces/action/SortPlace \
  '{max_parts: 0}' --feedback
```

`max_parts: 0` 处理本次快照中所有合格的源区薄片；`max_parts: 2` 最多处理两片。默认任务按对象 ID 排序执行，因此先蓝、再绿、再红。薄片最终进入各自颜色的目标槽。

```bash
ros2 topic echo /sort_place/status
ros2 topic echo /sort_place/parts
ros2 control list_controllers
```

预期六片完成时动作返回 `success: true`、`completed: 6`。没有可处理工件时返回成功和 `completed: 0`。置信度不足、已经吸附或位于源区外的工件会跳过；合格工件没有对应槽位时任务失败。

仅无界面运行：

```bash
ros2 launch sort_place_sim cell.launch.py rviz:=false 'gz_args:=-s -r -v 3'
```

## 3. 端到端验收

先关闭手动启动的同名工位，避免节点、服务和话题重名，再在已 source 环境的终端执行：

```bash
python3 scripts/smoke_test.py
```

脚本自行启动一套新的无界面仿真，等待任务服务器，执行六片任务，检查每片的实际位置、朝向和释放状态，并再次提交任务确认不重复加工。结束后关闭自己启动的仿真。日志位于 `work/gazebo_smoke.log`，物理位姿检查报告位于 `work/gazebo_smoke_result.json`。脚本只有在全部检查通过时才正常退出。默认等待批次 240 秒，可用 `--timeout 360` 调整；轨迹等待针对接近实时的仿真，机器过慢需要调整超时系数。

这些端到端检查尚未在当前交付环境执行，报告文件也不会预先伪造。

## 4. 不安装 ROS 的逻辑验证

Linux：

```bash
python3 -m unittest discover -s tests -v
PYTHONPATH=src/sort_place_core python3 -m sort_place_core.offline_demo \
  --config src/sort_place_sim/config/cell.json
```

PowerShell：

```powershell
python -m unittest discover -s tests -v
$env:PYTHONPATH = (Resolve-Path src/sort_place_core).Path
python -m sort_place_core.offline_demo --config src/sort_place_sim/config/cell.json
```

离线演示使用内存后端，六片完成对应 42 段动作，只用于验证流程、槽位和运动计算。它不运行 ROS，也不验证 Gazebo 物理或 C++ 编译。

## 动作流程与失败处理

```text
读取新鲜观测 / 恢复已占用槽位
  → 筛选源区工件 / 分配同类空槽
  → 对整段动作进行逆解和 TCP 高度预检查
  → 当前 XY 抬到安全高度
  → 源区上方 → 下降 → 开吸盘
  → 抬升 / 检查工件跟随 → 安全高度转移
  → 目标槽上方 → 下降 / 停留 → 释放
  → 抬升 → 等待物理稳定 → 实际位姿验收
  → 提交槽位占用 / 处理下一片
```

- 一次只接受一个批次，避免两个任务同时驱动机械臂。
- 轨迹具有解析速度/加速度上界；动作结果还会与实际关节位置比较。
- 吸附要求 TCP 与工件水平偏差不超过 4 mm，TCP 高于工件中心 3–10 mm，工件基本平放；通过后创建 Gazebo 物理约束，不修改工件坐标。
- 工件观测和关节状态超过 1 秒未更新时停止继续规划。运动、服务和停留均有墙钟超时，仿真暂停不会无限等待。
- 取消和运动异常会请求控制器停止；已吸附的工件保持吸附。停止请求未得到确认时输出错误日志，不声称机械臂已经停稳。
- 任务失败或取消后，任务服务器锁定后续批次；检查现场并完整重启仿真后再运行。
- 吸附服务响应丢失、放置失败或取消后，目标槽位会在本进程中保留为待检查状态，避免复用。
- 节点重启可以由实际薄片位置恢复已占槽位；未决故障的保留记录没有跨进程持久化。故障后建议关闭整套 launch 并重新启动新世界。不要只重启任务节点来绕过故障。

取消可由 ROS Action 客户端调用 `goal_handle.cancel_goal_async()`；取消是软件动作请求，不能代替真实设备的硬件急停。工程只提供仿真后端。

## 修改工件、分选规则和位置

编辑 `src/sort_place_sim/config/cell.json`：

- `parts` 指定初始工件及类别；`slots` 指定同类目标槽。
- `pose` 格式为 `[x, y, z, yaw]`，单位为米、弧度，参考系为 `world`，表示薄片中心。
- `tcp_part_offset` 是抓取时 TCP 相对薄片中心的 Z 偏移。默认薄片厚度 8 mm，偏移 6 mm，留下 2 mm 接近间隙。
- `source_x/y/z_*` 定义可抓取区域，`safe_z` 定义转移高度。
- `press_dwell` 只是定位后停留时间，不表示施加压力。

修改工件/槽位后重新生成世界并重新构建：

```bash
python3 scripts/generate_scene.py
python3 -m unittest discover -s tests -v
colcon build --symlink-install
```

默认生成器包含红绿蓝三种颜色，增加类别需要同时扩展其颜色映射。修改机械臂尺寸时需要同步修改 URDF、`kinematics.py` 和关节控制限制；资源检查能覆盖部分配置漂移，但不能替代物理验证。

## 接入真实视觉或其他机械臂

视觉节点可发布同一 `PartArray`，需要稳定且唯一的 `object_id`、类别、`world` 位姿、置信度和时间戳。相机坐标必须经过标定和 TF 转换。当前真值发布位于 `cell_vacuum.cpp` 的 `PostUpdate()`；换成视觉输入时应关闭/替换这一发布，避免两个数据源竞争。吸附插件仍用 Gazebo 模型名称定位工件，因此仿真视觉跟踪 ID 需要映射到模型 ID。生产真空反馈应来自工具驱动，不应伪造。

换 UR5、xArm 或其他六轴机械臂时，保留 `Engine` 的业务流程，替换 `TaskServer.move()`/运动学后端，并接入相应 URDF、控制器、工具和 MoveIt 2 碰撞场景。当前自定义 SCARA 模型不是某个商业机器人的精确数字孪生。

## 常见排查

| 现象 | 检查 |
|---|---|
| 找不到 `libcell_vacuum.so` | 确认 colcon 构建成功，已 source install/setup.bash，并通过提供的 launch 启动 |
| 找不到控制器 | 查看 Gazebo 的 gz_ros2_control 加载日志、URDF 控制器 YAML 路径和 spawner 返回码 |
| `nozzle_link not found` | 检查 URDF/SDF 转换结果，TCP 吸盘必须保留为 joint4 的活动子连杆 |
| 吸附超出范围 | 核对工具原点、工件中心 Z、TCP 偏移和实际关节位置 |
| 任务等待/报观测过期 | 确认仿真播放、ROS_DOMAIN_ID 一致、/clock 和 /sort_place/parts 在更新 |
| 放置误差失败 | 检查关节跟踪、吸附间隙、物理稳定时间、场景碰撞，不要仅放宽阈值 |
| 第二次任务 completed 为 0 | 已放好的工件不在源区，这是正常行为；完整重启 launch 可恢复初始场景 |

物理约束使用的 Gazebo API 参考 [DetachableJoint 组件](https://gazebosim.org/api/sim/8/structgz_1_1sim_1_1components_1_1DetachableJointInfo.html)。
