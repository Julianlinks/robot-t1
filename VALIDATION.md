# 交付验证记录

日期：2026-10-27。执行环境：Windows / PowerShell，Python 3.12.11。

已执行：

```text
python scripts/generate_scene.py
python -m unittest discover -s tests -v
python -m compileall -q src scripts
PYTHONPATH=src/sort_place_core python -m sort_place_core.offline_demo --config src/sort_place_sim/config/cell.json
```

结果：16 项离线测试通过；Python 源文件语法检查通过；六片离线任务完成，六个槽位占用，42 段动作。

覆盖内容：随机正逆解往返、不可达/非有限目标、轨迹速度和加速度约束、完整及分批作业、重复加工抑制、携片时取消、物理跟随失效模拟、服务响应丢失、放置误差、低置信度、槽位不足、重启后占用恢复、消息包及场景/URDF 资源契约。

尚未执行：colcon 构建、C++ 编译、ROS 节点通信、Gazebo 吸附/释放物理运行、实际端到端分选验收。当前环境没有 ROS 2 / Gazebo，WSL 发行版查询也未获系统访问权限。离线测试不能证明这些集成检查通过。

Ubuntu 上构建后执行 `python3 scripts/smoke_test.py`，用实际物理位姿确认集成结果。具体步骤见 README.md。
