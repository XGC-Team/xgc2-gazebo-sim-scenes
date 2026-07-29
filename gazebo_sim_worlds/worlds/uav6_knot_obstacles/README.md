# UAV6 Knot Obstacles

通用静态 Gazebo Classic 场景，逐项复现 `paper-leader` 的
`uav6_knot.launch` 障碍物配置：

- 16 个障碍物；
- 位置、姿态和尺度沿用源配置；
- cube、sphere、cylinder 使用 Gazebo 原生 collision；
- `icosahedron_1` 使用闭合凸三角网格作为真实静态物理 collision，
  visual 与 collision 引用同一网格和尺度。

Gazebo 坐标帧名为 `world`；数值坐标直接对应原实验的 `map`。模型名采用
`xgc2_obstacle_<source-name>` 协议，因此
`libxgc2_gazebo_scene_system.so` 会从 collision 自动发布规划几何与状态。
本场景不包含车辆、控制器、算法节点或 Gazebo 启动编排。
算法接入时应显式使用 `reference_frame_id:=world`；如必须保留 `map`，
则由平台提供 `map -> world` 单位静态变换。

安装后在既有 XGC2 `gazebo-server` 面板/自动化节点中直接选择：

```text
world=/opt/ros/noetic/share/gazebo_sim_worlds/worlds/catalog/uav6_knot_obstacles.world
```

独立手动回归命令：

```bash
roslaunch gazebo_ros empty_world.launch \
  world_name:="$(rospack find gazebo_sim_worlds)/worlds/catalog/uav6_knot_obstacles.world" \
  extra_gazebo_args:="-s libxgc2_gazebo_scene_system.so" gui:=true
```

`source_manifest.json` 是可审计的源配置快照。运行以下命令可重新生成并验证
world/mesh 资产：

```bash
python3 gazebo_sim_worlds/tools/generate_formation_scenes.py --check
python3 gazebo_sim_worlds/test/verify_formation_scenes.py --require-upstream
```
