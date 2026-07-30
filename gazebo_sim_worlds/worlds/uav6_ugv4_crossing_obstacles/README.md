# UAV6 + UGV4 Crossing Obstacles (Manuscript-2)

Manuscript-2 混合编队（6 PX4 q=4 + 4 Scout Mini q=2）的穿越场景，逐项复现
`paper-leader` 的 `config/scenarios/uav6_ugv4_crossing/obstacles.yaml`：

- 4 个障碍物，**全部是已知匀速运动体**，没有任何静态体；
- 位置、姿态、尺度沿用源配置（`position` 是**任务 t = 0** 的位姿）；
- sphere、cylinder、cube 使用 Gazebo 原生 collision，无凸网格模板。

Gazebo 坐标帧名为 `world`；数值坐标直接对应原实验的 `map`。模型名采用
`xgc2_obstacle_<source-name>` 协议，因此
`libxgc2_gazebo_scene_system.so` 会从 collision 自动发布规划几何与状态。
本场景不包含车辆、控制器、算法节点或 Gazebo 启动编排。

## 运动是运行时装载的，不在 SDF 里

四个模型都以 `<static>true</static>` 生成。恒定 twist 由平台作业
`simulation.gazebo-configure-obstacle-motion`（`MODE_CONSTANT_TWIST`）在运行时
挂上，由 M2 的 “arm moving obstacles” 工作流下发。这不是折中，而是必须：
`MotionController::Configure` 在**作业被服务的那一刻**同时锁定原点位姿和时间零点，
所以 SDF 里预先给速度只会让原点绑定到错误的位姿。

因此有两个后果，两者都必须记住：

1. **未运行装载工作流的一次 take 看起来完全正常。** 场景插件发布的是
   `is_static = IsStatic() && (!controlled || mode == "hold")`，即 `is_static`
   为 true、速度为零——仅凭这一个字段无法与“本来就该静止的物体”区分。这不是安全
   问题（证书用观测到的占据与速度构造），但整个 take 不含任何动障碍证据，通常表现
   为算法侧静默停在 `waiting`。算法工作区的
   `m2_moving_obstacle_watchdog.py` 就是为此存在的告警。
2. **生成位姿带一个可测偏置。** 每个障碍物按
   `P = position - linear_velocity * spawn_lead_seconds` 生成，
   `spawn_lead_seconds` 是“该障碍物的作业被服务”到“第一个 rolling SyncTrigger”
   之间实测的滞后。首跑取 0.0，实测后写回 `source_manifest.json` 并重新生成，
   `position` 始终保持导出真值以便上游比对。

## 安装后使用

```text
world=<products-install>/share/gazebo_sim_worlds/worlds/catalog/uav6_ugv4_crossing_obstacles.world
```

## 回归

```bash
python3 tools/generate_formation_scenes.py --check
python3 test/verify_formation_scenes.py --paper-leader-src <paper-leader>/ros1_ws/src
```
