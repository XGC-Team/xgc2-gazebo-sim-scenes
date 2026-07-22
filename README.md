# XGC2 Gazebo Sim Scenes

This repository is the Gazebo Classic scene product for XGC2. A scene has two
parts:

- `gazebo_sim_worlds`: reusable world and model assets. This package does not
  own model-control logic.
- `xgc2_gazebo_scene`: the scene director. It owns deterministic obstacle
  motion and the XGC2 runtime scene-control interface.

The director currently retains both obstacle controllers without changing
their behavior:

- `libobstaclePathPlugin.so` is the legacy per-model path animation plugin used
  by existing dynamic world files.
- `libxgc2_gazebo_scene_system.so` is the XGC2-managed global scene plugin used
  for runtime motion commands and obstacle ground truth.

Install both functional packages with:

```bash
sudo apt update
sudo apt install \
  ros-noetic-xgc2-gazebo-sim-worlds \
  ros-noetic-xgc2-gazebo-scene
```

The detailed reusable-world catalog is in
[`gazebo_sim_worlds/README.md`](gazebo_sim_worlds/README.md).
