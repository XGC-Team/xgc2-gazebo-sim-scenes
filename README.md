# XGC2 Gazebo Sim Scenes

This repository is the Gazebo Classic scene product for XGC2. A scene has two
parts:

- `gazebo_sim_worlds`: reusable world and model assets. This package does not
  own model-control logic.
- `xgc2_gazebo_scene`: the scene director. It owns deterministic obstacle
  motion and the XGC2 runtime scene-control interface.

For independently authored obstacle scenes, choose
[`scene_editable`](gazebo_sim_worlds/worlds/scene_editable/README.md). Its world
plugin consumes the unified scene runtime interface, materializes live geometry
and follows scene state. The separate user workflow chooses YAML; starting an
algorithm is not required. The adapter supports boxes, spheres, cylinders,
capsules, closed convex meshes and multi-part obstacles without filling openings.

The director currently retains both obstacle controllers without changing
their behavior:

- `libobstaclePathPlugin.so` is the legacy per-model path animation plugin used
  by existing dynamic world files.
- `libxgc2_gazebo_scene_system.so` is the XGC2-managed global scene plugin used
  for runtime motion commands and obstacle ground truth.

The catalog includes exact, algorithm-independent obstacle worlds for the
six-UAV knot and four-UGV figure-eight experiments. Their physical collisions
are source primitives or closed convex meshes. The global plugin discovers
those collisions and publishes both its native truth messages and the shared
`xgc2_geometry_msgs` planning contract; no planner-specific node is embedded in
the scene. It also reduces Gazebo's raw high-volume contact stream to a
latched, platform-independent forbidden-contact verdict. Normal ground and
self contacts are filtered before they leave the scene product.

Install both functional packages with:

```bash
sudo apt update
sudo apt install \
  ros-noetic-xgc2-gazebo-sim-worlds \
  ros-noetic-xgc2-gazebo-scene
```

The detailed reusable-world catalog is in
[`gazebo_sim_worlds/README.md`](gazebo_sim_worlds/README.md).
