# XGC2 Gazebo Sim Scenes

This repository is the Gazebo Classic scene product for XGC2. A scene has two
parts:

- `gazebo_sim_worlds`: reusable world and model assets. This package does not
  own model-control logic.
- `xgc2_gazebo_scene`: the scene director. It owns deterministic obstacle
  motion and the XGC2 runtime scene-control interface.

The worlds package installs `gazebo-client`, `gazebo-server` and
`gazebo-server-hybrid` from `process-definitions/` under
`/usr/share/xgc2/process-definitions/`. Servers consume an explicitly prepared
single-world SDF. Core passes its allocated `socketPath` to `native_world_start`
as `--socket-path`; the launcher forwards it to the native world host through
`XGC_SIM_SOCKET_PATH`, leaving the SDF's resource and component grants intact.
Unbound `GET /v1/describe?wait_ready_ms=N` waits asynchronously for the native
world and required components, up to 30000 ms. The GUI's `modelDatabaseUri`
remains a plain URI parameter and defaults to offline access.

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
those collisions and publishes the shared `xgc2_geometry_msgs` geometry library
and instances; no planner-specific node is embedded in
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
