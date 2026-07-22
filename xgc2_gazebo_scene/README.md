# XGC2 Gazebo Scene

`xgc2_gazebo_scene` is the Gazebo Classic scene director. It retains the
legacy per-model `libobstaclePathPlugin.so` path animator used by existing
worlds and provides the global XGC2 ground-truth scene layer. The SystemPlugin
is loaded with `gzserver`; it discovers `xgc2_obstacle_*` models, reads their
actual collision primitives, publishes convex geometry and synchronized state,
and executes deterministic motion programs against Gazebo simulation time.

The plugin deliberately does not own `gzserver`, ROS Core, world selection, or
model spawning. Existing XGC semantic operations keep using trusted Gazebo ROS
services for one-shot scene edits. Continuous motion is configured through:

- `/xgc2/gazebo/obstacles/configure_motions`
- `/xgc2/gazebo/obstacles/stop_motions`

Planning ground truth is published on:

- `/xgc2/simulation/obstacles/geometry` (latched)
- `/xgc2/simulation/obstacles/state` (latched latest snapshot, simulation time)

Only box, sphere, and cylinder collisions are admitted into the planning
contract. Unsupported collision shapes are reported and omitted instead of
being approximated silently.
