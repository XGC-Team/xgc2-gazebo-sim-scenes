# XGC2 Gazebo Scene

`xgc2_gazebo_scene` is the Gazebo Classic scene director. It retains the
legacy per-model `libobstaclePathPlugin.so` path animator used by existing
worlds and provides the global XGC2 ground-truth scene layer. The SystemPlugin
is loaded with `gzserver`; it discovers `xgc2_obstacle_*` models, reads their
actual collisions, publishes convex geometry and synchronized state, and
executes deterministic motion programs against Gazebo simulation time.

The plugin deliberately does not own `gzserver`, ROS Core, world selection, or
model spawning. Existing XGC semantic operations keep using trusted Gazebo ROS
services for one-shot scene edits. Continuous motion is configured through:

- `/xgc2/gazebo/obstacles/configure_motions`
- `/xgc2/gazebo/obstacles/stop_motions`

Planning ground truth is published on:

- `/xgc2/simulation/obstacles/geometry` (latched)
- `/xgc2/simulation/obstacles/state` (latched latest snapshot, simulation time)
- `/xgc2/simulation/obstacles/geometry_library` (standard planning library,
  latched)
- `/xgc2/simulation/obstacles/instances` (standard planning instances,
  latched latest snapshot at 30 Hz)

High-fidelity regression collision truth is published independently of the
planning geometry:

- `/xgc2/simulation/physical_collision` (`std_msgs/Bool`, latched)
- `/xgc2/simulation/physical_collision_detail` (`std_msgs/String`, latched on
  the first forbidden contact)

The SystemPlugin subscribes to Gazebo's raw physics contacts internally. It
classifies contacts by top-level model, so wheel/chassis and rotor/body
self-contact is ignored. A tracked non-static model touching a managed
obstacle, or two different tracked non-static models touching each other,
latches failure. Contact with an ordinary static model such as
`ground_plane` is allowed. A full Gazebo simulation reset clears both latched
status and detail, defining a fresh physical-regression epoch without
restarting ROS.

No robot instance is hard-coded. By default every non-static model is tracked;
set `/xgc2_gazebo_scene/physical_contacts/tracked_model_prefixes` to a YAML
string list such as `[uav]` or `[ugv]` when a world contains unrelated dynamic
actors. Set `/xgc2_gazebo_scene/physical_contacts/enabled` to `false` to
disable the contact subscriber. The managed obstacle prefix is independently
configurable at `/xgc2_gazebo_scene/managed_obstacle_prefix` and defaults to
`xgc2_obstacle_`.

The first pair preserves the native scene-truth contract:

```text
/xgc2/simulation/obstacles/geometry
  xgc2_gazebo_scene/ObstacleDefinitionArray
  Header + scene_epoch + scene_revision + ObstacleDefinition[]
  ObstacleDefinition: name + model_name + generation + ConvexPart[]
  ConvexPart: part_id + shape + local_pose + primitive parameters
              + mesh URI/submesh/scale + conservative_vertices

/xgc2/simulation/obstacles/state
  xgc2_gazebo_scene/ObstacleStateArray
  Header + scene_epoch + scene_revision + ObstacleState[]
  ObstacleState: name + model_name + generation + pose + twist
                 + motion_mode + motion_revision
```

The second pair uses `xgc2_geometry_msgs/GeometryLibrary` and
`xgc2_geometry_msgs/ConvexBodyArray` directly. Primitive instances keep the
analytic `sphere`, `cylinder`, and `cube` geometry types. Convex meshes use one
stable `convex_mesh:<mesh-uri>[#submesh][#centered]` template with exact
unscaled local vertices, while every instance carries its world pose, collision
scale, static flag, and velocity. This keeps planner adapters independent from
the native Gazebo scene messages. A whole-mesh asset also publishes the
GeometryLibrary-compatible alias `v_polytope:<mesh-file-stem>` with the same
support points. Instances retain the canonical URI type; the alias lets
robot-body and other non-obstacle consumers use the semantic template name
without duplicating geometry. If two different URIs share one file stem, the
ambiguous alias is rejected rather than bound to either mesh.

The collision contract supports box, sphere, cylinder, and closed convex
triangle meshes. The reusable geometry loader accepts Gazebo triangle lists,
strips, and fans, resolves optional named/centered submeshes, and verifies that
every triangle belongs to one closed convex body. Unsupported, open, concave,
degenerate, or multi-body mesh collisions reject the entire managed obstacle
instead of publishing partial or approximated planning geometry.
