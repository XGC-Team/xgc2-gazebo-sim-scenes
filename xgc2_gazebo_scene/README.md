# XGC2 Gazebo Scene

The editable scene entry point is `libxgc2_scene_authoring_world.so`, loaded by
`gazebo_sim_worlds/worlds/scene_editable/scene_editable.world`. It is an adapter
for the independent scene runtime, not an algorithm launcher. Select the world
in Gazebo Server, then load the YAML through the separate user scene workflow.

## Editable scene adapter

- `/xgc/scene/gazebo/apply`: `xgc2_geometry_msgs/ApplyScene`, full typed snapshot.
- `/xgc/scene/state`: `xgc2_geometry_msgs/SceneState`, computed current poses from
  the scene runtime. This plugin has no separate motion clock or YAML loader.
- `/xgc/scene/consumer_status`: `SceneConsumerStatus`, successful applied epoch
  and revision or an explicit error, with a wall-clock heartbeat.

The world plugin's `scene_namespace` parameter changes the common namespace.
`apply_timeout` is a wall-clock timeout, default 5 seconds, maximum 60 seconds.
Applications also work while Gazebo is paused; the adapter does not unpause the
world or advance simulation time.

Geometry is in metres and the snapshot frame must be `world`. Boxes preserve
full side lengths, spheres preserve radius, cylinders preserve radius and full
height. A capsule is an exact union of its cylinder and two end spheres; its
height is the straight section only. Convex meshes preserve the supplied vertex
coordinates and triangle topology. The existing convex validator rejects open,
degenerate and concave meshes. Multi-part objects stay multi-part, so a gate's
opening remains open. The temporary OBJ files are adapter-owned transport
artifacts; they are never a second authoring source or a file path accepted from
the viewer. Gazebo's mesh loader uses float coordinates internally.
After allocation, original double parameters and cached collision poses are
restored under the physics update mutex: Gazebo's SDF clone/set paths otherwise
round authoring values. A zero-height capsule is exactly one sphere.

IDs become injectively encoded model/part names. An unchanged shape retains its
Gazebo model across other edits. Resize/regeometry replaces only that obstacle's
model with the same stable name. Clear and replace only remove models actually
created by this plugin. Prefix-matching foreign models, robots and cameras are
never adopted or deleted. The name prefix remains `xgc2_obstacle_` so an optional
legacy physical-contact observer can identify these obstacles, but that observer
cannot configure or stop their motion. A global stop skips scene-runtime models.
Retired entities receive a unique internal name before destruction, preventing
Gazebo's delayed delete requests from deleting their same-name replacements.

Success is returned after the actual Gazebo collision objects have the requested
types, dimensions, local poses and mesh identity, and their visual definitions
exist. Stale revisions, retired epochs and different contents at an already
applied revision fail explicitly. An identical retry preserves current moving
poses; a missing or corrupted model can be repaired with the same full snapshot.
State from another epoch/revision, incomplete sets and malformed poses cannot move
models. Mesh compilation/validation happens before any world mutation.
Unchanged initial definitions retain their current running poses across edits to
other obstacles, including when a scene is playing.

World changes are not an atomic transaction. If Gazebo fails after replacement
has begun, the adapter reports failure and retains the previous applied revision;
the world can be partially modified. Motion is suspended until a full snapshot
repairs it. The scene runtime must retain its unsynchronized state and retry or
apply a compensating full snapshot. Stopping the scene runtime does not clear the
last physical models. Saving belongs to the scene runtime, not this adapter.
Corrective snapshots first drain outstanding factory insertions; an older failed
request cannot insert a late obstacle after a successful correction or Clear.

`scene_authoring.test` tests real Gazebo physics objects, paused edits, all shape
classes, a ray through the gate opening, stable identity, live membership/size
updates, stale/malformed messages and foreign-model protection. It uses its own
Gazebo master and a private rostest ROS master.

`test/scene_runtime_gazebo_integration.py` additionally starts private ROS/Gazebo
masters and the independent scene runtime. Pass `--scene-file` pointing to the
algorithm-owned original `uav6_knot/scene.yaml` and `--evidence-dir` pointing to a
new test output directory. It compares all 16 actual collision definitions using
the existing collision observer, tests live edits, persistence/reload, motion,
late subscribers, simulator failure, exact-command retry and accepted-document
resynchronization. It starts no algorithm. With separately built catkin workspaces,
source the scene product `devel/setup.bash`, then the common runtime
`devel/setup.bash --extend`; prepend the scene product's `devel/lib` to
`GAZEBO_PLUGIN_PATH`. Test input is copied and all subprocesses are cleaned up.

## Legacy world observation and motion

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
