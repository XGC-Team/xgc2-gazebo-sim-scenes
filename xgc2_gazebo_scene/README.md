# XGC2 Gazebo Scene

## World host

`libxgc2_simulation_world.so` is the native `xgc2.simulation` v1 service of one world. It serves simulation-v1
on an `http.v1` Unix socket, the chassis HOLD capability of the world (`xgc2.chassis.hold`, built on the
`libxgc2-chassis-hold1` domain; see [docs/chassis-hold.md](docs/chassis-hold.md)) and the parts of the service
that exist only in Gazebo: the scene extension and the native health notes
([docs/scene-extension.md](docs/scene-extension.md)). Its parameters are `socket_path`, `target_id` and
`resource_root` (required), `configuration_revision`, and one `required_component` per native component whose
readiness gates the world's health. HOLD has no parameters: chassis model plugins join and leave its roster by
themselves through `xgc2_gazebo_scene/chassis_hold.hpp`. It builds against `libxgc2-xrpc-dev` and
`libxgc2-chassis-hold-dev` 0.2 or later.

The editable scene entry point is `libxgc2_scene_authoring_world.so`, loaded by
`gazebo_sim_worlds/worlds/scene_editable/scene_editable.world`. It is an adapter
for the independent scene runtime, not an algorithm launcher. Select the world
in Gazebo Server, then load the YAML through the separate user scene workflow.

## Editable scene adapter

- `/xgc/scene/gazebo/apply`: `xgc2_geometry_msgs/ApplyScene`, full typed snapshot.
- `/xgc/scene/state`: `xgc2_geometry_msgs/SceneState`, computed current poses from
  the scene runtime. This plugin has no separate motion clock or YAML loader.

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
Every obstacle's SDF is parsed twice per application (validation, then restoring
its parameters under the physics update mutex). `ModelSdfParser`
(`model_sdf_parser.hpp`) reads the SDF specification once, which costs about 25 ms
per `sdf::init`, and the parameters are restored after parsing, not during it.
`test/run_sdf_parser_benchmark.sh` measures it with libsdformat.
Applying and State resolve scene models through an index of the
world's model list (`model_index.hpp`) that is rebuilt only when the list changes
(`World::ModelByName` walks every entity and holds the model-loading mutex while
it does), refreshed before every step and every 10 ms wait pass. A held obstacle
whose requested pose is unchanged, and which is still exactly where the adapter
last put it, is not set again.
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

The contact table and the managed obstacle set are derived from the world's
model list. Each world update compares that list (models, names, static
flags) with the previous one (`model_snapshot.hpp`) and rebuilds both only
when it changed, or while a managed model is still rejected; an unchanged
world costs one comparison per model instead of a fresh map per update.
`test/run_model_snapshot_benchmark.sh` measures both without Gazebo.

The 30 Hz instance message keeps instance ids and names, geometry types, scales
and local collision poses in `ObstacleMessages` (`obstacle_messages.hpp`), which
is rebuilt when the set of obstacles changes. A publication writes only world
poses, velocities and static flags, so an unchanged set allocates nothing.
`test/obstacle_messages_test.cpp` checks those values, membership changes and
cache reuse without Gazebo. Geometry-library templates are published when
the obstacle set changes.

The planning topics use `xgc2_geometry_msgs/GeometryLibrary` and
`xgc2_geometry_msgs/ConvexBodyArray` directly. Primitive instances keep the
analytic `sphere`, `cylinder`, and `cube` geometry types. Convex meshes use one
stable `convex_mesh:<mesh-uri>[#submesh][#centered]` template with exact
unscaled local vertices, while every instance carries its world pose, collision
scale, static flag, and velocity. A whole-mesh asset also publishes the
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

## Robot spawn helper

`spawn_robot_model` replaces `gazebo_ros spawn_model` in robot launch files
(`-urdf|-sdf -param P -model M -x ... -Y ...`, plus `-hold`). It holds the
host-wide `/tmp/xgc2-gazebo-spawn.lock`, which Core's FS150 spawn Job also
takes, only across the insert and its check in the world model list, so robots
start their interpreters and read their models in parallel. It waits for
Gazebo with bounded timeouts: exit 5 when no Gazebo is registered on the ROS
master, 6 when the master does not answer, 1 for other failures. A model that
already exists is reused. With `-hold` a successful run becomes `sleep`, so the
launch can mark the node `required="true"` and end as soon as a spawn fails.
