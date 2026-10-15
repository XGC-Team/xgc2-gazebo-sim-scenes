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

The native world starts in parallel with ROS and its consumers. ROS data, physical
contacts and simple lidar register through one shared master watcher outside
the Gazebo initialization and update threads. Missing masters use exponential
backoff with full jitter from 250 ms to a 30 s cap, with a single identity read in
flight and at most one failure summary per 10 s. A different bound master PID or
run ID ends the process through SIGINT. The world's `ready` remains a statement
about its native resources; `facts.ros_master` reports the ROS binding separately.
Topic consumers are discovered through ROS updates. Data queues keep one current
sample, and a new clock subscriber receives the current time even while paused.
Robot creation needs the bound world service and its ROS asset parameter; it does
not wait for a scene adapter or controller.

Python consumers use `xgc2_gazebo_scene.simulation_client.SimulationClient` with
the formal `xgc2_xrpc` 0.2.0 wheel and an explicit simulation-v1 ServiceRef.
The client submits once and waits for the world's terminal operation result.

The start-order fixtures need ROS but do not run Gazebo. From the repository root:

```bash
. /home/node/workspace/underlay/setup.sh
nice -n 19 cmake -S xgc2_gazebo_scene/test/start_order -B build/start_order
nice -n 19 cmake --build build/start_order -j2
cd build/start_order
nice -n 19 ctest -LE long --output-on-failure
nice -n 19 ctest -L long -V
```

The `long` cell holds the master and a topic consumer down for separate 60 s
windows and checks CPU, connect attempts, logs, threads and descriptors. It then
checks registration, the latest desired revision and consumer restart. Every
fixture uses a private master port in 20000-40000 and its own ROS home.

Gazebo world preparation is provided by `xgc2-scene-prepare` from the
`xgc_scene_ros1_adapter` package (`ros-noetic-xgc2-scene-adapter` >= 0.7.0-1).
It materializes `gazebo_sim_worlds/worlds/scene_editable/scene_editable.world`
with the native world host's socket, target and configuration parameters.
The independent C++ `xgc2_scene_adapter` owns scene authoring and persistence.

## Editable scene adapter

- `/v1/extensions/scene/apply`: full scene document through the bound world service.
- `/xgc/scene/state`: `xgc2_geometry_msgs/SceneState`, computed current poses from
  the scene adapter. The native world host owns simulator application and motion.

The scene adapter uses `__ns` for its ROS namespace, `_profile:=gazebo` and an
explicit `_simulation_service_ref_json`. Applications also work while Gazebo
is paused; applying a scene does not advance simulation time.

Geometry is in metres and the snapshot frame must be `world`. Boxes preserve
full side lengths, spheres preserve radius, cylinders preserve radius and full
height. A capsule is an exact union of its cylinder and two end spheres; its
height is the straight section only. Convex meshes preserve the supplied vertex
coordinates and triangle topology. The existing convex validator rejects open,
degenerate and concave meshes. Multi-part objects stay multi-part, so a gate's
opening remains open. The temporary OBJ files are world-owned transport
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
cannot configure or stop their motion. A global stop skips adapter-authored models.
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
repairs it. The scene adapter must retain its unsynchronized state and retry or
apply a compensating full snapshot. Stopping the scene adapter does not clear the
last physical models. Saving belongs to the scene adapter.
Corrective snapshots first drain outstanding factory insertions; an older failed
request cannot insert a late obstacle after a successful correction or Clear.

`scene_authoring.test` tests real Gazebo physics objects, paused edits, all shape
classes, a ray through the gate opening, stable identity, live membership/size
updates, stale/malformed messages and foreign-model protection. It uses its own
Gazebo master and a private rostest ROS master.

`test/scene_adapter_gazebo_integration.py` additionally starts private ROS/Gazebo
masters and the C++ scene adapter. Pass `--scene-file` pointing to the
algorithm-owned original `uav6_knot/scene.yaml` and `--evidence-dir` pointing to a
new test output directory. It compares all 16 actual collision definitions using
the existing collision observer, tests live edits, persistence/reload, motion,
late subscribers, simulator failure, exact-command retry and accepted-document
resynchronization. Simulator fault tests suspend and resume the same process to
preserve its bound ServiceRef instance. It starts no algorithm. Source the Gazebo
scenes and ROS1 adapter install overlays; prepend the Gazebo scenes library
directory to `GAZEBO_PLUGIN_PATH`. The adapter executable must be discoverable
in `xgc_scene_ros1_adapter`. Test input is copied and all subprocesses are cleaned up.

## Physical contact observation

The SystemPlugin loaded with `gzserver` observes raw physical contacts. Scene definitions and dynamic state use the scene adapter's `SceneSnapshot` and `SceneState`; geometry is transported as a numerical type and binary parameters. Native Gazebo CPU/GPU sensors measure the simulator's physical geometry.

Physical collision status is published on:

- `/xgc2/simulation/physical_collision` (`std_msgs/Bool`, latched)
- `/xgc2/simulation/physical_collision_detail` (`std_msgs/String`, latched on the first forbidden contact)

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

The contact table is derived from the world's model list. Each world update compares models, names and static flags with the previous list (`model_snapshot.hpp`) and rebuilds the table only when they change.

## Robot placement CLI

`/opt/ros/noetic/lib/xgc2_gazebo_scene/spawn_robot_model` places one supplied
robot model and exits. Core runs it after awaiting the world, with a 60-second
command bound. It accepts UTF-8 XML from a file or stdin without a ROS
environment, a ROS master or `HOME`. `--help` works without the placement module
or XRPC SDK being importable.

| Argument | Required / default | Validation |
| --- | --- | --- |
| `--world-endpoint PATH` | Required | Canonical absolute Unix socket path, at most 107 filesystem bytes |
| `--target-id ID` | Required | Nonempty canonical target ID |
| `--model-name NAME` | Required | Entity ID: 1–256 ASCII letters, digits, `_`, `.`, `:`, `-` |
| `--model-file PATH` | One model source required | UTF-8, well-formed XML, at most 8 MiB |
| `--model-stdin` | Mutually exclusive with `--model-file` | Raw UTF-8 XML, at most 8 MiB; input must finish within the deadline |
| `--model-format urdf\|sdf` | Required | Selects `application/urdf+xml` or `application/sdf+xml` |
| `--robot-namespace /ns` | `/` | Absolute ROS namespace, at most 128 characters |
| `--x X` | `0` | Finite position in metres |
| `--y Y` | `0` | Finite position in metres |
| `--z Z` | `0` | Finite position in metres |
| `--roll R` | `0` | Finite angle in radians |
| `--pitch P` | `0` | Finite angle in radians |
| `--yaw YAW` | `0` | Finite angle in radians |
| `--timeout S` | `50` | Finite seconds in `(0, 55]`, shared by input, discovery and creation |
| `--help` | Optional | Show arguments and exit 0 without contacting a world |

```bash
/opt/ros/noetic/lib/xgc2_gazebo_scene/spawn_robot_model \
  --world-endpoint /run/xgc2/session/world.sock --target-id local \
  --model-name ugv1 --model-file /path/to/robot.urdf --model-format urdf \
  --robot-namespace /ugv1 --x 1 --y 2 --z 0.181 --yaw 0.5 --timeout 50
```

The entity has role `robot`, the model text in its asset realization, the
position and RPY quaternion, and `parameters.ros_namespace`. Success prints
exactly one JSON line:

```json
{"schema":"xgc2.placement.v1","kind":"robot","id":"ugv1","entity":{"entity":{"id":"ugv1","generation":7}},"instance":"world-instance"}
```

The `entity` field preserves the native operation result object. Exit codes are
0 for success, 2 for usage or model/argument validation errors before world
contact, and 1 for discovery or creation failures. Failures print one stderr
line, `spawn_robot_model: <reason>`. A repeated entity ID fails with the world's
conflict error; placement never replaces or replays a creation.

## Shared Python placement API

The installed module is
`/opt/ros/noetic/lib/python3/dist-packages/xgc2_gazebo_scene/placement.py`.
`discover_world(endpoint, target_id, timeout)` returns a bound `SimulationClient`.
It waits for socket appearance through inotify, performs one unbound
`GET /v1/describe?wait_ready_ms=<remaining budget, capped at 30000>`, and requires
readiness inside that call. It validates the simulation service, API version,
HTTP profile, Unix endpoint, target and instance identity. The caller must close
the returned client; closing it also closes its owned runtime.

`place_entity(endpoint, target_id, entity, timeout)` returns
`(result_dict, instance_id)`. Discovery, one `POST /v1/entities` and at most one
notification-driven operation wait share the same deadline. It always closes
the bound client and runtime, including on failure. Both functions accept a
finite timeout in `(0, 55]` seconds. Other callers can continue using
`SimulationClient` with their own explicitly owned runtime.

The CLI's 8 MiB validation bound does not increase the native world's current
64 KiB artifact and 128 KiB request limits; the world can refuse a larger model.
