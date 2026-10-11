# Chassis HOLD in the Gazebo world

HOLD stops one robot's chassis from executing motion commands until it is explicitly released. It is not a
world pause, an entity disable or a reset: the other robots, the simulation clock and the sensors keep running.
The domain, the JSON bodies and their semantics are the chassis HOLD contract of `xgc2-chassis-hold`
(`libxgc2-chassis-hold-dev`, `contracts/chassis-hold.md`); this page is only its Gazebo binding.

## Host

The world plugin `libxgc2_simulation_world.so` owns one HOLD host per world: one `xgc2::chassis_hold::Domain` and
one `Service` and the library's `HttpAdapter`. HOLD is a capability of the world host, `xgc2.chassis.hold`, and
is called like any XRPC method (the generic method addressing of `http.v1`) on the world's Unix server, the one
that serves simulation-v1:

    POST /v1/call/xgc2.chassis.hold/{Describe,State,Engage,Release}

The request is the JSON body of the contract (an empty body is `{}`) and the reply is its JSON reply, unchanged.
Status mapping: `ok` 200, `invalid_argument` 400, `not_found` 404 (an unknown method, or a `State` that names an
unknown robot), `internal` 500; an unknown method or service under `/v1/call/` is 404. A verb other than POST
on a HOLD route is 405 with `Allow: POST`. Errors use the XRPC `error` envelope. Engage replies once the
native tick has written zero (at most 60 ms, never beyond the call deadline) without blocking the server's IO
thread. Like every business call these carry `X-Xrpc-Instance-ID`; the `instance` of the HOLD bodies is the same
instance ID, so a caller that kept a plan for a previous world host is refused by the transport (409) or, if it
re-resolved the host but kept an old `expected_instance`, by the domain (`conflict` for every robot). A
restarted world host is a new instance and starts with nothing held; HOLD is not persisted.

The capability is listed in the facts of `GET /v1/describe`, with the entities that are bound to it now, so a
caller can resolve "entity X, capability `xgc2.chassis.hold`" to this host:

    {"facts": {"capabilities": [{"name": "xgc2.chassis.hold", "entities": ["scout-1", "ugv-2"]}]}}

There is nothing else to configure: no `/v1/chassis/*` routes, no `chassis_robot_id` world parameters and no
roster from Core.

## Roster

The roster is dynamic. A chassis model plugin reserves its entity ID with `xgc2_gazebo_scene::ChassisHold`
(`chassis_hold.hpp`) when it is constructed and joins the roster with `Ready()` once its native setup is complete.
It leaves when the plugin is destroyed or when the entity is removed through simulation-v1, whichever comes first.
The ID is the `chassisRobotId` element of the plugin, which `POST /v1/entities` requires to equal the public
entity ID; an ID is bound to one model at a time and a duplicate fails the plugin's load. A create whose chassis
plugin did not bind fails with `unavailable`.

The HOLD state of an ID outlives its model: a model that is deleted and created again under the same ID starts
held (same revision, stage `gated` until the next tick has written zero to the new model). At most 1024 distinct
IDs can be used by one world host.

## Command admission

Chassis models take `cmd_vel` from ROS. Their subscriptions use the callback queue of the HOLD host
(`ChassisHold::CommandQueue()`), a single dispatcher thread per world that stamps every callback with the
monotonic time at which roscpp queued the message. A command is applied only if `Domain::admit(id, receipt)`
accepts it: the robot is in the roster, not held, and the command was received after its last release. A
command that was queued before a release is dropped when it is finally dispatched, however late that is.
The stamp is not simulation time, which stops while the world is paused.

## Native tick and zero output

`Domain::tick` writes zero to every held robot on every tick. The domain's state machines always tick on one
thread of the host, every 10 ms under the physics update lock,
whether the world runs or is paused. `DomainOptions::wake` wakes that thread whenever the domain needs a tick,
including engage and release. HOLD takes effect and is re-asserted while simulation time is frozen. Every
model's control step also observes the admission gate and uses zero commands while held.

Each model supplies its zero writer to its `ChassisHold`. The host calls it with the physics update lock and the
seat lock of the model held; the model's `cmd_vel` callback and control step run under the same seat lock
(`Command()` and `Control()`), so they never overlap the zero writer and a command that slipped in just before an
engage is cleared by the first tick.

| Plugin | Zero writer | Control step while held |
| --- | --- | --- |
| Scout skid steer (`libscout_gazebo.so`) | clears the delay line, wheel targets and I/P state; zero effort on the four wheel joints | the same zero (coasts) |
| Scout unicycle (`libscout_unicycle_plugin.so`) | resets the drive; zero velocity on every link; publishes zero velocity | the drive state is held at zero (rigid stop) |
| Scout implicit wheel (`libscout_implicit_wheel_plugin.so`) | clears the delayed command and drive-error memory; zero effort on the four joints | runs the wheel model with a zero command (the motors brake) |
| Mecanum (`libgazebo_sim_mecanum_contract.so`) | zero commands, zero joint efforts; ideal mode also zero model velocity | high fidelity runs its wheel loops with zero commands (brakes); ideal holds zero velocity |

Zero output is not a physical stop: inertia, contact and slope are separate facts.

## Feedback and `stopped`

Every chassis model reports the horizontal speed and the yaw rate of its model (`BodySpeed`), read at most every
5 ms after a tick that wrote its zero; the domain turns rest for 300 ms (below 0.02 m/s and 0.05 rad/s) into the
stage `stopped`. The
vertical velocity is not used: contact solvers leave a residual vertical velocity on a resting vehicle. A world
that is paused keeps the velocities of its last step, so a robot that was moving when the world was paused is
`zero_written` and becomes `stopped` only after the world runs again; models whose zero writer sets velocities
directly (the unicycle and the ideal Mecanum) are `stopped` at once.

## Entity operations

* Removing an entity ends its seat (a final zero first) before the model is removed, because Gazebo finalizes
  the joints and links of a model before it destroys its plugins. A plugin that is destroyed without this step
  only leaves the roster.
* Resetting an entity or the world writes zero to the models first.
* Pausing and resuming the world is independent of HOLD.

## Writing a chassis plugin

```cpp
hold_.reset(new ChassisHold(world, robot_id, {[this] { WriteZero(); }, BodySpeed(model_.get())}));
node.setCallbackQueue(hold_->CommandQueue());
command_ = node.subscribe("cmd_vel", 1, &Plugin::OnCommand, this);        // OnCommand: hold_->Command([&] { ... })
update_ = gazebo::event::Events::ConnectWorldUpdateBegin(...);            // the callback: hold_->Control([&](bool held) { ... })
hold_->Ready();
// ~Plugin: update_.reset(); command_.shutdown(); hold_.reset();
```

## Verification

`scout/test/native/verify.py` of `gazebo-sim-scout` runs a real headless `gzserver` with the world plugin, the
three Scout drive plugins and both Mecanum drive models, drives all of them with ROS `cmd_vel` and checks
the method calls and the describe facts, engage, `zero_written`, `stopped`, unaffected robots, release by
revision, conflicts, commands queued before a release (a test plugin blocks the dispatcher), HOLD while the
world is paused, deleting and creating held models, and a restart of the world host.
`test/native_simulation/verify.py` of this package covers the world host with a fixture plugin.
