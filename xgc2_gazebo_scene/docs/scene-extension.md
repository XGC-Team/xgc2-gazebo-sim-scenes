# Gazebo scene extension and native components

The engine-neutral simulation service contract (`xgc2.simulation` v1: entities, sensors, world control and
operations) does not describe the parts that exist only in the Gazebo world host. They are specified here. The
extension routes live on the same `http.v1` Unix server as the common routes, use the same XRPC headers
(`X-Xrpc-Instance-ID`, `X-Xrpc-Timeout-Ms`, `X-Request-ID`), the same operation model for mutations (202 with an
operation that is observed with `/v1/operations/{id}/wait`) and the same closed request schemas. The host
advertises them in `/v1/describe` under `extensions`.

## Scene extension (`/v1/extensions/scene/*`)

The Gazebo scene extension owns its authored obstacle roster. Generic entity remove/state/reset rejects those
scene-owned entities before effects; clients use the declared scene domain.

| Route | Meaning |
| --- | --- |
| `GET /v1/extensions/scene` | Snapshot: immutable definition, actual native state, authoring epoch, revision and serial, and `simulation_time:{epoch,nanoseconds}` |
| `POST /v1/extensions/scene/observe` | `{after_serial}`: holds a bounded request (at most 16 observers) until the native scene changes, or the caller's deadline ends |
| `POST /v1/extensions/scene/apply` | `{epoch, revision, document, operation_timeout_ms}`: applies the `xgc2.scene.v1` document (`id`, `frame`, `obstacles` with `parts` and `motion`) as a new revision; an unchanged obstacle keeps its entity reference |
| `POST /v1/extensions/scene/motion` | `{epoch, revision, operation, operation_timeout_ms}` with `operation` one of `play`, `pause`, `reset`, for the applied revision |
| `POST /v1/extensions/entities/motion` | `{entities:[{ref:{id,generation}, motion}], operation_timeout_ms}`: motion of independently owned obstacle entities |

`GET /v1/extensions/scene` reports the authoring epoch/revision/serial and
`simulation_time:{epoch,nanoseconds}`. A consumer that fences scene state across a world rewind also checks the
simulation time epoch: an unchanged authoring revision does not make old physical state fresh.

## Native health and required components

`GET /v1/health` is ready only after world initialization and every explicitly required component's owned
initialization succeeds. A component is required by a `required_component` parameter of the world plugin; its
owner attaches with `NativeComponentBinding` and reports `Ready()` or `Failed()` from its own native
initialization. VRPN readiness includes its actual listening socket initialization when `vrpn` is required.
Clock samples, ROS graph membership, TCP probes and a discovered endpoint are not readiness authorities.

Optional native extensions aggregate into the world's existing SDK listener and management executor
(`WorldExtensionBinding`, `NativeWorldExtension`); their declarations state actual consumers, schemas and
completion, without requiring every library to expose RPC.
