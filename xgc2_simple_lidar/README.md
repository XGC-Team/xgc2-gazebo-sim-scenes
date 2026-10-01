# Simple lidar

An optional ideal sensor for Gazebo Classic 11. It publishes
`/<robot namespace>/simple_lidar/points` as `sensor_msgs/PointCloud2`: world-frame
XYZ metres, stamped at the observation time. GPU rays consume rendered
surfaces; CPU rays consume collision surfaces. Both preserve first-return occlusion. It does not simulate scan motion,
measurement noise, intensity, or a physical lidar/localization stack.

The shared `models/sensor.xacro` macro can be included inside a Gazebo link
(or a URDF `gazebo reference` element). `models/sensor.sdf.xacro` renders the
sensor element for SDF model generators. Its required arguments are `namespace`
and `pose` (`x y z roll pitch yaw`, relative to the parent link). Defaults are
10 Hz, 360 horizontal samples, 16 vertical samples over one radian, and 20 m
range, full circle, GPU; they are defined once, as properties in
`models/sensor.xacro` that the macro parameters and the `sensor.sdf.xacro`
arguments both read. `max_range`, `samples`, `layers`, `horizontal_fov`,
`vertical_fov`, `rate`, and `acceleration` can be set by the model description.

FS150, Scout and Wheeltec launch files expose `enable_simple_lidar` and
`simple_lidar_pose`. The sensor is omitted by default. Once installed and
enabled, it scans only while the point cloud has subscribers. Invalid or
out-of-range points are omitted: a range must lie strictly between the minimum
and maximum compared as floats, so a maximum without an exact float, such as
20.9 m, does not turn misses into a sphere of points. On the GPU path, the final horizontal column is omitted because
Gazebo 11 repeats a texture-edge ray there; the default complete frame contains
at most 5,744 points (68,928 XYZ bytes). CPU keeps all 360 columns, up to
5,760 points (69,120 bytes).

`test/run_standalone_test.sh` builds the projection test with only a C++17
compiler and the stand-in headers under `test/standalone` (their ignition
quaternion arithmetic repeats ignition-math 6 operation for operation); the
catkin build runs the same test against GoogleTest, ignition-math and
sensor_msgs. `test/run_standalone_benchmark.sh` times a frame against the
frozen 1.4.0-2 projection (`test/legacy_scan_projection.hpp`) and checks that
both produce the same bytes.

Use XGC ROS1 runtime 1.2.4 or later with the Gazebo GPU-laser material fix, and a
working hardware OpenGL display. Upstream Gazebo 11.15.1 shares its second-pass
material between sensors; multiple full-circle sensors can exhaust OGRE's
texture slots. The image builds the rendering library with an independent
material per sensor. No CUDA or PCL dependency is needed.

The live check (`--acceleration cpu|gpu`) runs its own ROS and Gazebo processes, tests a nonzero mount
transform while translating and rotating, opening occlusion, thin obstacle
position updates, deletion and respawn, and optionally reports one- and
multi-sensor load:

```bash
python3 test/gpu_lidar_check.py \
  --ros-port 11369 --gazebo-port 11389 \
  --plugin-dir /opt/ros/noetic/lib \
  --output-dir /tmp/simple-lidar-check --benchmark-sensors 8 --rebuild-cycles 5
```

Choose unused ports. Measurements apply to this small fixture and scan
configuration; they do not measure planner, vehicle physics, SITL or complex
world performance.

## CPU and GPU surface scans

`acceleration:=cpu` selects Gazebo's native collision ray sensor and the thin
CPU plugin; `acceleration:=gpu` keeps its native rendered-surface GPU ray path.
Both publish only world-frame XYZ and preserve measurement time. CPU updates the
native collision rays on the world update thread, then reads the pose and
measurement time before that world advances. Between scans each world update
costs only a flag and a sim-time comparison (`scan_schedule.hpp`): the
subscriber flag is kept by the connect/disconnect callbacks, and the parent
link is looked up only when a scan is due. Its independent background sensor
worker stays inactive; this avoids Gazebo Classic's sensor-container/physics
lock inversion during model removal. Only the ROS point cloud is consumed; the
internal Gazebo scan topic is not a supported input. Delayed ROS delivery cannot
substitute a later robot pose. Invalid acceleration names fail xacro expansion. CPU and GPU
use collision and visual geometry respectively; scene assets must keep those
representations consistent. Both sensors stop scanning when no ROS subscriber
needs the cloud. `rate`, `max_range`, `samples`, `layers`, `horizontal_fov` and
`vertical_fov` remain reusable sensor parameters.

GPU acceptance rejects llvmpipe/softpipe and records the actual GL renderer.
With NVIDIA containers, enable the graphics/display driver capabilities and
NVIDIA GLX vendor; a working X display alone is insufficient. The Experiment
exposes the observation/acceleration/scan settings, not an independent mount
pose. Model-owned internal offsets remain responsible for chassis clearance.
