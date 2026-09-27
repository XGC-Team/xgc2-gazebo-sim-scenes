# Simple GPU lidar

An optional ideal sensor for Gazebo Classic 11. It publishes
`/<robot namespace>/simple_lidar/points` as `sensor_msgs/PointCloud2`: world-frame
XYZ metres, stamped at the rendered observation time. It uses Gazebo GPU rays
and visible surfaces, including occlusion. It does not simulate scan motion,
measurement noise, intensity, or a physical lidar/localization stack.

The shared `models/sensor.xacro` macro can be included inside a Gazebo link
(or a URDF `gazebo reference` element). `models/sensor.sdf.xacro` renders the
sensor element for SDF model generators. Its required arguments are `namespace`
and `pose` (`x y z roll pitch yaw`, relative to the parent link). Defaults are
10 Hz, 360 horizontal samples, 16 vertical samples over one radian, and 20 m
range. `max_range`, `samples`, `layers`, `vertical_fov`, and `rate` can be set
by the model description.

FS150, Scout and Wheeltec launch files expose `enable_simple_lidar` and
`simple_lidar_pose`. The sensor is omitted by default. Once installed and
enabled, it renders only while the point cloud has subscribers. Invalid or
out-of-range points are omitted. The final horizontal column is omitted because
Gazebo 11 repeats a texture-edge ray there; the default complete frame contains
at most 5,744 points (68,928 XYZ bytes).

Use XGC ROS1 runtime 1.2.4 or later with the Gazebo GPU-laser material fix, and a
working hardware OpenGL display. Upstream Gazebo 11.15.1 shares its second-pass
material between sensors; multiple full-circle sensors can exhaust OGRE's
texture slots. The image builds the rendering library with an independent
material per sensor. No CUDA or PCL dependency is needed.

The live check runs its own ROS and Gazebo processes, tests a nonzero mount
transform while translating and rotating, opening occlusion, thin obstacle
position updates, deletion and respawn, and optionally reports one- and
multi-sensor load:

```bash
python3 test/gpu_lidar_check.py \
  --ros-port 11369 --gazebo-port 11389 \
  --plugin-dir /opt/ros/noetic/lib \
  --output-dir /tmp/simple-lidar-check --benchmark-sensors 8
```

Choose unused ports. Measurements apply to this small fixture and scan
configuration; they do not measure planner, vehicle physics, SITL or complex
world performance.
