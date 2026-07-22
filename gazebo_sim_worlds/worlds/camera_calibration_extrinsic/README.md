# camera_calibration_extrinsic

相机外参标定场景，在三维空间布置六个彩色标记，用于验证相机位姿与世界坐标系之间的外参。

World: [`camera_calibration_extrinsic.world`](camera_calibration_extrinsic.world)

## Launch

```bash
roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/camera_calibration_extrinsic/camera_calibration_extrinsic.world" gui:=true paused:=true
```
