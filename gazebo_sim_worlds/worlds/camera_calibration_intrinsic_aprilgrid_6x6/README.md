# camera_calibration_intrinsic_aprilgrid_6x6

与现场实物一致的相机内参标定世界：Kalibr AprilGrid 6×6、
`tag36h11` ID 0–35、Tag 边长 88 mm、间隙 26.4 mm。
Tag 标定基准从首个 Tag 外边到末个 Tag 外边为 660×660 mm。

旧的 8×6 黑白棋盘格世界保留在
`camera_calibration_intrinsic/camera_calibration_intrinsic.world`，两种标定板
可以独立选择。

World:
[`camera_calibration_intrinsic_aprilgrid_6x6.world`](camera_calibration_intrinsic_aprilgrid_6x6.world)

## Launch

```bash
roslaunch gazebo_ros empty_world.launch \
  world_name:="$(rospack find gazebo_sim_worlds)/worlds/camera_calibration_intrinsic_aprilgrid_6x6/camera_calibration_intrinsic_aprilgrid_6x6.world" \
  gui:=true paused:=true
```
