# camera_calibration_intrinsic

相机内参标定场景，提供 8×6 棋盘格目标和稳定光照，用于采集 Gazebo 相机标定图像。

World: [`camera_calibration_intrinsic.world`](camera_calibration_intrinsic.world)

## Launch

```bash
roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/camera_calibration_intrinsic/camera_calibration_intrinsic.world" gui:=true paused:=true
```
