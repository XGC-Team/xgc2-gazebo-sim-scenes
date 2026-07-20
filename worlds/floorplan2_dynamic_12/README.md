# floorplan2_dynamic_12

室内楼层平面场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。

World: [`floorplan2_dynamic_12.world`](floorplan2_dynamic_12.world)

![floorplan2_dynamic_12 scene preview](preview.png)

## Launch

```bash
roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floorplan2_dynamic_12/floorplan2_dynamic_12.world" gui:=true paused:=true
```
