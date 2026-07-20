# floorplan1_dynamic_16

室内楼层平面场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。

World: [`floorplan1_dynamic_16.world`](floorplan1_dynamic_16.world)

![floorplan1_dynamic_16 scene preview](preview.png)

## Launch

```bash
roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floorplan1_dynamic_16/floorplan1_dynamic_16.world" gui:=true paused:=true
```
