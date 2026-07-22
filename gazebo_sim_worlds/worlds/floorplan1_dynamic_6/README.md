# floorplan1_dynamic_6

室内楼层平面场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。

World: [`floorplan1_dynamic_6.world`](floorplan1_dynamic_6.world)

![floorplan1_dynamic_6 scene preview](preview.png)

## Launch

```bash
roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floorplan1_dynamic_6/floorplan1_dynamic_6.world" gui:=true paused:=true
```
