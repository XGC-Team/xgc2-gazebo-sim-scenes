# floorplan3_box_dynamic_4

室内楼层平面场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。

World: [`floorplan3_box_dynamic_4.world`](floorplan3_box_dynamic_4.world)

![floorplan3_box_dynamic_4 scene preview](preview.png)

## Launch

```bash
roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floorplan3_box_dynamic_4/floorplan3_box_dynamic_4.world" gui:=true paused:=true
```
