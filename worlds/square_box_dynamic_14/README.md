# square_box_dynamic_14

方形障碍物场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。

World: [`square_box_dynamic_14.world`](square_box_dynamic_14.world)

![square_box_dynamic_14 scene preview](preview.png)

## Launch

```bash
roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/square_box_dynamic_14/square_box_dynamic_14.world" gui:=true paused:=true
```
