# test2

测试场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。

World: [`test2.world`](test2.world)

![test2 scene preview](preview.png)

## Launch

```bash
roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/test2/test2.world" gui:=true paused:=true
```
