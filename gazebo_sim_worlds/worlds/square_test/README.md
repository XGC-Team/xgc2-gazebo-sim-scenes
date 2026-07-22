# square_test

方形障碍物场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。

World: [`square_test.world`](square_test.world)

![square_test scene preview](preview.png)

## Launch

```bash
roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/square_test/square_test.world" gui:=true paused:=true
```
