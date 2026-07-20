# square_exp

方形障碍物场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。

World: [`square_exp.world`](square_exp.world)

![square_exp scene preview](preview.png)

## Launch

```bash
roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/square_exp/square_exp.world" gui:=true paused:=true
```
