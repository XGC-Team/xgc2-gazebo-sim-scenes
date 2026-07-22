# floor1_empty

室内楼层平面场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。

World: [`floor1_empty.world`](floor1_empty.world)

![floor1_empty scene preview](preview.png)

## Launch

```bash
roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floor1_empty/floor1_empty.world" gui:=true paused:=true
```
