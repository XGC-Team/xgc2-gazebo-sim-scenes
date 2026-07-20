# tunnel_dynamic_1

隧道场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。

World: [`tunnel_dynamic_1.world`](tunnel_dynamic_1.world)

![tunnel_dynamic_1 scene preview](preview.png)

## Launch

```bash
roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/tunnel_dynamic_1/tunnel_dynamic_1.world" gui:=true paused:=true
```
