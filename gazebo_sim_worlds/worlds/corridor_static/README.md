# corridor_static

结构化窄缝压力场景（structured-narrow stress）。保留原始几何，
用于验证“通过可行窄缝或明确安全停车”，不作为基础功能调参场景。

World: [`corridor_static.world`](corridor_static.world)

![corridor_static scene preview](preview.png)

## Launch

```bash
roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/corridor_static/corridor_static.world" gui:=true paused:=true
```
