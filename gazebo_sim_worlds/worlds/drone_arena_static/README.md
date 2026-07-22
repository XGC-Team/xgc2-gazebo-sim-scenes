# drone_arena_static

无人机测试场地，静态布景。

World: [`drone_arena_static.world`](drone_arena_static.world)

![drone_arena_static scene preview](preview.png)

## Launch

```bash
roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/drone_arena_static/drone_arena_static.world" gui:=true paused:=true
```
