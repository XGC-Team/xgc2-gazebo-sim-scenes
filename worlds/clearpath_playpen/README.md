# clearpath_playpen

Clearpath playpen 地面机器人场地，用于 Gazebo 场景加载和仿真验证。

World: [`clearpath_playpen.world`](clearpath_playpen.world)

![clearpath_playpen scene preview](preview.png)

## Launch

```bash
roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/clearpath_playpen/clearpath_playpen.world" gui:=true paused:=true
```
