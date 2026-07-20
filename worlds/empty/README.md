# empty

基础空场景，用于基础加载和坐标/光照检查。

World: [`empty.world`](empty.world)

![empty scene preview](preview.png)

## Launch

```bash
roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/empty/empty.world" gui:=true paused:=true
```
