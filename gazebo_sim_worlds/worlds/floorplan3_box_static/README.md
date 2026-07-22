# floorplan3_box_static

室内楼层平面场景，静态布景，包含箱体障碍物。

World: [`floorplan3_box_static.world`](floorplan3_box_static.world)

![floorplan3_box_static scene preview](preview.png)

## Launch

```bash
roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floorplan3_box_static/floorplan3_box_static.world" gui:=true paused:=true
```
