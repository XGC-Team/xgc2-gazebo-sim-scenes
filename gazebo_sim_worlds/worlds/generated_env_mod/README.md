# generated_env_mod

程序生成环境场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。

World: [`generated_env_mod.world`](generated_env_mod.world)

![generated_env_mod scene preview](preview.png)

## Launch

```bash
roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/generated_env_mod/generated_env_mod.world" gui:=true paused:=true
```
