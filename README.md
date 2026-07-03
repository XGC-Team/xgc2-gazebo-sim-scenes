# XGC2 Gazebo Sim Worlds

Shared Gazebo Classic world assets for XGC2 simulation products.

This repository owns only reusable Gazebo Classic world and model resources. Complete simulation scenarios, vehicle launch orchestration, controller startup, VRPN routing, and estimator startup stay in `gazebo_sim_examples` and other product-specific packages.

## Packages

- `gazebo_sim_worlds`

## APT

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-gazebo-sim-worlds
```

## Installed Assets

- `/opt/ros/noetic/share/gazebo_sim_worlds/worlds/empty/empty.world`
- `/opt/ros/noetic/lib/libobstaclePathPlugin.so`
- `/opt/ros/noetic/share/gazebo_sim_worlds/worlds/clearpath_playpen/clearpath_playpen.world`
- `/opt/ros/noetic/share/gazebo_sim_worlds/worlds/corridor_dynamic_9/corridor_dynamic_9.world`
- `/opt/ros/noetic/share/gazebo_sim_worlds/models/corridor/model.sdf`
- `/opt/ros/noetic/share/gazebo_sim_worlds/models/jersey_barrier/model.sdf`

## CI Checks

The package compliance check validates every world XML file, every shared model metadata file, same-name scene layout, all `model://` references against the packaged `models/` directory, and Gazebo SDF validity through `gz sdf -k`.

<!-- WORLD_CATALOG_START -->

## World Preview Catalog

The preview images are generated from the ROS1 Docker runtime with Gazebo GUI on the middle display. Each entry lists the scene content, the direct launch command for the installed world asset, and the captured preview image.

Dynamic worlds use `libobstaclePathPlugin.so`, which is built and installed by this package. No extra UAV controller, planner, or simulator algorithm package is required for obstacle motion.

### `worlds/bridge_static/bridge_static.world`

- 场景内容：桥梁场景，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/bridge_static/bridge_static.world" gui:=true paused:=true
  ```

- 截图：

  ![bridge_static](pics/bridge_static.png)

### `worlds/building_2f_static/building_2f_static.world`

- 场景内容：建筑物场景，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/building_2f_static/building_2f_static.world" gui:=true paused:=true
  ```

- 截图：

  ![building_2f_static](pics/building_2f_static.png)

### `worlds/building_4f_static/building_4f_static.world`

- 场景内容：建筑物场景，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/building_4f_static/building_4f_static.world" gui:=true paused:=true
  ```

- 截图：

  ![building_4f_static](pics/building_4f_static.png)

### `worlds/clearpath_playpen/clearpath_playpen.world`

- 场景内容：Clearpath playpen 地面机器人场地，用于 Gazebo 场景加载和仿真验证。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/clearpath_playpen/clearpath_playpen.world" gui:=true paused:=true
  ```

- 截图：

  ![clearpath_playpen](pics/clearpath_playpen.png)

### `worlds/corridor_dynamic_9/corridor_dynamic_9.world`

- 场景内容：走廊场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/corridor_dynamic_9/corridor_dynamic_9.world" gui:=true paused:=true
  ```

- 截图：

  ![corridor_dynamic_9](pics/corridor_dynamic_9.png)

### `worlds/corridor_exp/corridor_exp.world`

- 场景内容：走廊场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/corridor_exp/corridor_exp.world" gui:=true paused:=true
  ```

- 截图：

  ![corridor_exp](pics/corridor_exp.png)

### `worlds/corridor_static/corridor_static.world`

- 场景内容：走廊场景，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/corridor_static/corridor_static.world" gui:=true paused:=true
  ```

- 截图：

  ![corridor_static](pics/corridor_static.png)

### `worlds/drone_arena/drone_arena.world`

- 场景内容：无人机测试场地，用于 Gazebo 场景加载和仿真验证。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/drone_arena/drone_arena.world" gui:=true paused:=true
  ```

- 截图：

  ![drone_arena](pics/drone_arena.png)

### `worlds/drone_arena_static/drone_arena_static.world`

- 场景内容：无人机测试场地，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/drone_arena_static/drone_arena_static.world" gui:=true paused:=true
  ```

- 截图：

  ![drone_arena_static](pics/drone_arena_static.png)

### `worlds/empty/empty.world`

- 场景内容：基础空场景，用于基础加载和坐标/光照检查。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/empty/empty.world" gui:=true paused:=true
  ```

- 截图：

  ![empty](pics/empty.png)

### `worlds/floor1_dynamic/floor1_dynamic.world`

- 场景内容：室内楼层平面场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floor1_dynamic/floor1_dynamic.world" gui:=true paused:=true
  ```

- 截图：

  ![floor1_dynamic](pics/floor1_dynamic.png)

### `worlds/floor1_empty/floor1_empty.world`

- 场景内容：室内楼层平面场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floor1_empty/floor1_empty.world" gui:=true paused:=true
  ```

- 截图：

  ![floor1_empty](pics/floor1_empty.png)

### `worlds/floor1_static_1/floor1_static_1.world`

- 场景内容：室内楼层平面场景，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floor1_static_1/floor1_static_1.world" gui:=true paused:=true
  ```

- 截图：

  ![floor1_static_1](pics/floor1_static_1.png)

### `worlds/floor2_static_2/floor2_static_2.world`

- 场景内容：室内楼层平面场景，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floor2_static_2/floor2_static_2.world" gui:=true paused:=true
  ```

- 截图：

  ![floor2_static_2](pics/floor2_static_2.png)

### `worlds/floorplan1_dynamic_16/floorplan1_dynamic_16.world`

- 场景内容：室内楼层平面场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floorplan1_dynamic_16/floorplan1_dynamic_16.world" gui:=true paused:=true
  ```

- 截图：

  ![floorplan1_dynamic_16](pics/floorplan1_dynamic_16.png)

### `worlds/floorplan1_dynamic_6/floorplan1_dynamic_6.world`

- 场景内容：室内楼层平面场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floorplan1_dynamic_6/floorplan1_dynamic_6.world" gui:=true paused:=true
  ```

- 截图：

  ![floorplan1_dynamic_6](pics/floorplan1_dynamic_6.png)

### `worlds/floorplan1_static/floorplan1_static.world`

- 场景内容：室内楼层平面场景，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floorplan1_static/floorplan1_static.world" gui:=true paused:=true
  ```

- 截图：

  ![floorplan1_static](pics/floorplan1_static.png)

### `worlds/floorplan2_dynamic_12/floorplan2_dynamic_12.world`

- 场景内容：室内楼层平面场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floorplan2_dynamic_12/floorplan2_dynamic_12.world" gui:=true paused:=true
  ```

- 截图：

  ![floorplan2_dynamic_12](pics/floorplan2_dynamic_12.png)

### `worlds/floorplan2_dynamic_5/floorplan2_dynamic_5.world`

- 场景内容：室内楼层平面场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floorplan2_dynamic_5/floorplan2_dynamic_5.world" gui:=true paused:=true
  ```

- 截图：

  ![floorplan2_dynamic_5](pics/floorplan2_dynamic_5.png)

### `worlds/floorplan2_static/floorplan2_static.world`

- 场景内容：室内楼层平面场景，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floorplan2_static/floorplan2_static.world" gui:=true paused:=true
  ```

- 截图：

  ![floorplan2_static](pics/floorplan2_static.png)

### `worlds/floorplan3_box_dynamic_4/floorplan3_box_dynamic_4.world`

- 场景内容：室内楼层平面场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floorplan3_box_dynamic_4/floorplan3_box_dynamic_4.world" gui:=true paused:=true
  ```

- 截图：

  ![floorplan3_box_dynamic_4](pics/floorplan3_box_dynamic_4.png)

### `worlds/floorplan3_box_static/floorplan3_box_static.world`

- 场景内容：室内楼层平面场景，静态布景，包含箱体障碍物。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floorplan3_box_static/floorplan3_box_static.world" gui:=true paused:=true
  ```

- 截图：

  ![floorplan3_box_static](pics/floorplan3_box_static.png)

### `worlds/floorplan3_dynamic_8/floorplan3_dynamic_8.world`

- 场景内容：室内楼层平面场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floorplan3_dynamic_8/floorplan3_dynamic_8.world" gui:=true paused:=true
  ```

- 截图：

  ![floorplan3_dynamic_8](pics/floorplan3_dynamic_8.png)

### `worlds/floorplan3_static/floorplan3_static.world`

- 场景内容：室内楼层平面场景，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floorplan3_static/floorplan3_static.world" gui:=true paused:=true
  ```

- 截图：

  ![floorplan3_static](pics/floorplan3_static.png)

### `worlds/floorplan4_static/floorplan4_static.world`

- 场景内容：室内楼层平面场景，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/floorplan4_static/floorplan4_static.world" gui:=true paused:=true
  ```

- 截图：

  ![floorplan4_static](pics/floorplan4_static.png)

### `worlds/gen_env_generated_env/gen_env_generated_env.world`

- 场景内容：程序生成环境场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/gen_env_generated_env/gen_env_generated_env.world" gui:=true paused:=true
  ```

- 截图：

  ![gen_env_generated_env](pics/gen_env_generated_env.png)

### `worlds/generated_env/generated_env.world`

- 场景内容：程序生成环境场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/generated_env/generated_env.world" gui:=true paused:=true
  ```

- 截图：

  ![generated_env](pics/generated_env.png)

### `worlds/generated_env_mod/generated_env_mod.world`

- 场景内容：程序生成环境场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/generated_env_mod/generated_env_mod.world" gui:=true paused:=true
  ```

- 截图：

  ![generated_env_mod](pics/generated_env_mod.png)

### `worlds/generated_env_mod_1/generated_env_mod_1.world`

- 场景内容：程序生成环境场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/generated_env_mod_1/generated_env_mod_1.world" gui:=true paused:=true
  ```

- 截图：

  ![generated_env_mod_1](pics/generated_env_mod_1.png)

### `worlds/room_empty/room_empty.world`

- 场景内容：房间场景，用于基础加载和坐标/光照检查。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/room_empty/room_empty.world" gui:=true paused:=true
  ```

- 截图：

  ![room_empty](pics/room_empty.png)

### `worlds/room_static/room_static.world`

- 场景内容：房间场景，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/room_static/room_static.world" gui:=true paused:=true
  ```

- 截图：

  ![room_static](pics/room_static.png)

### `worlds/simple_box_dynamic_3/simple_box_dynamic_3.world`

- 场景内容：箱体障碍物场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/simple_box_dynamic_3/simple_box_dynamic_3.world" gui:=true paused:=true
  ```

- 截图：

  ![simple_box_dynamic_3](pics/simple_box_dynamic_3.png)

### `worlds/simple_box_static/simple_box_static.world`

- 场景内容：箱体障碍物场景，静态布景，包含箱体障碍物。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/simple_box_static/simple_box_static.world" gui:=true paused:=true
  ```

- 截图：

  ![simple_box_static](pics/simple_box_static.png)

### `worlds/square_box/square_box.world`

- 场景内容：方形障碍物场景，包含箱体障碍物。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/square_box/square_box.world" gui:=true paused:=true
  ```

- 截图：

  ![square_box](pics/square_box.png)

### `worlds/square_box_dynamic_14/square_box_dynamic_14.world`

- 场景内容：方形障碍物场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/square_box_dynamic_14/square_box_dynamic_14.world" gui:=true paused:=true
  ```

- 截图：

  ![square_box_dynamic_14](pics/square_box_dynamic_14.png)

### `worlds/square_box_dynamic_8/square_box_dynamic_8.world`

- 场景内容：方形障碍物场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/square_box_dynamic_8/square_box_dynamic_8.world" gui:=true paused:=true
  ```

- 截图：

  ![square_box_dynamic_8](pics/square_box_dynamic_8.png)

### `worlds/square_dynamic_7/square_dynamic_7.world`

- 场景内容：方形障碍物场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/square_dynamic_7/square_dynamic_7.world" gui:=true paused:=true
  ```

- 截图：

  ![square_dynamic_7](pics/square_dynamic_7.png)

### `worlds/square_exp/square_exp.world`

- 场景内容：方形障碍物场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/square_exp/square_exp.world" gui:=true paused:=true
  ```

- 截图：

  ![square_exp](pics/square_exp.png)

### `worlds/square_static/square_static.world`

- 场景内容：方形障碍物场景，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/square_static/square_static.world" gui:=true paused:=true
  ```

- 截图：

  ![square_static](pics/square_static.png)

### `worlds/square_test/square_test.world`

- 场景内容：方形障碍物场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/square_test/square_test.world" gui:=true paused:=true
  ```

- 截图：

  ![square_test](pics/square_test.png)

### `worlds/test/test.world`

- 场景内容：测试场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/test/test.world" gui:=true paused:=true
  ```

- 截图：

  ![test](pics/test.png)

### `worlds/test2/test2.world`

- 场景内容：测试场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/test2/test2.world" gui:=true paused:=true
  ```

- 截图：

  ![test2](pics/test2.png)

### `worlds/test_empty/test_empty.world`

- 场景内容：测试场景，用于基础加载和坐标/光照检查。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/test_empty/test_empty.world" gui:=true paused:=true
  ```

- 截图：

  ![test_empty](pics/test_empty.png)

### `worlds/test_mill19_1/test_mill19_1.world`

- 场景内容：Mill19 测试场景，用于 Gazebo 场景加载和仿真验证。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/test_mill19_1/test_mill19_1.world" gui:=true paused:=true
  ```

- 截图：

  ![test_mill19_1](pics/test_mill19_1.png)

### `worlds/test_mill19_2/test_mill19_2.world`

- 场景内容：Mill19 测试场景，用于 Gazebo 场景加载和仿真验证。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/test_mill19_2/test_mill19_2.world" gui:=true paused:=true
  ```

- 截图：

  ![test_mill19_2](pics/test_mill19_2.png)

### `worlds/test_mill19_3/test_mill19_3.world`

- 场景内容：Mill19 测试场景，用于 Gazebo 场景加载和仿真验证。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/test_mill19_3/test_mill19_3.world" gui:=true paused:=true
  ```

- 截图：

  ![test_mill19_3](pics/test_mill19_3.png)

### `worlds/test_mill19_floor2/test_mill19_floor2.world`

- 场景内容：Mill19 测试场景，用于 Gazebo 场景加载和仿真验证。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/test_mill19_floor2/test_mill19_floor2.world" gui:=true paused:=true
  ```

- 截图：

  ![test_mill19_floor2](pics/test_mill19_floor2.png)

### `worlds/test_mill19_floor2_box/test_mill19_floor2_box.world`

- 场景内容：箱体障碍物场景，包含箱体障碍物。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/test_mill19_floor2_box/test_mill19_floor2_box.world" gui:=true paused:=true
  ```

- 截图：

  ![test_mill19_floor2_box](pics/test_mill19_floor2_box.png)

### `worlds/tunnel_basic_static/tunnel_basic_static.world`

- 场景内容：隧道场景，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/tunnel_basic_static/tunnel_basic_static.world" gui:=true paused:=true
  ```

- 截图：

  ![tunnel_basic_static](pics/tunnel_basic_static.png)

### `worlds/tunnel_c_shape_basic_static/tunnel_c_shape_basic_static.world`

- 场景内容：隧道场景，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/tunnel_c_shape_basic_static/tunnel_c_shape_basic_static.world" gui:=true paused:=true
  ```

- 截图：

  ![tunnel_c_shape_basic_static](pics/tunnel_c_shape_basic_static.png)

### `worlds/tunnel_dynamic_1/tunnel_dynamic_1.world`

- 场景内容：隧道场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/tunnel_dynamic_1/tunnel_dynamic_1.world" gui:=true paused:=true
  ```

- 截图：

  ![tunnel_dynamic_1](pics/tunnel_dynamic_1.png)

### `worlds/tunnel_fukushima_dynamic1/tunnel_fukushima_dynamic1.world`

- 场景内容：隧道场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/tunnel_fukushima_dynamic1/tunnel_fukushima_dynamic1.world" gui:=true paused:=true
  ```

- 截图：

  ![tunnel_fukushima_dynamic1](pics/tunnel_fukushima_dynamic1.png)

### `worlds/tunnel_fukushima_static/tunnel_fukushima_static.world`

- 场景内容：隧道场景，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/tunnel_fukushima_static/tunnel_fukushima_static.world" gui:=true paused:=true
  ```

- 截图：

  ![tunnel_fukushima_static](pics/tunnel_fukushima_static.png)

### `worlds/tunnel_fukushima_static_box/tunnel_fukushima_static_box.world`

- 场景内容：隧道场景，静态布景，包含箱体障碍物。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/tunnel_fukushima_static_box/tunnel_fukushima_static_box.world" gui:=true paused:=true
  ```

- 截图：

  ![tunnel_fukushima_static_box](pics/tunnel_fukushima_static_box.png)

### `worlds/tunnel_s_shape_basic_static/tunnel_s_shape_basic_static.world`

- 场景内容：隧道场景，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/tunnel_s_shape_basic_static/tunnel_s_shape_basic_static.world" gui:=true paused:=true
  ```

- 截图：

  ![tunnel_s_shape_basic_static](pics/tunnel_s_shape_basic_static.png)

### `worlds/tunnel_static/tunnel_static.world`

- 场景内容：隧道场景，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/tunnel_static/tunnel_static.world" gui:=true paused:=true
  ```

- 截图：

  ![tunnel_static](pics/tunnel_static.png)

### `worlds/tunnel_static_long/tunnel_static_long.world`

- 场景内容：隧道场景，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/tunnel_static_long/tunnel_static_long.world" gui:=true paused:=true
  ```

- 截图：

  ![tunnel_static_long](pics/tunnel_static_long.png)

### `worlds/tunnel_straight_basic_static/tunnel_straight_basic_static.world`

- 场景内容：隧道场景，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/tunnel_straight_basic_static/tunnel_straight_basic_static.world" gui:=true paused:=true
  ```

- 截图：

  ![tunnel_straight_basic_static](pics/tunnel_straight_basic_static.png)

### `worlds/tunnel_straight_dynamic_5/tunnel_straight_dynamic_5.world`

- 场景内容：隧道场景，包含动态障碍物，运动由本包发布的 `libobstaclePathPlugin.so` 驱动。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/tunnel_straight_dynamic_5/tunnel_straight_dynamic_5.world" gui:=true paused:=true
  ```

- 截图：

  ![tunnel_straight_dynamic_5](pics/tunnel_straight_dynamic_5.png)

### `worlds/tunnel_straight_static/tunnel_straight_static.world`

- 场景内容：隧道场景，静态布景。
- 启动命令：

  ```bash
  roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/tunnel_straight_static/tunnel_straight_static.world" gui:=true paused:=true
  ```

- 截图：

  ![tunnel_straight_static](pics/tunnel_straight_static.png)

<!-- WORLD_CATALOG_END -->
