# XGC2 Gazebo Sim Worlds

Reusable Gazebo Classic world and model assets for XGC2 simulation products.

This ROS package owns assets only. Model control belongs to the sibling
`xgc2_gazebo_scene` director package in the same scene repository. Vehicle
launch orchestration, controller startup, VRPN routing, and estimator startup
are owned by XGC2 and the corresponding robot simulation products.

## Packages

- `gazebo_sim_worlds`

## APT

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-gazebo-sim-worlds
```

## Installed Assets

- `/opt/ros/noetic/share/gazebo_sim_worlds/worlds/empty/empty.world`
- `/opt/ros/noetic/share/gazebo_sim_worlds/worlds/clearpath_playpen/clearpath_playpen.world`
- `/opt/ros/noetic/share/gazebo_sim_worlds/worlds/corridor_dynamic_9/corridor_dynamic_9.world`
- `/opt/ros/noetic/share/gazebo_sim_worlds/worlds/<scene>/README.md`
- `/opt/ros/noetic/share/gazebo_sim_worlds/worlds/<scene>/preview.png`（有截图的场景）
- `/opt/ros/noetic/share/gazebo_sim_worlds/worlds/catalog/<scene>.world`（供文件选择器连续浏览的扁平索引）
- `/opt/ros/noetic/share/gazebo_sim_worlds/worlds/catalog/<scene>.md`
- `/opt/ros/noetic/share/gazebo_sim_worlds/worlds/catalog/<scene>.png`（有截图的场景）
- `/opt/ros/noetic/share/gazebo_sim_worlds/models/corridor/model.sdf`
- `/opt/ros/noetic/share/gazebo_sim_worlds/models/jersey_barrier/model.sdf`

## CI Checks

The package compliance check validates every world XML file, every shared model metadata file, same-name scene layout, required per-scene README files, optional PNG previews, all `model://` references against the packaged `models/` directory, and Gazebo SDF validity through `gz sdf -k`.

<!-- WORLD_CATALOG_START -->

## World Catalog

每个场景由 world 同目录下的伴随文件自描述：标准场景目录使用 `README.md` 和可选的 `preview.png`；扁平 `worlds/catalog` 索引使用与 world 同名的 `<scene>.md` 和可选的 `<scene>.png`。工具应相对 `.world` 文件自动解析这些文件；伴随文件缺失不影响 world 加载或选择。

GUI 文件选择器应进入 `/opt/ros/noetic/share/gazebo_sim_worlds/worlds/catalog`，即可在同一列表中连续选择和预览全部场景。原有 `worlds/<scene>/<scene>.world` 路径继续保留，兼容已有 launch 和脚本。

| Scene | Summary | Preview |
| --- | --- | --- |
| [`bridge_static`](worlds/bridge_static/README.md) | 桥梁场景，静态布景。 | [preview](worlds/bridge_static/preview.png) |
| [`building_2f_static`](worlds/building_2f_static/README.md) | 建筑物场景，静态布景。 | [preview](worlds/building_2f_static/preview.png) |
| [`building_4f_static`](worlds/building_4f_static/README.md) | 建筑物场景，静态布景。 | [preview](worlds/building_4f_static/preview.png) |
| [`camera_calibration_extrinsic`](worlds/camera_calibration_extrinsic/README.md) | 相机外参标定场景，在三维空间布置六个彩色标记，用于验证相机位姿与世界坐标系之间的外参。 | — |
| [`camera_calibration_intrinsic`](worlds/camera_calibration_intrinsic/README.md) | 相机内参标定场景，提供 8×6 棋盘格目标和稳定光照，用于采集 Gazebo 相机标定图像。 | — |
| [`clearpath_playpen`](worlds/clearpath_playpen/README.md) | Clearpath playpen 地面机器人场地，用于 Gazebo 场景加载和仿真验证。 | [preview](worlds/clearpath_playpen/preview.png) |
| [`corridor_dynamic_9`](worlds/corridor_dynamic_9/README.md) | 走廊场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/corridor_dynamic_9/preview.png) |
| [`corridor_exp`](worlds/corridor_exp/README.md) | 走廊场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/corridor_exp/preview.png) |
| [`corridor_static`](worlds/corridor_static/README.md) | 走廊场景，静态布景。 | [preview](worlds/corridor_static/preview.png) |
| [`drone_arena`](worlds/drone_arena/README.md) | 无人机测试场地，用于 Gazebo 场景加载和仿真验证。 | [preview](worlds/drone_arena/preview.png) |
| [`drone_arena_static`](worlds/drone_arena_static/README.md) | 无人机测试场地，静态布景。 | [preview](worlds/drone_arena_static/preview.png) |
| [`empty`](worlds/empty/README.md) | 基础空场景，用于基础加载和坐标/光照检查。 | [preview](worlds/empty/preview.png) |
| [`floor1_dynamic`](worlds/floor1_dynamic/README.md) | 室内楼层平面场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/floor1_dynamic/preview.png) |
| [`floor1_empty`](worlds/floor1_empty/README.md) | 室内楼层平面场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/floor1_empty/preview.png) |
| [`floor1_static_1`](worlds/floor1_static_1/README.md) | 室内楼层平面场景，静态布景。 | [preview](worlds/floor1_static_1/preview.png) |
| [`floor2_static_2`](worlds/floor2_static_2/README.md) | 室内楼层平面场景，静态布景。 | [preview](worlds/floor2_static_2/preview.png) |
| [`floorplan1_dynamic_16`](worlds/floorplan1_dynamic_16/README.md) | 室内楼层平面场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/floorplan1_dynamic_16/preview.png) |
| [`floorplan1_dynamic_6`](worlds/floorplan1_dynamic_6/README.md) | 室内楼层平面场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/floorplan1_dynamic_6/preview.png) |
| [`floorplan1_static`](worlds/floorplan1_static/README.md) | 室内楼层平面场景，静态布景。 | [preview](worlds/floorplan1_static/preview.png) |
| [`floorplan2_dynamic_12`](worlds/floorplan2_dynamic_12/README.md) | 室内楼层平面场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/floorplan2_dynamic_12/preview.png) |
| [`floorplan2_dynamic_5`](worlds/floorplan2_dynamic_5/README.md) | 室内楼层平面场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/floorplan2_dynamic_5/preview.png) |
| [`floorplan2_static`](worlds/floorplan2_static/README.md) | 室内楼层平面场景，静态布景。 | [preview](worlds/floorplan2_static/preview.png) |
| [`floorplan3_box_dynamic_4`](worlds/floorplan3_box_dynamic_4/README.md) | 室内楼层平面场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/floorplan3_box_dynamic_4/preview.png) |
| [`floorplan3_box_static`](worlds/floorplan3_box_static/README.md) | 室内楼层平面场景，静态布景，包含箱体障碍物。 | [preview](worlds/floorplan3_box_static/preview.png) |
| [`floorplan3_dynamic_8`](worlds/floorplan3_dynamic_8/README.md) | 室内楼层平面场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/floorplan3_dynamic_8/preview.png) |
| [`floorplan3_static`](worlds/floorplan3_static/README.md) | 室内楼层平面场景，静态布景。 | [preview](worlds/floorplan3_static/preview.png) |
| [`floorplan4_static`](worlds/floorplan4_static/README.md) | 室内楼层平面场景，静态布景。 | [preview](worlds/floorplan4_static/preview.png) |
| [`gen_env_generated_env`](worlds/gen_env_generated_env/README.md) | 程序生成环境场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/gen_env_generated_env/preview.png) |
| [`generated_env`](worlds/generated_env/README.md) | 程序生成环境场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/generated_env/preview.png) |
| [`generated_env_mod`](worlds/generated_env_mod/README.md) | 程序生成环境场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/generated_env_mod/preview.png) |
| [`generated_env_mod_1`](worlds/generated_env_mod_1/README.md) | 程序生成环境场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/generated_env_mod_1/preview.png) |
| [`room_empty`](worlds/room_empty/README.md) | 房间场景，用于基础加载和坐标/光照检查。 | [preview](worlds/room_empty/preview.png) |
| [`room_static`](worlds/room_static/README.md) | 房间场景，静态布景。 | [preview](worlds/room_static/preview.png) |
| [`simple_box_dynamic_3`](worlds/simple_box_dynamic_3/README.md) | 箱体障碍物场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/simple_box_dynamic_3/preview.png) |
| [`simple_box_static`](worlds/simple_box_static/README.md) | 箱体障碍物场景，静态布景，包含箱体障碍物。 | [preview](worlds/simple_box_static/preview.png) |
| [`square_box`](worlds/square_box/README.md) | 方形障碍物场景，包含箱体障碍物。 | [preview](worlds/square_box/preview.png) |
| [`square_box_dynamic_14`](worlds/square_box_dynamic_14/README.md) | 方形障碍物场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/square_box_dynamic_14/preview.png) |
| [`square_box_dynamic_8`](worlds/square_box_dynamic_8/README.md) | 方形障碍物场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/square_box_dynamic_8/preview.png) |
| [`square_dynamic_7`](worlds/square_dynamic_7/README.md) | 方形障碍物场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/square_dynamic_7/preview.png) |
| [`square_exp`](worlds/square_exp/README.md) | 方形障碍物场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/square_exp/preview.png) |
| [`square_static`](worlds/square_static/README.md) | 方形障碍物场景，静态布景。 | [preview](worlds/square_static/preview.png) |
| [`square_test`](worlds/square_test/README.md) | 方形障碍物场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/square_test/preview.png) |
| [`test`](worlds/test/README.md) | 测试场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/test/preview.png) |
| [`test2`](worlds/test2/README.md) | 测试场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/test2/preview.png) |
| [`test_empty`](worlds/test_empty/README.md) | 测试场景，用于基础加载和坐标/光照检查。 | [preview](worlds/test_empty/preview.png) |
| [`test_mill19_1`](worlds/test_mill19_1/README.md) | Mill19 测试场景，用于 Gazebo 场景加载和仿真验证。 | [preview](worlds/test_mill19_1/preview.png) |
| [`test_mill19_2`](worlds/test_mill19_2/README.md) | Mill19 测试场景，用于 Gazebo 场景加载和仿真验证。 | [preview](worlds/test_mill19_2/preview.png) |
| [`test_mill19_3`](worlds/test_mill19_3/README.md) | Mill19 测试场景，用于 Gazebo 场景加载和仿真验证。 | [preview](worlds/test_mill19_3/preview.png) |
| [`test_mill19_floor2`](worlds/test_mill19_floor2/README.md) | Mill19 测试场景，用于 Gazebo 场景加载和仿真验证。 | [preview](worlds/test_mill19_floor2/preview.png) |
| [`test_mill19_floor2_box`](worlds/test_mill19_floor2_box/README.md) | 箱体障碍物场景，包含箱体障碍物。 | [preview](worlds/test_mill19_floor2_box/preview.png) |
| [`tunnel_basic_static`](worlds/tunnel_basic_static/README.md) | 隧道场景，静态布景。 | [preview](worlds/tunnel_basic_static/preview.png) |
| [`tunnel_c_shape_basic_static`](worlds/tunnel_c_shape_basic_static/README.md) | 隧道场景，静态布景。 | [preview](worlds/tunnel_c_shape_basic_static/preview.png) |
| [`tunnel_dynamic_1`](worlds/tunnel_dynamic_1/README.md) | 隧道场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/tunnel_dynamic_1/preview.png) |
| [`tunnel_fukushima_dynamic1`](worlds/tunnel_fukushima_dynamic1/README.md) | 隧道场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/tunnel_fukushima_dynamic1/preview.png) |
| [`tunnel_fukushima_static`](worlds/tunnel_fukushima_static/README.md) | 隧道场景，静态布景。 | [preview](worlds/tunnel_fukushima_static/preview.png) |
| [`tunnel_fukushima_static_box`](worlds/tunnel_fukushima_static_box/README.md) | 隧道场景，静态布景，包含箱体障碍物。 | [preview](worlds/tunnel_fukushima_static_box/preview.png) |
| [`tunnel_s_shape_basic_static`](worlds/tunnel_s_shape_basic_static/README.md) | 隧道场景，静态布景。 | [preview](worlds/tunnel_s_shape_basic_static/preview.png) |
| [`tunnel_static`](worlds/tunnel_static/README.md) | 隧道场景，静态布景。 | [preview](worlds/tunnel_static/preview.png) |
| [`tunnel_static_long`](worlds/tunnel_static_long/README.md) | 隧道场景，静态布景。 | [preview](worlds/tunnel_static_long/preview.png) |
| [`tunnel_straight_basic_static`](worlds/tunnel_straight_basic_static/README.md) | 隧道场景，静态布景。 | [preview](worlds/tunnel_straight_basic_static/preview.png) |
| [`tunnel_straight_dynamic_5`](worlds/tunnel_straight_dynamic_5/README.md) | 隧道场景，包含动态障碍物，运动由场景导演包发布的 `libobstaclePathPlugin.so` 驱动。 | [preview](worlds/tunnel_straight_dynamic_5/preview.png) |
| [`tunnel_straight_static`](worlds/tunnel_straight_static/README.md) | 隧道场景，静态布景。 | [preview](worlds/tunnel_straight_static/preview.png) |

<!-- WORLD_CATALOG_END -->
