# scene_editable

可在线布景的通用空世界。只提供地面、光照和场景适配插件；独立用户工作流选择 YAML，
启动场景节点后通过 `/xgc/scene/gazebo/apply` 创建初始障碍。无需启动算法。

World: [`scene_editable.world`](scene_editable.world)

ROS Control 的 Gazebo Server 选择本世界即可，不增加算法专有 YAML 参数。
场景节点统一管理几何、在线编辑、保存与动态运动；插件负责真实 Gazebo 模型和碰撞。
停止算法不会删除场景。Clear 只删除本插件实际创建的模型，不影响机器人、相机或地面。

## Launch

```bash
roslaunch gazebo_ros empty_world.launch world_name:="$(rospack find gazebo_sim_worlds)/worlds/scene_editable/scene_editable.world" gui:=true paused:=true
```

需要同版本 `xgc2_gazebo_scene` 中的 `libxgc2_scene_authoring_world.so` 和包含 Scene 消息的
`xgc2_geometry_msgs`。可通过世界插件的 `scene_namespace` 配置显式场景绑定，缺省为
`/xgc/scene`。操作可以在 Gazebo 暂停时完成，不推进仿真时间。

相机等附加插件可独立组合进世界。不要再给本世界加载算法专有静态障碍，或用旧的
spawn/motion 入口修改这批受场景节点管理的模型。
