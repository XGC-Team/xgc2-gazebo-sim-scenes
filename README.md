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
- `/opt/ros/noetic/share/gazebo_sim_worlds/worlds/weston_robot_empty/weston_robot_empty.world`
- `/opt/ros/noetic/share/gazebo_sim_worlds/worlds/clearpath_playpen/clearpath_playpen.world`
- `/opt/ros/noetic/share/gazebo_sim_worlds/worlds/corridor_dynamic_9/corridor_dynamic_9.world`
- `/opt/ros/noetic/share/gazebo_sim_worlds/models/corridor/model.sdf`
- `/opt/ros/noetic/share/gazebo_sim_worlds/models/jersey_barrier/model.sdf`

## CI Checks

The package compliance check validates every world XML file, every shared model metadata file, same-name scene layout, all `model://` references against the packaged `models/` directory, and Gazebo SDF validity through `gz sdf -k`.
