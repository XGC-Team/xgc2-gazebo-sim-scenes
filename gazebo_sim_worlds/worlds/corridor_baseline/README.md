# corridor_baseline

TARE + CMU motion-primitives + MINCO/NMPC 的宽松基础验收场景。

- 物理步长 `0.001 s`、更新频率 `1000 Hz`
- 走廊有效宽度约 `5.85 m`
- 障碍物间距 `7 m`，有效通道显著大于 Scout `0.611 × 0.574 m` 足迹

```bash
scripts/launch_full_exploration \
  world_name:="$(rospack find gazebo_sim_worlds)/worlds/corridor_baseline/corridor_baseline.world"
```
