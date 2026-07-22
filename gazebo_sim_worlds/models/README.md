# XGC2 Gazebo Models

The `xgc2_geom_*` models are reusable kinematic obstacle assets built entirely
from SDF primitive geometry. Their `<collision>` elements are the canonical
collision description and form an exact convex decomposition of each model.
Visual elements intentionally mirror the collision elements. Kinematic links
remain in the physics engine, ignore gravity, and can be relocated through
`/gazebo/set_model_state`; do not convert them to `<static>true>`.

| Model URI | Convex collision parts |
| --- | ---: |
| `model://xgc2_geom_cube` | 1 box |
| `model://xgc2_geom_cuboid` | 1 box |
| `model://xgc2_geom_sphere` | 1 sphere |
| `model://xgc2_geom_cylinder` | 1 cylinder |
| `model://xgc2_geom_capped_pillar` | 1 cylinder + 1 sphere |
| `model://xgc2_geom_l_block` | 2 boxes |
| `model://xgc2_geom_t_block` | 2 boxes |
| `model://xgc2_geom_arch` | 3 boxes |
| `model://xgc2_geom_dumbbell` | 1 cylinder + 2 spheres |
| `model://xgc2_geom_stairs` | 3 boxes |

Scene-management and planning bridges should use the collision part names as
stable part identifiers and transform each local collision pose by the current
Gazebo model pose.
