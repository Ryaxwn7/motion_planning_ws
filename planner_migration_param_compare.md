# Planner Migration Parameter Comparison

Date: 2026-03-20
Scope: `fm2`, `fm2star`, `my_planner`, related launch/config entry points

## Summary

- This migration applies the first three planned steps:
  - upgrade `fm2` implementation
  - upgrade `my_planner` implementation
  - add `fm2star` as an optional global planner
- Real-robot frame/topic conventions were preserved deliberately:
  - `base_frame_id = base_footprint`
  - `odom_frame_id = odom_combined`
  - scan topic remains `/scan`
- `new_ws` algorithm defaults were not copied blindly when they would materially change real-robot behavior.

## 1. Global Planner Entry

| Item | Current workspace before | `new_ws` | Applied in current workspace |
| --- | --- | --- | --- |
| `global_planner` options in `move_base.launch.xml` | includes `fm2`, no `fm2star` | includes `fm2`, `fm2star`, `sfmm2` | includes `fm2`, `fm2star`; `sfmm2` not added in this phase |
| `GraphPlanner` implementation | old `FM2_Planner` only | new `FM2_Planner` + `FM2Star_Planner` | new `FM2_Planner` + `FM2Star_Planner` |
| Dynamic reconfigure for FM2 family | none | `FM2Planner.cfg` | added |

## 2. GraphPlanner General Params

| Param | Current before | `new_ws` | Applied |
| --- | --- | --- | --- |
| `convert_offset` | `0.0` | `0.0` | `0.0` |
| `default_tolerance` | `0.0` | `0.0` | `0.0` |
| `outline_map` | `true` | `false` | `true` |
| `obstacle_factor` | `0.5` | `0.5` | `0.5` |
| `expand_zone` | `true` | `true` | `true` |
| `voronoi_map` | `false` | `false` | `false` |

Decision:
- `outline_map` stays at the current real-robot value `true`. This preserves existing map boundary behavior instead of switching to the simulation default.

## 3. FM2 / FM2Star Params

Current workspace did not previously have an explicit `GraphPlanner.fm2` parameter block in YAML. The old `fm2` implementation relied mostly on hard-coded behavior.

The following block is now added and is based on `new_ws`, with a debug path adapted for the real workspace:

| Param | Current before | `new_ws` | Applied |
| --- | --- | --- | --- |
| `v_max` | implicit | `1.0` | `1.0` |
| `velocity_alpha` | implicit | `0.2` | `0.2` |
| `velocity_dmax` | implicit | `0.5` | `0.5` |
| `robot_radius` | implicit | `0.25` | `0.25` |
| `velocity_mode` | implicit | `1` | `1` |
| `velocity_sigmoid_k` | implicit | `1.5` | `1.5` |
| `velocity_sigmoid_b` | implicit | `0.0` | `0.0` |
| `use_gather_style` | implicit | `true` | `true` |
| `use_star_heuristic` | implicit | `true` | `true` |
| `use_dynamic_obstacle_uncertainty` | implicit | `true` | `true` |
| `use_dynamic_obstacle_cluster_tracking` | implicit | `false` | `false` |
| `force_full_occupancy_refresh` | implicit | `false` | `false` |
| `dynamic_obstacle_threshold` | implicit | `253` | `253` |
| `dynamic_obstacle_unknown_is_obstacle` | implicit | `false` | `false` |
| `dynamic_obstacle_inflation_radius` | implicit | `0.25` | `0.25` |
| `dynamic_obstacle_uncertainty_radius` | implicit | `0.60` | `0.60` |
| `dynamic_obstacle_stride` | implicit | `1` | `1` |
| `dynamic_obstacle_cluster_min_cells` | implicit | `2` | `2` |
| `dynamic_obstacle_track_match_radius` | implicit | `0.50` | `0.50` |
| `dynamic_obstacle_track_ttl` | implicit | `0.60` | `0.60` |
| `static_obstacle_noise_reject_cells` | implicit | `4` | `4` |
| `dynamic_obstacle_max_samples` | implicit | `800` | `800` |
| `debug_velocity_path` | implicit | `/home/yxw/new_ws/data/fm2_velocity.csv` | `/tmp/fm2_velocity.csv` |
| `debug_dump_velocity` | implicit | `false` | `false` |
| `debug_visualize` | implicit | `false` | `false` |
| `debug_log` | implicit | `false` | `false` |

Decision:
- `fm2star` is wired in, but it is still opt-in through launch arg `global_planner:=fm2star`.
- `sfmm2` was intentionally not added in this phase.

## 4. MyPlanner Params

| Param | Current before | `new_ws` | Applied |
| --- | --- | --- | --- |
| `pre_n` | `15` | `10` | `15` |
| `Kp` | `2.0` | `8.0` | `2.0` |
| `Ki` | `0.01` | `0.01` | `0.01` |
| `Kd` | `3.0` | `3.0` | `3.0` |
| `trans_x_factor` | `1.5` | `1.0` | `1.5` |
| `trans_y_factor` | `1.5` | `1.0` | `1.5` |
| `trans_factor` | not present | `5.0` | `1.0` |
| `target_dist` | `0.2` | `0.2` | `0.2` |
| `max_vel_trans` | `1.0` | `1.0` | `1.0` |
| `max_vel_rot` | `0.9` | `1.57` | `0.9` |
| `acc_scale_trans` | `1.5` | `1.5` | `1.5` |
| `acc_scale_rot` | `0.5` | `0.5` | `0.5` |
| `adjust_r_factor` | `1.0` | `2.5` | `1.0` |
| `goal_dist_tolerance` | `0.1` | `0.1` | `0.1` |
| `goal_yaw_tolerance` | `0.05` | `0.05` | `0.05` |
| `debug_print` | not present | `false` | `false` |
| `scan_topic` | `/scan` | `/scan` | `/scan` |
| `base_frame_id` | `base_footprint` | `base_link` | `base_footprint` |
| `odom_frame_id` | `odom_combined` | `odom` | `odom_combined` |

Decision:
- `my_planner` behavior is upgraded to the new implementation.
- Real-robot control gains and frame ids are kept conservative.
- `trans_factor` is introduced but set to `1.0` so the new implementation stays close to the old linear velocity magnitude.
- `new_ws` values like `Kp=8.0`, `adjust_r_factor=2.5`, `max_vel_rot=1.57`, and `trans_factor=5.0` were not adopted directly because they would change real-robot motion behavior too aggressively.

## 5. Code-Level Behavior Changes Now Introduced

- `fm2` now supports dynamic reconfigure and velocity-field tuning.
- `fm2star` is now available as a planner option.
- `my_planner` now includes:
  - `target_pose` publishing
  - optional debug logging switch
  - translational scaling via `trans_factor`
  - acceleration limiting on output commands

## 6. Suggested Runtime Verification Order

1. Launch with existing default:
   - `global_planner:=fm2`
   - `local_planner:=my`
2. Verify the upgraded `my_planner` tracks correctly under current real-robot gains.
3. Test `global_planner:=fm2star` on the same map and compare:
   - planning latency
   - path smoothness
   - obstacle clearance
   - replan stability near moving obstacles
4. Only after that, retune FM2/FM2Star block if needed.
