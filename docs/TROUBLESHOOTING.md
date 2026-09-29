# Troubleshooting

## Duplicate processes

**Symptom:** `ros2 lifecycle get <node>` and `ros2 lifecycle set <node> <transition>`
give contradictory answers (e.g. one call says `unconfigured`, the next says the node
only accepts `cleanup`/`activate`/`shutdown`). `/tf` shows more publishers than
expected for `ekf_filter_node` or `rf2o_laser_odometry`.

**Cause:** More than one copy of `navigation.launch.py` and/or `ekf.launch.py` is
running at once — usually from a manual `ros2 launch` that was left running
alongside `console_launch.py`, or from running `console_launch.py` twice. Every
copy uses unnamespaced node names, so `ros2` commands hit whichever process
answers first.

**Fix:**
```bash
ps aux | grep -E "ros2 launch|ekf_node|rf2o|nav2|amcl|map_server|bt_navigator|lifecycle_manager|launch_manager" | grep -v grep
```
Kill every matching PID (`kill -SIGINT <pid>`, then `kill -9` if it doesn't exit),
restart the ROS daemon (`ros2 daemon stop && ros2 daemon start`), confirm
`ros2 node list` is clean, then start **only** through `console_launch.py` and the
web console's buttons from then on.

**Guardrails added to `launch_manager_node.py`:**
- Before starting EKF, check `ros2 node list` for an existing `/ekf_filter_node`
  and skip starting a second one if found.
- Before starting mapping/navigation, check for an existing `nav2_container` /
  `slam_toolbox` node not owned by this manager and refuse (publish an `error:`)
  instead of layering a second stack on top.

## AMCL / bt_navigator stuck in `unconfigured` or `inactive`

Usually a symptom of the duplicate-process issue above, not a real configuration
problem — re-check after a clean restart. If it persists on a genuinely clean
graph:
```bash
ros2 lifecycle get /map_server
ros2 lifecycle set /map_server configure   # prints the actual failure
```
Common real causes: `map:=` path wrong or file missing, node name in the params
YAML not matching `/amcl` or `/map_server`, or a lifecycle manager whose
`node_names` list references a node that never started.

## `ros2 lifecycle get /a, /b` errors with "unrecognized arguments"

`ros2 lifecycle get` takes one node at a time — run it once per node, not
comma-separated.

## No `map` frame in `view_frames`

If the TF tree only shows `odom → base_footprint → base_link → …` with no `map`,
AMCL either isn't running or hasn't configured yet — see above. `bt_navigator`
needs `map → base_link` to operate and will stay inactive until it's available.

## EKF started twice

`console_launch.py`'s docstring assumes `bringup.launch.py` already includes
`ekf.launch.py`; `launch_manager_node.py`'s docstring says that include doesn't
resolve on this setup, so the manager starts EKF itself. **Only one of these
should be true at a time.** After starting bringup on the Pi, run
`ros2 node list | grep ekf` before starting the laptop side — if
`/ekf_filter_node` already appears, remove the EKF start from
`launch_manager_node.py` (`start_ekf` / `watch_ekf` / `ensure_ekf`) rather than
fixing bringup's include, to avoid reintroducing the two-rf2o-nodes conflict.

## Robot doesn't avoid an obstacle

- Check `robot_radius` in `local_costmap`/`global_costmap` against the real
  chassis — too small means the robot thinks it has more clearance than it does.
- The lidar is a single 2D plane; obstacles above/below scan height are invisible
  to this pipeline (glass, table edges, steps).
- `BaseObstacle.scale` is fairly low relative to `PathAlign`/`GoalAlign`, so the
  controller favors sticking to the planned path over swerving hard in tight
  spaces — raise it if the robot cuts corners too close.
- Watch `local_costmap` and `global_costmap` in RViz live while testing: if the
  path bends around an obstacle, avoidance is working; if the robot just stops,
  the obstacle is likely outside the local costmap window or the lidar's FOV.

## Console can't reach rosbridge from a phone

- Phone and laptop must be on the **same Wi-Fi**, not mobile data.
- Confirm rosbridge is bound to all interfaces, not just localhost:
  ```bash
  ss -tlnp | grep 9090   # should show 0.0.0.0:9090
  ```
- Confirm the HTML is served with `--bind 0.0.0.0`, not the default (localhost-only
  on some setups).
- Open both ports if a firewall is active: `sudo ufw allow 8000/tcp && sudo ufw allow 9090/tcp`.
