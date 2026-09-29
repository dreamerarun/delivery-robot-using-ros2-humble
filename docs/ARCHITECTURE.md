# Architecture

## Machines

| Machine | Runs |
|---|---|
| Pi (on robot) | `amr_bringup/bringup.launch.py`: `robot_state_publisher`, motor driver, lidar driver, IMU driver |
| Laptop | `amr_navigation/console_launch.py`: `rosbridge_websocket` + `launch_manager_node` (which in turn owns EKF and starts/stops mapping or navigation) |
| Phone / laptop browser | `amr_console.html`, served over plain HTTP, talking to rosbridge over WebSocket |

Both machines must be on the same ROS 2 DDS domain / network for topics and
services to be visible across them.

## Launch flow

```
console_launch.py
├── rosbridge_websocket           (port 9090 — bridges ROS topics/services to the web console)
└── launch_manager_node
      ├── starts ekf.launch.py on startup, restarts it if it dies
      │     ├── rf2o_laser_odometry   (publish_tf: false — EKF owns the TF)
      │     ├── ekf_filter_node       (robot_localization; publishes odom → base_footprint)
      │     └── odom_cov_relay
      └── on /amr/mode_cmd:
            "mapping"              → mapping.launch.py     (SLAM)
            "navigation:<map.yaml>"→ navigation.launch.py  (Nav2 + AMCL, map:=<path>)
            "idle"                 → stops the current stack; EKF keeps running
```

`launch_manager_node` communicates with the console over:
- Subscribes: `/amr/mode_cmd` (`std_msgs/String`) — `"idle"`, `"mapping"`, `"navigation:/abs/path/map.yaml"`
- Publishes: `/amr/mode_status` (`std_msgs/String`) — `idle`, `starting_mapping`, `mapping`,
  `starting_navigation`, `navigation`, `stopping`, `error: <reason>`

## Nav2 node graph (navigation mode)

| Node | Role |
|---|---|
| `map_server` | Loads the static map from the `.yaml` passed at launch |
| `amcl` | Particle-filter localization; publishes `map → odom` |
| `planner_server` (NavFn) | Global path planning |
| `controller_server` (DWB) | Local trajectory control and obstacle avoidance |
| `behavior_server` | Recovery behaviors: `spin`, `backup`, `wait` |
| `bt_navigator` | Behavior-tree orchestration of navigate-to-pose / navigate-through-poses |
| `waypoint_follower` | Multi-goal tours |
| `velocity_smoother` | Smooths `cmd_vel` before it reaches the base |
| `lifecycle_manager_localization` | Brings up `map_server` + `amcl` |
| `lifecycle_manager_navigation` | Brings up planner/controller/bt_navigator/behavior/waypoint/velocity_smoother |
| `nav2_container` | Composed-node container hosting the above (where composition is used) |

## TF tree

```
map
 └── odom                     (published by amcl)
      └── base_footprint      (published by ekf_filter_node)
           └── base_link
                ├── left_wheel
                ├── right_wheel
                ├── caster_wheel
                ├── imu_link
                └── laser_frame
```

## Key topics

| Topic | Type | Notes |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Lidar input to rf2o, AMCL, and both costmaps |
| `/odom_rf2o` | `nav_msgs/Odometry` | Raw scan-matching odometry (no TF published) |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF output; consumed by `bt_navigator`, `velocity_smoother` |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | See tree above |
| `/map` | `nav_msgs/OccupancyGrid` | From `map_server` (nav) or `slam_toolbox` (mapping) |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Localization estimate + covariance |
| `/amr/mode_cmd`, `/amr/mode_status` | `std_msgs/String` | Console ↔ launch-manager control channel |

## Services used by the console

- `/controller_server/set_parameters` — live speed/turn-rate limits on `FollowPath`
- `/local_costmap/clear_entirely_local_costmap`, `/global_costmap/clear_entirely_global_costmap`
- `/reinitialize_global_localization` — AMCL global relocalization

## Web console

`amr_console.html` is a single self-contained static file. It connects directly
to `rosbridge_websocket` over a WebSocket (default `ws://<host>:9090`) — no backend
of its own beyond a plain static file server for the HTML itself. It uses Pointer
Events (not mouse/touch-specific handlers), so the joystick and map interactions
work the same on desktop and mobile.
