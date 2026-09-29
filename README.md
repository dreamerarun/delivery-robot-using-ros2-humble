# AMR Workspace

A ROS 2 (Humble) autonomous mobile robot stack: differential-drive base, 2D lidar,
IMU, EKF-fused localization, Nav2 navigation, and a browser-based control console
usable from a laptop or phone on the same network.

No wheel encoders — odometry comes from **lidar scan matching (rf2o)** fused with
the **IMU** through an EKF (`robot_localization`).

---

## Packages

| Package | What it does |
|---|---|
| `amr_bringup` | Robot description, sensor drivers, motor driver, `bringup.launch.py`. Runs on the robot's onboard computer (Pi). |
| `amr_navigation` | EKF, SLAM/localization, Nav2, the launch-manager node, and the web console. Runs from the laptop. |
| `rf2o_laser_odometry` | Third-party package providing lidar-based odometry (`odom_rf2o`) in place of wheel encoders. |

## System architecture

```
Pi (robot):      bringup.launch.py
                    → robot_state_publisher, motor driver, lidar driver, IMU driver

Laptop:          console_launch.py
                    → rosbridge_websocket (port 9090)
                    → launch_manager_node
                         → owns ekf.launch.py (rf2o + EKF + odom_cov_relay)
                         → starts/stops mapping.launch.py or navigation.launch.py
                           on command from the web console

Phone/laptop:    amr_console.html  (served over HTTP, connects to rosbridge over WS)
```

TF tree in navigation mode:

```
map → odom → base_footprint → base_link → {left_wheel, right_wheel, caster_wheel, imu_link, laser_frame}
```

`map → odom` is published by AMCL. `odom → base_footprint` is published by the EKF
(fusing `rf2o` + IMU), **not** by rf2o directly (`publish_tf: false` in `ekf.launch.py`).

See [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) for the full node/topic list and
[`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md) for known failure modes.

---

## Quick start

**On the Pi:**
```bash
ros2 launch amr_bringup bringup.launch.py
```

**On the laptop:**
```bash
ros2 launch amr_navigation console_launch.py
```
This starts `rosbridge_websocket` and `launch_manager_node` together. The
launch-manager starts EKF immediately and keeps it alive; it does **not** start
mapping or navigation until told to.

**Serve the web console** (from wherever `amr_console.html` lives):
```bash
python3 -m http.server 8000 --bind 0.0.0.0
```

**Open the console:**
- Same machine: `http://localhost:8000/amr_console.html`
- Phone/other device on the same Wi-Fi: `http://<laptop-ip>:8000/amr_console.html`

The console's rosbridge field defaults to `ws://<the-host-you-loaded-it-from>:9090`,
so it's usually correct without editing.

From the console you can:
- Start mapping or navigation (pick a saved map for navigation)
- Drive manually with the joystick
- Set the initial pose / relocalize
- Send navigation goals, run a waypoint tour, cancel goals
- Tune `FollowPath` max speed/turn rate live, clear costmaps
- Watch localization quality, map stats, and per-topic health ("receiving" / "silent")

⚠️ Only run **one instance** of `console_launch.py` and never launch
`navigation.launch.py` / `ekf.launch.py` by hand while it's running — see
[`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md#duplicate-processes) for why
that causes conflicting duplicate nodes.

---

## Requirements

- ROS 2 Humble
- Nav2 (`amcl`, `nav2_controller`, `nav2_planner`, `nav2_bt_navigator`,
  `nav2_behaviors`, `nav2_costmap_2d`, `nav2_lifecycle_manager`, `nav2_waypoint_follower`,
  `nav2_velocity_smoother`)
- `robot_localization` (EKF)
- `rf2o_laser_odometry`
- `rosbridge_server`
- A modern browser for the console (uses Pointer Events; works on desktop and mobile)

## Configuration

Key tunables live in `amr_navigation/config/nav2_params.yaml`:

- `controller_server.FollowPath.max_vel_x` / `max_speed_xy` — top linear speed.
  Keep ≤ `max_wheel_speed` in `amr_bringup/config/motor_params.yaml`.
- `local_costmap` / `global_costmap` — `robot_radius` **must match the real
  chassis**; too small and the robot will clip obstacles.
- `amcl` — `base_frame_id: base_link`, tuned for TF chain `odom → base_footprint → base_link`.

EKF config: `amr_navigation/config/ekf.yaml`.

## Repository layout

```
amr_ws/
├── src/
│   ├── amr_bringup/
│   ├── amr_navigation/
│   │   ├── config/          # nav2_params.yaml, ekf.yaml
│   │   ├── launch/          # bringup, mapping, navigation, ekf, console launch files
│   │   ├── maps/            # saved .yaml/.pgm maps
│   │   ├── web/             # amr_console.html
│   │   └── amr_navigation/  # launch_manager_node.py, odom_cov_relay
│   └── rf2o_laser_odometry/
├── docs/
├── README.md
└── .gitignore
```

Adjust this tree to match your actual layout — the paths above reflect where
each file was referenced in this project's setup.
