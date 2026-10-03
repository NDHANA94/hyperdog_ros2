# Navigation (Nav2) on the obstacle course

```bash
ros2 launch hyperdog_navigation sim_navigation.launch.xml headless:=true
ros2 run hyperdog_navigation navigate_test --ros-args -p use_sim_time:=true -p goal_x:=5.0
```

World `obstacles`: two blocks and a pillar between the start (0, 0) and the goal (5, 0) in a
4.4 m wide corridor. Nav2 plans in the odometry frame with lidar costmaps; MPPI (omnidirectional)
drives the robot through `/cmd_vel`.

| leg odometry | Nav2 result | time (sim) | true final position | error | max tilt |
|---|---|---|---|---|---|
| `foot_rolling: true` (set by `sim_navigation.launch.xml`) | SUCCEEDED | 113.6 s | (4.909, 0.040) | 0.10 m | 7.2 deg |
| `foot_rolling: false` | SUCCEEDED | 120.9 s | (5.233, 0.234) | 0.33 m | 17.9 deg |

**Result: PASS** (tolerance 0.4 m on the true position).
