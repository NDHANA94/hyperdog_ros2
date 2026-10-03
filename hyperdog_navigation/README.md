# hyperdog_navigation

Nav2 configuration for HyperDog.

| path | content |
|---|---|
| `config/nav2.yaml` | Nav2 parameters: navigation in the odometry frame (no map / localization), rolling local and global costmaps from the 3D lidar, NavFn (A*) planner, MPPI controller with the omnidirectional motion model, velocity smoother |
| `launch/navigation.launch.xml` | controller, planner, behaviors, BT navigator, velocity smoother, lifecycle manager; velocity commands go `cmd_vel_nav` -> velocity smoother -> `cmd_vel` |
| `launch/sim_navigation.launch.xml` | Gazebo (`obstacles` world, lidar) + locomotion + Nav2 |
| `src/navigate_test.cpp` | sends a `NavigateToPose` goal and grades it with the simulator ground truth |

```bash
ros2 launch hyperdog_navigation sim_navigation.launch.xml headless:=true
ros2 run hyperdog_navigation navigate_test --ros-args -p use_sim_time:=true -p goal_x:=5.0
# or any NavigateToPose client / RViz "Nav2 Goal" with the fixed frame `odom`
```

For navigation in a map, add localization (e.g. `slam_toolbox` or AMCL with a map) and set the
`global_frame` of `bt_navigator`, `behavior_server` and `global_costmap` to `map`.
