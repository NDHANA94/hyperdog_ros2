# hyperdog_perception

Terrain perception for HyperDog: a robot-centric 2.5D elevation map built from point clouds
(depth camera, lidar), consumed by the locomotion controller for foothold heights, step edge
avoidance and swing clearance.

| path | content |
|---|---|
| `include/.../elevation_map.hpp`, `src/elevation_map.cpp` | ROS independent rolling elevation grid: per scan cell means, fusion (low-pass / replace on large changes), small-hole inpainting |
| `src/height_map_node.cpp` | point clouds + TF -> map in the odometry frame, self filter, publishes `hyperdog/height_map` (`hyperdog_msgs/HeightMap`) and `hyperdog/height_map/cloud` (RViz) |
| `config/perception.yaml` | inputs, crop box, map size / resolution, fusion |
| `test/test_elevation_map.cpp` | unit tests |

```bash
ros2 launch hyperdog_gazebo sim.launch.xml world:=stairs use_camera:=true perception:=true
```

In `hyperdog_locomotion` (`terrain.*` in `config/locomotion.yaml`) the map:
- sets foothold heights (instead of the plane through the stance feet),
- moves footholds away from step edges (`edge_radius`, `edge_threshold`, `search_radius`),
- raises the swing apex above the highest terrain along the swing path and uses a
  lift - move - lower swing over terrain steps (`terrain_swing`).

The map lives in the odometry frame, so odometry drift between seeing a patch of ground and
stepping on it shifts footholds. `terrain.register_with_feet` (experimental, off by default)
re-aligns the map with the terrain heights measured by the stance feet.
