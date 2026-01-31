# Warehouse Robot - Mapping & Localization

This ROS2 package implements a mapping and localization pipeline for a simulated differential-drive robot in a warehouse environment.

## Phase A: Mapping
We use **slam_toolbox** for generating a 2D occupancy grid map.
- **Why slam_toolbox?**: It is the standard, well-maintained SLAM solution for ROS2. It offers efficient online asynchronous mapping and is robust for indoor environments compared to GMapping (deprecated) or Cartographer (complex setup).

### How to Run Mapping
1. Build the workspace:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```
2. Launch the simulation and SLAM:
   ```bash
   ros2 launch warehouse_robot mapping.launch.py
   ```
3. Drive the robot using teleop (in a new terminal):
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
4. Visualize in RViz (should be auto-launched).
5. Save the map:
   In the RViz "SlamToolbox" panel, click "Save Map" or run:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f src/warehouse_robot/maps/warehouse_map
   ```

## Phase B: Localization
We use **AMCL (Adaptive Monte Carlo Localization)** with a pre-built map.

### How to Run Localization
1. Ensure `maps/warehouse_map.yaml` and `maps/warehouse_map.pgm` exist.
2. Launch localization:
   ```bash
   ros2 launch warehouse_robot localization.launch.py map:=src/warehouse_robot/maps/warehouse_map.yaml
   ```
3. In RViz:
   - Use "2D Pose Estimate" to initialize the robot's position.
   - Drive the robot to see particles converge (AMCL cloud).

## Key Parameters Tuned
1. **`slam_toolbox` -> `max_laser_range` (20.0)**: Increased to ensure the lidar sees the distant warehouse walls, improving loop closure.
2. **`amcl` -> `max_particles` (2000)**: Increased from default to handle the large open spaces in the warehouse for better global localization.
3. **`amcl` -> `update_min_d` (0.25) / `update_min_a` (0.2)**: Tuned to update filter more frequently during motion, ensuring the pose estimate doesn't drift too far between updates.

## Troubleshooting / What Broke
- **Issue**: Lidar data wasn't showing in RViz initially.
- **Fix**: The `robot_state_publisher` wasn't publishing TF correctly because the `joint_state_publisher` was missing for non-fixed joints. However, since we use a plugin for wheel odometry and static joints for Lidar, ensuring the `robot_description` was loaded correctly and `use_sim_time` was true fixed the TF tree synchronization.
- **Issue**: Map not appearing in Localization mode.
- **Fix**: The `lifecycle_manager` was required to transition `map_server` and `amcl` to the "Active" state. Simply running the nodes wasn't enough. Added `lifecycle_manager` to the launch file.

## Dependencies
Ensure you have the following installed:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup
```

- `slam_toolbox`
- `nav2_map_server`, `nav2_amcl`, `nav2_lifecycle_manager`
- `gazebo_ros_pkgs`
- `xacro`
