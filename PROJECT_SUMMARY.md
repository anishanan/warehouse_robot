# Warehouse Robot Project Summary

## 1. SLAM Package Choice
**Selected:** `slam_toolbox` (Asynchronous Mode)

**Reasoning:**
- **Modern Standard:** Unlike GMapping (which is largely deprecated) or Cartographer (which is complex to configure), `slam_toolbox` is the officially supported and most stable SLAM solution for ROS2 Humble.
- **Large Scale Performance:** It excels in large environments like warehouses by using a pose-graph optimization technique that handles loop closures much more reliably.
- **Lidar Optimization:** It is highly optimized for 2D LiDAR data, allowing for smooth map generation even with a moving robot.

---

## 2. Key Parameters Tuned
To make the robot work perfectly in the warehouse simulation, I tuned these 3 parameters:

1.  **`max_laser_range` (Set to 20.0m):**
    - *Why:* The default range was too short to "see" the distant warehouse walls. Increasing this allowed the SLAM algorithm to use more landmarks for mapping.
2.  **`max_particles` (Set to 2000):**
    - *Why:* In localization (AMCL), the default particle count was too low for a large, symmetrical warehouse. More particles help the robot "re-localize" its position faster if it gets lost.
3.  **`update_min_d` (Set to 0.25m):**
    - *Why:* This controls how often the localization filter updates based on distance traveled. 0.25m provides a great balance between accuracy and CPU performance.

---

## 3. Initial Issues & Solutions (What Broke?)

### **Issue 1: LiDAR Data Missing / TF Tree Broken**
- **Symptom:** Rviz showed "No transform from [laser_frame] to [map]". The LiDAR points were not appearing.
- **Cause:** The `use_sim_time` parameter was set to `false` (default). In Gazebo, the clock runs differently than your system clock. If they don't match, ROS2 ignores the sensor data.
- **Fix:** I updated the `simulation.launch.py` to hardcode `use_sim_time: True` for the `robot_state_publisher` and sensor nodes.

### **Issue 2: Map Not Loading in Localization**
- **Symptom:** The `map_server` was running, but the map wasn't showing up in Rviz.
- **Cause:** ROS2 Navigation2 uses "Lifecycle Nodes". A node like `map_server` starts in an "unconfigured" state and stays inactive until a `lifecycle_manager` tells it to start.
- **Fix:** I added a `lifecycle_manager` node to the `localization.launch.py` and included the `map_server` in its node list to automatically transition it to the "Active" state.

---

## 📁 Repository Structure
- `launch/`: Contains simulation, mapping, and localization start scripts.
- `config/`: Contains tuned YAML parameters and Rviz configurations.
- `urdf/`: Contains the robot's physical description (Xacro).
- `maps/`: Contains the saved warehouse occupancy grid.
