# Warehouse Robot - Mapping & Localization

This ROS2 package implements a mapping and localization pipeline for a simulated differential-drive robot in a warehouse environment.

## 🚀 Why Slam Toolbox?
We chose **`slam_toolbox`** (Asynchronous mode) for this project over GMapping or Cartographer. 
- **Reliability**: It is the modern standard for ROS2, offering superior stability in large-scale indoor environments like warehouses.
- **Efficiency**: It handles loop closure more effectively without heavy CPU overhead.
- **Flexibility**: It allows for online mapping and manual map manipulation, which is critical for dynamic industrial settings.

## ⚙️ 3 Key Parameters Tuned
To achieve high-quality mapping and localization, we tuned the following:
1.  **`max_laser_range` (20.0)**: Increased from default to allow the LiDAR to detect far-away warehouse walls. This significantly improved loop closure and geometric consistency of the map.
2.  **`max_particles` (2000)**: Increased the particle limit in AMCL. This helped the robot recover its pose more quickly in the large, repetitive open spaces of the warehouse.
3.  **`update_min_d` (0.25)**: Adjusted the minimum distance for filter updates. This ensured the localization remained sharp during movement without overwhelming the processor with unnecessary updates.

## 🛠️ Troubleshooting: What Broke & How We Fixed It
- **The Issue**: Initially, the LiDAR data was not visible in Rviz, and the robot’s Transformation Tree (TF) was "broken" or jumping.
- **The Solution**: We discovered two things:
    1.  `use_sim_time` was not set to `True` for all nodes, causing a time-sync mismatch between Gazebo and ROS2.
    2.  The `robot_description` wasn't being fully parsed by the `robot_state_publisher`. 
- **The Fix**: We updated the launch file to pass `use_sim_time:=True` globally and ensured the Xacro file correctly defined the LiDAR link relative to the `base_link`. 

---

## 💻 How to Run

### 1. Build and Source
```bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Run Mapping
```bash
ros2 launch warehouse_robot mapping.launch.py
```

### 3. Run Localization
*(Ensure you have a saved map in the `maps/` folder first)*
```bash
ros2 launch warehouse_robot localization.launch.py map:=src/warehouse_robot/maps/warehouse_map.yaml
```

## 📦 Dependencies
- `ros-humble-gazebo-ros-pkgs`
- `ros-humble-slam-toolbox`
- `ros-humble-navigation2`
- `ros-humble-nav2-bringup`
