# TurtleBot3 Simulation with GMapping SLAM

## 1. Set Up the Workspace and Clone the Repository

### Navigate to `catkin_ws/src`
```bash
cd ~/catkin_ws/src
```
### Clone the TurtleBot3 Simulations Repository
```bash
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

## 2. Modify the Launch File

### Open `turtlebot3_world.launch` for Editing
```bash
nano turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_world.launch
```

### Add the Following Lines Before `</launch>`
```xml
<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="false" output="screen"/>

<!-- Send fake joint values -->
<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
  <param name="use_gui" value="true"/>
</node>
```

## 3. Build the Workspace
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 4. Set the TurtleBot3 Model
```bash
export TURTLEBOT3_MODEL=waffle
```

## 5. Launch the Gazebo Simulation
```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

## 6. Install `gmapping`
```bash
sudo apt-get install ros-noetic-slam-gmapping
```

# SLAM Algorithms
SLAM algorithms include:
- **Extended Kalman Filter (EKF)**
- **GraphSLAM**
- **FastSLAM**

We focus on **FastSLAM**, which uses a particle filter with a low-dimensional EKF for efficiency.

## What is GMapping?
GMapping implements **Grid-Based FastSLAM**, enabling 2D map generation using LiDAR and odometry data.

### **Particle Filter**
- Estimates robot pose using particles (x, y, θ).
- Propagates, reweights, and resamples particles based on sensor data.

### **Loop Closure**
- Detects previously visited locations using **scan matching**.
- Adjusts trajectory to improve mapping accuracy.

## ROS Integration with GMapping SLAM
### Input:
- **Laser scans (`/scan` topic)**
- **Transformations (`/tf` topic)**

### Output:
- **Occupancy grid map (`/map`, `/map_metadata` topics)**
- **Estimated position (`/tf` topic)**

## Installation
```bash
sudo apt-get install ros-noetic-gmapping
```

## Configuring GMapping Launch File

### Create ROS Package
```bash
cd ~/catkin_ws/src
catkin_create_pkg slam_gmapping
cd slam_gmapping
```

### Create Launch File
```bash
mkdir launch
cd launch
nano slam_gmapping.launch
```

### Add the Following Content to `slam_gmapping.launch`
```xml
<launch>
    <param name="use_sim_time" value="true"/>
    <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
        <remap from="scan" to="/scan"/>
        <param name="base_frame" value="base_footprint"/>
        <param name="odom_frame" value="odom"/>
        <param name="map_frame" value="map"/>
        <param name="map_update_interval" value="2.0"/>
        <param name="particles" value="100"/>
        <param name="xmin" value="-10.0"/>
        <param name="ymin" value="-10.0"/>
        <param name="xmax" value="10.0"/>
        <param name="ymax" value="10.0"/>
        <param name="delta" value="0.05"/>
    </node>
</launch>
```

## Running GMapping

### Build and Launch GMapping
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch slam_gmapping slam_gmapping.launch
```

### Run RViz
1. Set `Fixed Frame` to `map`
2. Add `Map` and `LaserScan`
3. Choose topics:
   - `LaserScan --> /scan`
   - `Map --> /map`

### Move the Robot
```bash
source devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## Save The Map

### Install `map_server`
```bash
sudo apt update
sudo apt install ros-noetic-map-server
```

### Save the Map
```bash
source devel/setup.bash
rosrun map_server map_saver -f ~/catkin_ws/src/maps/mymap
```

### Map Files Generated
1. **PGM File (`mymap.pgm`)** - Stores occupancy grid map.
2. **YAML File (`mymap.yaml`)** - Contains metadata.

### Example `mymap.yaml`
```yaml
image: maps/mymap.pgm
resolution: 0.050000
origin: [-10.000000, -10.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

## Summary
This guide covers setting up **TurtleBot3 simulation**, running **GMapping SLAM**, and **saving the map** for further use in navigation and path planning.

