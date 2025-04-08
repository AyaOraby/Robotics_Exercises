# TurtleBot3 Simulation and SLAM in ROS

## **1. Set Up the Workspace and Clone the Repository**

### **1.1 Navigate to Your `catkin_ws/src` Directory**
```bash
cd ~/catkin_ws/src
```

### **1.2 Clone the TurtleBot3 Simulations Repository**
```bash
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

## **2. Modify the Launch File**

### **2.1 Open `turtlebot3_world.launch` for Editing**
```bash
nano turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_world.launch
```

### **2.2 Add the Following Lines Before the Closing `</launch>` Tag**
```xml
<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="false" output="screen"/>

<!-- Send fake joint values -->
<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
  <param name="use_gui" value="true"/>
</node>
```
Save and exit the editor (`Ctrl+O`, `Enter`, `Ctrl+X`).

## **3. Build and Source the Workspace**
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## **4. Set the TurtleBot3 Model**
Set the TurtleBot3 model to `waffle` (or `burger` if preferred):
```bash
export TURTLEBOT3_MODEL=waffle
```

## **5. Launch the Gazebo Simulation**
Launch the TurtleBot3 world in Gazebo:
```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

## **6. Install the `gmapping` Package for SLAM**
If not installed, install it using `apt`:
```bash
sudo apt-get install ros-noetic-slam-gmapping
```

## **7. Run GMapping for SLAM**
```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

## **8. Open RViz for Visualization**
1. Set `Fixed Frame` to `map`.
2. Add the following topics:
   - **LaserScan** → `/scan`
   - **Map** → `/map`

## **9. Move the Robot Using Teleoperation**

### **9.1 Run the Teleoperation Node**
```bash
source devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### **9.2 Keyboard Controls**
```
   u    i    o
   j    k    l
   m    ,    .
```
- `i` → Move forward
- `,` → Move backward
- `j` → Turn left
- `l` → Turn right
- `k` → Stop

## **10. Save the Generated Map**
After mapping, save the generated map using `map_server`:

### **10.1 Install `map_server`**
```bash
sudo apt-get install ros-noetic-map-server
```

### **10.2 Save the Map**
```bash
rosrun map_server map_saver -f ~/catkin_ws/src/maps/my_map
```
- This creates `my_map.pgm` and `my_map.yaml` files in the `maps` directory.



## **11. Conclusion**
This guide covers setting up the TurtleBot3 simulation, modifying launch files, running SLAM with GMapping, teleoperating the robot via the keyboard, visualizing in RViz, and saving the generated map. 

