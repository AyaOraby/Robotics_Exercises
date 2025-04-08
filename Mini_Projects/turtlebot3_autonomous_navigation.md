
# TurtleBot3 Autonomous Navigation with Obstacle Avoidance

This project demonstrates how to make TurtleBot3 navigate autonomously in a Gazebo simulation while avoiding obstacles based on laser scan readings. The robot moves randomly in the environment but will back up and rotate when it detects an obstacle or is stuck. The system uses ROS with Python and C++ for movement control and sensor data handling.

## Prerequisites

Before you start, ensure you have the following:

1. **Ubuntu OS (20.04 or later)** with ROS Noetic installed.
2. **TurtleBot3** package installed (ROS package for simulation).
3. **Gazebo** for simulation.
4. **catkin workspace** set up.

If you don't have these prerequisites installed, you can follow the [official ROS installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu) and the [TurtleBot3 installation instructions](https://emanual.robotis.com/docs/en/platform/turtlebot3/installation/).

---

## Step-by-Step Instructions

### 1. Set Up Your Catkin Workspace

First, create and initialize a catkin workspace for your project.

```bash
# Create the workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Initialize the workspace
catkin_init_workspace

# Build the workspace
cd ..
catkin_make
source devel/setup.bash
```

### 2. Install the TurtleBot3 Simulation Packages

To interact with the TurtleBot3 simulation in Gazebo, you need to install the necessary TurtleBot3 packages.

```bash
# Install TurtleBot3 simulation packages
sudo apt install ros-noetic-turtlebot3-gazebo
sudo apt install ros-noetic-turtlebot3-simulation
```

### 3. Create Your ROS Package

In the `src` directory of your catkin workspace, create a new package that will include the Python node for movement and obstacle avoidance.

```bash
# Navigate to the src folder
cd ~/catkin_ws/src

# Create a new ROS package
catkin_create_pkg turtlebot3_autonomous_move rospy geometry_msgs sensor_msgs random

# Navigate into the package directory
cd turtlebot3_autonomous_move
```

### 4. Write the Movement and Avoidance Code

Create a Python script to control the robot's movement and handle obstacle avoidance. Inside the `turtlebot3_autonomous_move` package, create a new file `random_move.py`:

```bash
# Navigate to the scripts directory
mkdir scripts
cd scripts

# Create the Python script
touch random_move.py
```

Now, open `random_move.py` and add the following code:

```python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random

# Initialize global variables
visited_positions = set()  # To store visited positions
current_position = None  # Current position of the robot
close_to_wall = False  # Flag to check if near wall
stuck_count = 0  # Counter to detect when robot is stuck

# Callback function for laser scan data
def laser_callback(scan_data):
    global close_to_wall, stuck_count
    min_range = min(scan_data.ranges)  # Get the closest obstacle distance
    
    # If the robot is close to any obstacle or wall
    if min_range < 0.5:  
        close_to_wall = True
        stuck_count += 1  # Increment stuck counter
    else:
        close_to_wall = False
        stuck_count = 0  # Reset stuck counter if no obstacle is near

# Function to stop or turn the robot when obstacles are detected
def stop_or_turn():
    move_cmd = Twist()
    move_cmd.linear.x = 0
    move_cmd.angular.z = random.choice([2.0, -2.0])  # Increase rotation speed for faster turns
    cmd_vel_pub.publish(move_cmd)

# Function to generate random direction for the robot
def move_random():
    move_cmd = Twist()
    move_cmd.linear.x = random.uniform(0.5, 1.0)  # Increase speed (higher max speed)
    move_cmd.angular.z = random.uniform(-2.0, 2.0)  # Increase rotation speed for faster turns
    cmd_vel_pub.publish(move_cmd)

# Function to handle back-up and avoiding walls with a rotation
def avoid_walls():
    move_cmd = Twist()

    # If close to wall, back up and rotate
    if close_to_wall:
        rospy.loginfo("Near wall! Backing up and rotating...")

        # Move backward quickly
        move_cmd.linear.x = -0.5  # Increase speed for backing up
        move_cmd.angular.z = random.choice([2.0, -2.0])  # Rotate to change direction

        cmd_vel_pub.publish(move_cmd)
        rospy.sleep(1)  # Allow some time for back-up and turning

        # After moving backward and rotating, try random movement
        move_random()
    else:
        move_random()  # Otherwise, move randomly if no obstacle detected

# Function to escape from corners (robot stuck in tight places)
def escape_corner():
    move_cmd = Twist()

    rospy.loginfo("Trying to escape from a corner...")

    # Back up further to create more space
    move_cmd.linear.x = -0.5
    move_cmd.angular.z = random.choice([2.0, -2.0])  # Random rotation to change direction
    cmd_vel_pub.publish(move_cmd)

    rospy.sleep(1)  # Allow time for escape

    # Rotate in different directions to find a clearer path
    for _ in range(4):
        move_cmd.linear.x = 0  # Stop moving forward
        move_cmd.angular.z = random.choice([2.0, -2.0])  # Rotate in a random direction
        cmd_vel_pub.publish(move_cmd)
        rospy.sleep(1)  # Time for rotation

    # After multiple rotations, move forward in a random direction
    move_random()

# Function to check if the robot is stuck and needs special handling
def handle_stuck():
    global stuck_count

    # If stuck for more than a few cycles (e.g., 3 cycles), try to escape
    if stuck_count > 3:
        rospy.loginfo("Robot seems stuck! Trying to escape...")
        escape_corner()
        stuck_count = 0  # Reset stuck count after attempting escape
    else:
        avoid_walls()  # Continue avoiding walls in normal mode

# Function to update the robot's position (basic example)
def update_position():
    global current_position
    # Logic to update position could be added here
    # For now, we'll assume the robot is always moving forward
    current_position = (random.uniform(-5, 5), random.uniform(-5, 5))  # Example positions
    if current_position not in visited_positions:
        visited_positions.add(current_position)
        move_random()  # Move the robot randomly if not visited
    else:
        stop_or_turn()  # If revisiting, stop and turn instead

# ROS Node Setup
rospy.init_node('turtlebot3_random_move', anonymous=True)

# Publishers and Subscribers
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # Publisher for robot movement
laser_sub = rospy.Subscriber('/scan', LaserScan, laser_callback)  # Subscriber for laser data

# Main loop
rate = rospy.Rate(10)  # 10 Hz
while not rospy.is_shutdown():
    handle_stuck()  # Check if robot is stuck and handle it accordingly
    update_position()  # Update position and decide movement
    rate.sleep()

```

### 5. Make the Script Executable

After creating the script, make it executable:

```bash
chmod +x random_move.py
```

### 6. Launch the TurtleBot3 Simulation in Gazebo

To launch the TurtleBot3 world in Gazebo, use the following command:

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

This will launch the TurtleBot3 simulation in an empty world. You can modify the world later to add obstacles and more features.

### 7. Run the Movement Script

In another terminal, source the workspace and run the Python script to control the robot:

```bash
source ~/catkin_ws/devel/setup.bash
rosrun turtlebot3_autonomous_move random_move.py
```

The robot will start moving autonomously, avoiding obstacles and performing random movement.

### 8. Adjust Robot Behavior

You can adjust the robot's behavior by modifying the script:

- **Speed**: Modify `move_cmd.linear.x` to change the robot's speed.
- **Obstacle Distance**: Adjust the `min_range < 0.5` value in `laser_callback()` to make the robot more or less sensitive to obstacles.

### 9. Test and Debug

- If the robot is getting stuck in certain places, consider adjusting the movement speed or how quickly it reacts to obstacles.
- You can also modify the world in Gazebo by adding more obstacles to make the robot's path more complex.

---

## Conclusion

This project demonstrates how to make TurtleBot3 move autonomously while avoiding obstacles in Gazebo using ROS, Python, and the laser scan data. You can further expand this by adding more sophisticated navigation algorithms, SLAM, or path planning techniques.

---

## Troubleshooting

- **Robot not moving**: Ensure that your ROS master is running and the simulation is launched.
- **Script not working**: Check the ROS logs for any errors and ensure all dependencies are correctly installed.

---

### License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.
