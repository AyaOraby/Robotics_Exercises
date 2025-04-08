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

