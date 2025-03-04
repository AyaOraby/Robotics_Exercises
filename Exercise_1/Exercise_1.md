# **📝 Assignment: Custom ROS Publisher & Subscriber for TurtleBot3**

### **Objective**

In this assignment, you will:

1. Create a **custom ROS message** to send commands (e.g., direction and speed) to the TurtleBot3.
2. Write a **publisher node** that takes user input (direction and speed) and publishes it as a custom message.
3. Write a **subscriber node** that listens to the custom message and controls the TurtleBot3 by sending velocity commands to the `/cmd_vel` topic.
4. Test the system in a **Gazebo simulation** and observe the TurtleBot3 moving based on the commands.

---

## **📂 ROS Package Details**

* **Package Name** : `turtlebot3_custom_control`
* **Custom Message Name** : `Command.msg`
* **Topics** :
  * `/turtle_control` (published by your publisher)
  * `/cmd_vel` (published by your subscriber to control TurtleBot3)
* **Node Names** :
  * `turtle_control_publisher` (publisher node)
  * `turtle_control_subscriber` (subscriber node)

---

## **🛠 Prerequisites**

Before starting, make sure you have:

* **ROS Noetic installed** .
* **TurtleBot3 packages installed** :

```bash
sudo apt install ros-noetic-turtlebot3*
```

* **Your ROS workspace set up** (`~/catkin_ws`).

---

## **🚀 Steps to Follow**

### **1️⃣ Create the ROS Package**

* Create a package named `turtlebot3_custom_control`.
* Add the required dependencies.

  ```bash
  cd ~/catkin_ws/src
  catkin_create_pkg turtlebot3_custom_control rospy std_msgs geometry_msgs message_generation message_runtime
  ```

### **2️⃣ Create a Custom Message**

1. Navigate to the package folder:
   ```bash
   cd ~/catkin_ws/src/turtlebot3_custom_control
   ```
2. Create a `msg` folder and define the message:
   ```bash
   mkdir msg
   nano msg/Command.msg
   ```
3. Add the following fields to `Command.msg`:
   ```
   string direction
   float32 speed
   ```
4. Modify `CMakeLists.txt` to include:
   ```cmake
   add_message_files(
     FILES
     Command.msg
   )
   generate_messages(
     DEPENDENCIES
     std_msgs
   )
   ```
5. Modify `package.xml` to include:
   ```xml
   <depend>message_generation</depend>
   <depend>message_runtime</depend>
   ```
6. Build the package:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

### **3️⃣ Implement the Publisher**

1. Create a `scripts` directory and a Python publisher script:
   ```bash
   mkdir scripts
   nano scripts/turtle_control_publisher.py
   ```
2. Add the following code:
   ```python
   #!/usr/bin/env python
   import rospy
   from turtlebot3_custom_control.msg import Command

   def publisher():
       rospy.init_node('turtle_control_publisher', anonymous=True)
       pub = rospy.Publisher('/turtle_control', Command, queue_size=10)
       rate = rospy.Rate(1)
       while not rospy.is_shutdown():
           msg = Command()
           msg.direction = "forward"
           msg.speed = 0.5
           rospy.loginfo("Publishing: direction=%s, speed=%f", msg.direction, msg.speed)
           pub.publish(msg)
           rate.sleep()

   if __name__ == '__main__':
       try:
           publisher()
       except rospy.ROSInterruptException:
           pass
   ```
3. Make the script executable:
   ```bash
   chmod +x scripts/turtle_control_publisher.py
   ```

### **4️⃣ Implement the Subscriber**

1. Create the subscriber script:
   ```bash
   nano scripts/turtle_control_subscriber.py
   ```
2. Add the following code:
   ```python
   #!/usr/bin/env python
   import rospy
   from geometry_msgs.msg import Twist
   from turtlebot3_custom_control.msg import Command

   def callback(msg):
       rospy.loginfo("Received: direction=%s, speed=%f", msg.direction, msg.speed)
       vel_msg = Twist()
       if msg.direction == "forward":
           vel_msg.linear.x = msg.speed
       elif msg.direction == "backward":
           vel_msg.linear.x = -msg.speed
       elif msg.direction == "left":
           vel_msg.angular.z = msg.speed
       elif msg.direction == "right":
           vel_msg.angular.z = -msg.speed
       pub.publish(vel_msg)

   rospy.init_node('turtle_control_subscriber', anonymous=True)
   pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
   rospy.Subscriber('/turtle_control', Command, callback)
   rospy.spin()
   ```
3. Make the script executable:
   ```bash
   chmod +x scripts/turtle_control_subscriber.py
   ```

### **5️⃣ Run the TurtleBot3 Simulation**

1. **Export the TurtleBot3 model**:
   ```bash
   export TURTLEBOT3_MODEL=burger
   ```
2. **Launch the simulation**:
   ```bash
   roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
   ```

### **6️⃣ Run Your Nodes**

1. In one terminal, run:
   ```bash
   rosrun turtlebot3_custom_control turtle_control_publisher.py
   ```
2. In another terminal, run:
   ```bash
   rosrun turtlebot3_custom_control turtle_control_subscriber.py
   ```

### **7️⃣ Verify the Topics**

To check if messages are being published:
```bash
rostopic echo /turtle_control
```

To check if TurtleBot3 is moving:
```bash
rostopic echo /cmd_vel
```

---

## **📌 Submission Requirements**

* A working **ROS package** (.zip).
* A properly defined **custom message**.
* Publisher and subscriber scripts.
* Proof of TurtleBot3 movement (short video).

---

## **README File**

**Title: TurtleBot3 Custom Control**

**Description:** This package implements a custom ROS publisher and subscriber to control a TurtleBot3 in Gazebo simulation using a custom message.

**Installation & Setup:**
```bash
cd ~/catkin_ws/src
catkin_create_pkg turtlebot3_custom_control rospy std_msgs geometry_msgs message_generation message_runtime
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

**Running the Simulation & Nodes:**
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
rosrun turtlebot3_custom_control turtle_control_publisher.py
rosrun turtlebot3_custom_control turtle_control_subscriber.py
```

**Verification Commands:**
```bash
rostopic echo /turtle_control
rostopic echo /cmd_vel
```

