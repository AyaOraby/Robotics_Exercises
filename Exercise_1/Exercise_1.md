# **📝 Exercise 1: Custom ROS Publisher & Subscriber**

### **Objective**

In this assignment, you will:

1. Create a **custom ROS message** to send two integers.
2. Write a **publisher node** that sends two integers at a fixed rate.
3. Write a **subscriber node** that listens to the custom message, sums the integers, and prints the result.
4. Create a **launch file** to automate the process.

---

## **📂 ROS Package Details**

* **Package Name** : `custom_msg_example`
* **Custom Message Name** : `TwoInts.msg`
* **Topics** :
  * `/sum` (published by the publisher node)
* **Node Names** :
  * `two_ints_publisher` (publisher node)
  * `two_ints_subscriber` (subscriber node)

---

## **🛠 Prerequisites**

Before starting, make sure you have:

* **ROS Noetic installed**
* **Your ROS workspace set up** (`~/catkin_ws`)

---

## **🚀 Steps to Follow**

### **1️⃣ Create the ROS Package**

```bash
cd ~/catkin_ws/src
catkin_create_pkg custom_msg_example rospy std_msgs message_generation message_runtime
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### **2️⃣ Create a Custom Message**

1. Navigate to the package folder:

```bash
cd ~/catkin_ws/src/custom_msg_example
```

2. Create a `msg` folder and define the message:

```bash
mkdir msg
nano msg/TwoInts.msg
```

3. Add the following fields to `TwoInts.msg`:

```bash
int32 a
int32 b
```

4. Modify `CMakeLists.txt` to include:

```cmake
add_message_files(
  FILES
  TwoInts.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_install_python(PROGRAMS
  scripts/two_ints_publisher.py
  scripts/two_ints_subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
nano scripts/two_ints_publisher.py
```

2. Add the following code:

```python
#!/usr/bin/env python3
import rospy
from custom_msg_example.msg import TwoInts

def send_two_ints():
    rospy.init_node('send_two_ints_node', anonymous=True)
    pub = rospy.Publisher('sum', TwoInts, queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg = TwoInts()
        msg.a = 5
        msg.b = 10
        rospy.loginfo("Publishing: a = %d, b = %d", msg.a, msg.b)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_two_ints()
    except rospy.ROSInterruptException:
        pass
```

3. Make the script executable:

```bash
chmod +x scripts/two_ints_publisher.py
```

### **4️⃣ Implement the Subscriber**

1. Create the subscriber script:

```bash
nano scripts/two_ints_subscriber.py
```

2. Add the following code:

```python
#!/usr/bin/env python3
import rospy
from custom_msg_example.msg import TwoInts

def callback(msg):
    sum_result = msg.a + msg.b
    rospy.loginfo("Received: a = %d, b = %d, Sum = %d", msg.a, msg.b, sum_result)

def subscriber():
    rospy.init_node('two_ints_subscriber', anonymous=True)
    rospy.Subscriber('sum', TwoInts, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()
```

3. Make the script executable:

```bash
chmod +x scripts/two_ints_subscriber.py
```

### **5️⃣ Run the System**

1. **Start ROS Master**:

```bash
roscore
```

2. **Run the Publisher Node**:

```bash
rosrun custom_msg_example two_ints_publisher.py
```

3. **Run the Subscriber in another terminal**:

```bash
rosrun custom_msg_example two_ints_subscriber.py
```

### **6️⃣ Create a Launch File**

1. **Create a `launch` Folder**:

```bash
cd ~/catkin_ws/src/custom_msg_example
mkdir launch
```

2. **Create a Launch File**:

```bash
nano launch/two_ints.launch
```

3. **Add the following content**:

```xml
<launch>
    <node pkg="custom_msg_example" type="two_ints_publisher.py" name="two_ints_publisher" output="screen" />
    <node pkg="custom_msg_example" type="two_ints_subscriber.py" name="two_ints_subscriber" output="screen" />
</launch>
```

4. **Build the Workspace**:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

5. **Run the Launch File**:

```bash
roslaunch custom_msg_example two_ints.launch
```

This will:

1. Start the ROS Master (if not already running).
2. Launch the `two_ints_publisher` and `two_ints_subscriber` nodes.
3. Display the output of both nodes in the terminal.

---


