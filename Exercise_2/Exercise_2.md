# 🚀 Exercise 2: Publisher and Subscriber C++ Example

## 📌 Overview
This exercise demonstrates how to create a **ROS Publisher and Subscriber** in C++ using a custom ROS package. We will:
1. **Create a ROS package** named `cpp_talker_listener`.
2. **Implement a Publisher node (`talker.cpp`)** that sends messages.
3. **Implement a Subscriber node (`listener.cpp`)** that receives messages.
4. **Configure `CMakeLists.txt`** to compile both nodes.
5. **Build and launch the nodes** in ROS.

---

## 📂 Project Structure
```
cpp_talker_listener/
│── src/
│   ├── talker.cpp  # Publisher node
│   ├── listener.cpp  # Subscriber node
│── CMakeLists.txt  # Configures compilation
│── package.xml  # Defines dependencies
│── launch/
│   ├── talker_listener.launch  # Launch file to run both nodes
```

---

## 🔧 Step 1: Create a ROS Package

Navigate to your workspace and create a new ROS package:
```bash
cd ~/catkin_ws/src
catkin_create_pkg cpp_talker_listener roscpp std_msgs
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 📡 Step 2: Implement the Talker (Publisher) Node

1. Navigate to the package directory and create the `src` folder:
   ```bash
   cd ~/catkin_ws/src/cpp_talker_listener
   mkdir -p src
   ```
2. Create the `talker.cpp` file:
   ```bash
   nano src/talker.cpp
   ```
3. Add the following code to `talker.cpp`:

   ```cpp
   #include "ros/ros.h"
   #include "std_msgs/String.h"
   #include <sstream>

   int main(int argc, char **argv) {
       ros::init(argc, argv, "talker");
       ros::NodeHandle nh;
       ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
       ros::Rate loop_rate(10);
       int count = 0;

       while (ros::ok()) {
           std_msgs::String msg;
           std::stringstream ss;
           ss << "Hello World " << count;
           msg.data = ss.str();
           ROS_INFO("Publishing: %s", msg.data.c_str());
           chatter_pub.publish(msg);
           ros::spinOnce();
           loop_rate.sleep();
           ++count;
       }
       return 0;
   }
   ```

---

## 🎯 Step 3: Implement the Listener (Subscriber) Node

1. Create the `listener.cpp` file:
   ```bash
   nano src/listener.cpp
   ```
2. Add the following code:

   ```cpp
   #include "ros/ros.h"
   #include "std_msgs/String.h"

   void chatterCallback(const std_msgs::String::ConstPtr& msg) {
       ROS_INFO("I heard: %s", msg->data.c_str());
   }

   int main(int argc, char **argv) {
       ros::init(argc, argv, "listener");
       ros::NodeHandle nh;
       ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);
       ros::spin();
       return 0;
   }
   ```

---

## 🛠️ Step 4: Update `CMakeLists.txt`

1. Open the `CMakeLists.txt` file:
   ```bash
   nano CMakeLists.txt
   ```
2. Add the following lines:
   ```cmake
   add_executable(talker src/talker.cpp)
   target_link_libraries(talker ${catkin_LIBRARIES})

   add_executable(listener src/listener.cpp)
   target_link_libraries(listener ${catkin_LIBRARIES})
   ```

---

## 🔨 Step 5: Build the Package

1. Compile the package:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

---

## 🚀 Step 6: Create a Launch File

1. Create a `launch` directory:
   ```bash
   cd ~/catkin_ws/src/cpp_talker_listener
   mkdir launch
   ```
2. Create a launch file:
   ```bash
   nano launch/talker_listener.launch
   ```
3. Add the following content:
   ```xml
   <launch>
       <node pkg="cpp_talker_listener" type="talker" name="talker" output="screen" />
       <node pkg="cpp_talker_listener" type="listener" name="listener" output="screen" />
   </launch>
   ```

---

## ▶️ Step 7: Run the Nodes

1. Start the ROS core:
   ```bash
   roscore
   ```
2. In a new terminal, run the launch file:
   ```bash
   roslaunch cpp_talker_listener talker_listener.launch
   ```

🔹 **Expected Output:**
```plaintext
[INFO] [WallTime: ...] Publishing: Hello World 0
[INFO] [WallTime: ...] I heard: Hello World 0
[INFO] [WallTime: ...] Publishing: Hello World 1
[INFO] [WallTime: ...] I heard: Hello World 1
```

---



## 🎯 Summary
- We created a **ROS package**.
- Implemented **Publisher (`talker.cpp`)** and **Subscriber (`listener.cpp`)** nodes.
- Updated **`CMakeLists.txt`** to compile the nodes.
- Built the package and **ran it using a launch file**.

Now you have a fully working ROS Publisher-Subscriber system in C++! 🚀

