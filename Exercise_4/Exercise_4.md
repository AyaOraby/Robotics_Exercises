# **📷 Adding a Camera to Your Robot in ROS**

## **Objective**

In this exercise, you will:

1. Create a `camera_link` in the `myrobot.xacro` file.
2. Use an STL, STEP, or DAE file for the camera in the `meshes` folder.
3. Add an RGB camera plugin in the `.gazebo` file.
4. Visualize the camera data in **RViz** and take a screenshot.

---

## **📂 Directory Structure**

You will create the following directory structure:

```
lab4/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── myrobot_gazebo.launch
├── robot_description/
│   ├── meshes/
│   │   ├── camera_model.stl
│   ├── urdf/
│   │   ├── myrobot.xacro
│   │   ├── myrobot.gazebo
└── src/
```

---

## **🚀 Steps to Follow**

### **1️⃣ Create the ROS Package**

Navigate to your workspace and create a new package:

```bash
cd ~/catkin_ws/src
catkin_create_pkg lab4 std_msgs rospy roscpp
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

### **2️⃣ Create Required Folders**

Move into the package directory and create necessary subdirectories:

```bash
cd ~/catkin_ws/src/lab4
mkdir -p launch robot_description/meshes robot_description/urdf
```

---

### **3️⃣ Add the Camera Mesh File**

Copy your camera model file to the `meshes` folder:

```bash
cp /home/philomath/Desktop/camera_model.stl ~/catkin_ws/src/lab4/robot_description/meshes/
```

---

### **4️⃣ Create the `myrobot.xacro` File**

Create the `myrobot.xacro` file inside `robot_description/urdf`:

```bash
touch ~/catkin_ws/src/lab4/robot_description/urdf/myrobot.xacro
```

Edit the file and add the following content:

```xml
<?xml version="1.0"?>
<robot name="myrobot" xmlns:xacro="http://ros.org/wiki/xacro">

    <!-- Camera Link -->
    <link name="camera_link">
        <visual>
            <geometry>
                <mesh filename="package://lab4/robot_description/meshes/camera_model.stl" scale="0.1 0.1 0.1"/>
            </geometry>
        </visual>
        <collision>
            <geometry>
                <box size="0.1 0.1 0.1"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="0.5"/>
            <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
        </inertial>
    </link>

    <!-- Joint between base_link and camera_link -->
    <joint name="camera_joint" type="fixed">
        <parent link="base_link"/>
        <child link="camera_link"/>
        <origin xyz="0.0 0.0 0.5"/>
    </joint>

</robot>
```

---

### **5️⃣ Create the Gazebo Plugin File**

Create the `myrobot.gazebo` file:

```bash
touch ~/catkin_ws/src/lab4/robot_description/urdf/myrobot.gazebo
```

Edit it and add the following content:

```xml
<gazebo>
    <sensor type="camera" name="camera_sensor">
        <update_rate>30</update_rate>
        <camera>
            <horizontal_fov>1.396</horizontal_fov>
            <image>
                <width>640</width>
                <height>480</height>
                <format>R8G8B8</format>
            </image>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
            <always_on>true</always_on>
            <update_rate>30.0</update_rate>
            <camera_name>/camera</camera_name>
            <image_topic_name>/camera/image_raw</image_topic_name>
            <frame_name>camera_link</frame_name>
        </plugin>
    </sensor>
</gazebo>
```

---

### **6️⃣ Create the Launch File**

Create the `myrobot_gazebo.launch` file:

```bash
touch ~/catkin_ws/src/lab4/launch/myrobot_gazebo.launch
```

Edit it and add the following content:

```xml
<launch>
    <param name="robot_description" command="cat $(find lab4)/robot_description/urdf/myrobot.xacro"/>
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" output="screen"/>
    <node name="gazebo" pkg="gazebo_ros" type="gzserver" args="--verbose" output="screen"/>
    <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-param robot_description -urdf -model myrobot"/>
</launch>
```

---

### **7️⃣ Build and Run the Robot in Gazebo**

Go back to the workspace and rebuild:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Now, launch Gazebo with your robot:

```bash
roslaunch lab4 myrobot_gazebo.launch
```

---

### **8️⃣ Visualize in RViz**

Open a new terminal and run RViz:

```bash
rviz
```

1. In **RViz**, add a new **Image** display.
2. Set the topic to `/camera/image_raw`.
3. You should see the camera feed.

Take a screenshot and submit it as proof of completion.

---

## **✅ Submission Requirements**

1. **Updated `myrobot.xacro`** file with the `camera_link` and joint.
2. **Camera mesh file** (`camera_model.stl`) in the `meshes` folder.
3. **Updated `myrobot.gazebo`** file with the RGB camera plugin.
4. **Screenshot of camera data in RViz** (`/camera/image_raw`).

