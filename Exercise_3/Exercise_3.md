# 🚀 Creating a Xacro File 

## 📌 Overview
In this task we will create a **Xacro file** for defining your robot model in ROS. Xacro (XML Macros) helps simplify URDF files by using macros and parameters. We will:

1. **Create a ROS package** for the robot.
2. **Define a Xacro file** with base links, wheels, and joints.
3. **Explain the key components** of the Xacro file.

---

## 📂 Project Structure
```
my_robot/
│── urdf/
│   ├── myrobot.xacro  # Xacro file defining the robot
│── CMakeLists.txt  # ROS build configuration
│── package.xml  # Defines dependencies
```

---

## 🔧 Setting Up the Package
### 1️⃣ Create a ROS Package
To keep things organized, first, create a ROS package:
```bash
cd ~/catkin_ws/src
catkin_create_pkg my_robot
cd ~/catkin_ws
catkin_make
```

### 2️⃣ Create a Directory for URDF/Xacro Files
Inside your package, create a directory to store your URDF/Xacro files:
```bash
cd ~/catkin_ws/src/my_robot
mkdir urdf
```

---

## 📝 Writing the Xacro File
### 3️⃣ Create a `.xacro` File
Navigate to the `urdf` directory and create a new file named `myrobot.xacro`:
```bash
cd urdf
touch myrobot.xacro
```

### 4️⃣ Structure of the `myrobot.xacro` File

#### 📌 Declaring the Robot Name and Xacro Namespace
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="myrobot">
```

#### 📌 Defining the Base Link
The **base link** represents the main body of the robot.
```xml
  <link name="base_link">
      <inertial>
          <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
          <mass value="5.0"/>
          <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
      </inertial>
      <visual>
          <geometry>
             <cylinder radius="0.25" length="0.15"/>
          </geometry>
          <material name="blue">
              <color rgba="0.0 0.0 1.0 1.0"/>
          </material>
      </visual>
  </link>
```

#### 📌 Adding Left and Right Wheels
Each **wheel** is defined as a separate link.
```xml
  <link name="left_wheel">
      <inertial>
          <mass value="1.0"/>
      </inertial>
      <visual>
          <geometry>
              <cylinder radius="0.1" length="0.07"/>
          </geometry>
          <material name="black">
              <color rgba="0.0 0.0 0.0 1.0"/>
          </material>
      </visual>
  </link>

  <link name="right_wheel">
      <inertial>
          <mass value="1.0"/>
      </inertial>
      <visual>
          <geometry>
              <cylinder radius="0.1" length="0.07"/>
          </geometry>
          <material name="black">
              <color rgba="0.0 0.0 0.0 1.0"/>
          </material>
      </visual>
  </link>
```

#### 📌 Defining Joints for the Wheels
Joints connect the wheels to the base link.
```xml
  <joint name="left_wheel_joint" type="continuous">
      <origin xyz="0.065 0.15 -0.05" rpy="0.0 1.5707 0.0"/>
      <parent link="base_link"/>
      <child link="left_wheel"/>
      <axis xyz="0.0 1.0 0.0"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
      <origin xyz="0.065 -0.15 -0.05" rpy="0.0 1.5707 0.0"/>
      <parent link="base_link"/>
      <child link="right_wheel"/>
      <axis xyz="0.0 1.0 0.0"/>
  </joint>
```

#### 📌 Adding a Caster Wheel
A **caster wheel** helps balance the robot.
```xml
  <link name="caster_wheel">
      <inertial>
          <mass value="0.2"/>
      </inertial>
      <visual>
          <geometry>
              <sphere radius="0.05"/>
          </geometry>
          <material name="gray">
              <color rgba="0.5 0.5 0.5 1"/>
          </material>
      </visual>
  </link>

  <joint name="caster_wheel_joint" type="fixed">
      <parent link="base_link"/>
      <child link="caster_wheel"/>
      <origin xyz="-0.15 0 -0.1" rpy="0 0 0"/>
  </joint>
```

#### 📌 Closing the Robot Definition
```xml
</robot>
```

---

## 🛠️ Running the Xacro File
Once the Xacro file is ready, you can visualize it using **RViz**:



 **Launch the URDF model in RViz**
```bash
roslaunch urdf_tutorial display.launch model:=my_robot.xacro
```



---



