# `nav_ws` ROS Workspace Setup Guide with TurtleBot3 House World

This guide walks you through setting up a ROS workspace named `nav_ws` with two folders: `navigation` and `turtlebot3_simulations`, and launching  custom **house world** in Gazebo.

---

## 📁 Step 1: Create the Workspace and `src` Directory

```bash
mkdir -p ~/nav_ws/src
cd ~/nav_ws
```

---

## 🔧 Step 2: Initialize the Workspace

```bash
catkin_make
```

This will create `build/`, `devel/`, and other necessary files.

---

## 🧠 Step 3: Source the Workspace

```bash
source devel/setup.bash
```

Optional (recommended) — Add it to `.bashrc`:

```bash
echo "source ~/nav_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 📦 Step 4: Add uploaded Packages



```bash
mv ~/Downloads/navigation ~/nav_ws/src/
mv ~/Downloads/turtlebot3_simulations ~/nav_ws/src/
```

Or clone them:

```bash
cd ~/nav_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

---

## 🚠 Step 5: Build the Workspace Again

```bash
cd ~/nav_ws
catkin_make
```

---

## 🧽 Step 6: Launch TurtleBot3 in the Custom House World

> Make sure your custom world file exists, e.g., `house.world`, inside:

```
~/nav_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/
```



 launch the world:

```bash
export TURTLEBOT3_MODEL=burger   # or waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

---

## 🗘 Step 7: Run the Navigation Stack

In another terminal:

```bash
cd ~/nav_ws
source devel/setup.bash
roslaunch navigation navigation.launch
```

---

## ✅ Tips to Verify

- Use `rqt_graph` to verify node connections.
- Check if laser and TF data is publishing:

```bash
rostopic list
rostopic echo /scan
rosrun tf tf_echo map base_footprint
```

---

