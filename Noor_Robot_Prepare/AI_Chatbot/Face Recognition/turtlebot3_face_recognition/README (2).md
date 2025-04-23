# TurtleBot3 Face Recognition

This ROS package enables face detection using image, video, or a laptop webcam. It is designed to work with TurtleBot3 and supports various input sources.

---

## 📁 Package Structure

```
turtlebot3_face_recognition/
├── CMakeLists.txt
├── config/
├── data/
├── launch/
│   ├── face_detection_image.launch
│   ├── face_detection_video.launch
│   └── laptop_cam_face.launch
├── package.xml
├── scripts/
│   ├── detect_face_image.py
│   ├── detect_face_video.py
│   ├── laptop_cam_face_detection.py
│   └── test_face_detection.sh
└── src/
```

---

## ✅ Requirements

- ROS Noetic
- OpenCV (`cv2`)
- `cv_bridge`, `image_transport`
- TurtleBot3 setup (`turtlebot3_msgs`, etc.)

---

## 🔧 Setup

1. Clone the package into your catkin workspace:

   ```bash
   cd ~/catkin_ws/src
   git clone <repo-url> turtlebot3_face_recognition
   ```

2. Build the workspace:

   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

---

## 🚀 Launch Instructions

### 1. Detect Faces from an Image

```bash
roslaunch turtlebot3_face_recognition face_detection_image.launch
```

### 2. Detect Faces from a Video

```bash
roslaunch turtlebot3_face_recognition face_detection_video.launch
```

### 3. Detect Faces from Laptop Webcam

```bash
roslaunch turtlebot3_face_recognition laptop_cam_face.launch
```

---

## 🧪 Testing via Script

```bash
cd ~/catkin_ws/src/turtlebot3_face_recognition/scripts
bash test_face_detection.sh
```

---

## 📌 Notes

- Make sure your Python scripts are executable:

  ```bash
  chmod +x *.py
  ```

- You may need to adjust video/image paths in the scripts.
- Ensure your ROS environment is correctly sourced before launching.

---

## 🧠 Author

**Philomath**  
Custom face recognition tools for TurtleBot3 🤖
