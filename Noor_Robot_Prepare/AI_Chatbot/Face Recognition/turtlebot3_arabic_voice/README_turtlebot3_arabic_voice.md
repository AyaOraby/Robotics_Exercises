# TurtleBot3 Arabic Voice Recognition

This ROS package enables TurtleBot3 to recognize and process Arabic voice commands using either live microphone input or pre-recorded audio files.

---

## 📁 Package Structure

```
turtlebot3_arabic_voice/
├── CMakeLists.txt
├── data/
├── launch/
│   ├── arabic_voice.launch
│   └── process_audio.launch
├── package.xml
├── scripts/
│   ├── arabic_voice_recognition.py
│   └── process_audio_file.py
└── src/
```

---

## ✅ Requirements

- ROS Noetic
- Python 3
- `speech_recognition` library
- Microphone (for live voice recognition)
- Pre-recorded Arabic audio files (for testing)

---

## 🔧 Setup

1. Clone the package into your catkin workspace:

   ```bash
   cd ~/catkin_ws/src
   git clone <repo-url> turtlebot3_arabic_voice
   ```

2. Build your workspace:

   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

3. Install required Python packages:

   ```bash
   pip install SpeechRecognition
   ```

---

## 🚀 Launch Instructions

### 1. Live Arabic Voice Recognition

```bash
roslaunch turtlebot3_arabic_voice arabic_voice.launch
```

### 2. Process a Pre-recorded Audio File

```bash
roslaunch turtlebot3_arabic_voice process_audio.launch
```

---

## 📌 Notes

- Ensure your microphone is properly configured and working.
- You can place your test audio files in the `data/` directory.
- Make Python scripts executable if needed:

  ```bash
  chmod +x *.py
  ```

- Update the audio file path in `process_audio_file.py` as needed.

---
