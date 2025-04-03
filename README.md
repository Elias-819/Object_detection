# 🧠 RealSense Color Block & Bin Detection (ROS2)

This project uses **Intel RealSense** and **ROS 2 (Humble)** to detect **colored cube blocks (~3cm)** and **colored bin boxes (~25cm)** using RGB-D data. It performs real-world size estimation, shape filtering, and color segmentation to classify and publish the 3D positions of each object.

---

## 📸 Features

- 🎨 Detect red and yellow objects using HSV color filtering
- 📏 Estimate real-world size from pixel width and depth
- 📦 Classify small blocks vs. large bin boxes
- 🧠 Shape filtering to prefer square-like objects
- 🧼 Stable depth filtering with outlier rejection
- 📤 Publish object 3D coordinates via ROS2 topics
- 🖼 Real-time OpenCV display with annotations

---

## 🧰 Dependencies

### System Packages

Make sure the following are installed in your ROS2 environment:

```bash
sudo apt update
sudo apt install \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  librealsense2-dev


## 🧰 Python Packages

pip install opencv-python pyrealsense2 numpy


Output: 3D Pose data of detected objects for grasping and placing

👨‍💻 Authors
Developed by Team 11
MSC Robotics 2025 – Embedded Vision Project

