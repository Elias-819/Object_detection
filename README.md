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
Python Packages
bash
复制
编辑
pip install opencv-python pyrealsense2 numpy
📂 File Structure
text
复制
编辑
detect_opencv/
├── detect_opencv/
│   └── 27_Mar.py           # Main ROS2 node script
├── package.xml
├── setup.py
└── README.md
▶️ Running the Node
If this is part of a ROS2 package:

bash
复制
编辑
ros2 run detect_opencv detect_node
Or run directly with Python:

bash
复制
编辑
python3 detect_opencv/27_Mar.py
Make sure your RealSense camera is connected and accessible.

📤 Published ROS2 Topics
Topic Name	Message Type	Description
/block_pose	geometry_msgs/Pose	Position of detected blocks (3D)
/bin_pose	geometry_msgs/Pose	Position of detected bins (3D)
/color_image	sensor_msgs/Image	Annotated camera image for debugging
/color_detection	std_msgs/String	Optional detection string (status)
🧠 Detection Logic
Start RealSense pipeline (color + depth)

Apply Gaussian blur and convert to HSV

Segment red and yellow regions via thresholding

Find contours and filter by:

Contour area

Shape (square-like)

Depth range (0.1 - 1.0 meters)

Real-world object size (in cm)

Classify as:

BLOCK if size ≈ 3–6 cm and close to camera

BIN if size ≈ 20–35 cm and farther away

Publish the 3D Pose of the object

Annotate and display the image in OpenCV

⚙️ Configurable Parameters
These parameters can be modified inside the script:

Parameter	Description
COLOR_RANGES	HSV thresholds for each target color
depth_value range	Valid depth values (e.g., 0.1m to 1.0m)
size_cm thresholds	Size limits for classifying block/bin
shape_ratio	Width/height ratio allowed for square shape
morph kernel size	Kernel size for morphological operations
🛠️ Customization Tips
🟦 Add more colors by extending the COLOR_RANGES dictionary

🟥 Adjust is_square_shape() to support rectangles or circles

🧠 Add object tracking or identification logic

🤖 Integrate with robot arm for pick-and-place tasks

🧪 Record or log the detected poses for offline analysis

🎯 Example Use Case
Goal: Detect red/yellow blocks and place them into matching colored bins

Robot: ROS2-enabled robot arm with mounted RealSense camera

Input: Real-time RGB-D video stream

Output: 3D Pose data of detected objects for grasping and placing

👨‍💻 Authors
Developed by Team 11
MSC Robotics 2025 – Embedded Vision Project

