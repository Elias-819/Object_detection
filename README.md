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


sudo apt update
sudo apt install \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  librealsense2-dev 


## Python Packages

Install the following Python packages:

pip install opencv-python pyrealsense2 numpy

##  📂 File Structure
detect_opencv/
├── detect_opencv/
│   └── 27_Mar.py           # Main ROS2 node script
├── package.xml
├── setup.py
└── README.md

##  ▶️ Running the Node
If the node is part of a ROS2 package, run it with:

ros2 run detect_opencv detect_node
If you want to run it directly using Python:

python3 detect_opencv/27_Mar.py
Make sure your RealSense camera is connected and working.

## 📤 Published ROS 2 Topics
Topic Name	Message Type	Description
/block_pose	geometry_msgs/Pose	Position of detected blocks (3D)
/bin_pose	geometry_msgs/Pose	Position of detected bins (3D)
/color_image	sensor_msgs/Image	Annotated image from the camera
/color_detection	std_msgs/String	Optional status or debug messages

##  🧠 Detection Logic
Start RealSense RGB-D stream

Apply Gaussian blur and convert image to HSV

Use HSV color thresholds to segment target colors

Detect contours and filter by:

Contour area

Shape (square-like)

Valid depth range (0.1m to 1.0m)

Real-world size (in centimeters)

Classify each detected object as:

BLOCK: size ≈ 3–6 cm and close

BIN: size ≈ 20–35 cm and farther

Publish Pose of each object

Draw contours and labels using OpenCV

## ⚙️ Configurable Parameters
You can customize the following values in the script:

Parameter	Description
COLOR_RANGES	HSV lower and upper bounds for target colors
depth_value range	Valid depth thresholds to reject invalid regions
size_cm thresholds	Real-world size ranges for blocks and bins
shape_ratio	Acceptable width/height ratio for square detection
Morph kernel size	Morphological kernel for noise cleaning

## 🛠️ Customization Tips
Add more target colors by updating the COLOR_RANGES dictionary

Adjust the shape checking logic in is_square_shape() to allow rectangles or circles

Use object tracking to maintain persistent object identity

Integrate with a robot arm controller for automated pick-and-place

Add logging or data saving for offline analysis and training

## 🎯 Example Use Case
Objective: Pick colored blocks and drop them into matching colored bins

Hardware: ROS2-enabled robot arm with mounted RealSense D435 or D455

Input: Live RGB-D stream from RealSense

Output: Published 3D poses for robot grasp planning and bin placement

## 👨‍💻 Authors
Developed by Dongze Li
MSC Robotics 2025 – Embedded Vision Project

