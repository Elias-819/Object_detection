# Object_detection
Opencv
# 🧠 RealSense Color Block & Bin Detection (ROS2)

This project uses **Intel RealSense** and **ROS2 (Humble)** to detect **colored cube blocks** and **colored bin boxes** from live RGB-D camera streams. It distinguishes between small movable blocks (~3cm) and larger static bins (~25cm) using **color**, **shape**, **depth**, and **real-world size estimation**.

---

## 📸 Features

- 🎨 Detect red and yellow objects based on HSV color filtering
- 📦 Distinguish between **blocks** and **bins** by estimating physical size
- 📏 Robust depth filtering to avoid background/invalid values
- 🧠 Shape-based filtering (approximate squares only)
- 📤 Publishes object 3D coordinates (`geometry_msgs/Pose`) over ROS2 topics
- 🖼 Displays annotated color image with OpenCV
- 🔧 Easily extendable to more colors and shapes

---

## 🧰 Dependencies

Make sure the following are installed:

- ROS2 Humble
- Intel RealSense SDK (`librealsense2`)
- Python packages:
  - `rclpy`
  - `cv2` (OpenCV)
  - `numpy`
  - `pyrealsense2`
  - `cv_bridge`
  - `sensor_msgs`, `geometry_msgs`, `std_msgs`

You can install dependencies via:

```bash
sudo apt install ros-humble-cv-bridge ros-humble-image-transport
pip install pyrealsense2 numpy opencv-python
🧩 How It Works
The script performs the following steps:

Initializes the RealSense RGB and depth streams

Converts frames into OpenCV images

Filters by HSV color range

Extracts contours and filters by:

Area

Shape (square-like)

Depth value

Estimated real-world object size

Classifies object as:

BLOCK if size ≈ 3–6cm and close to camera

BIN if size ≈ 20–35cm and farther from camera

Publishes the detected object's Pose to ROS2 topics:

/block_pose

/bin_pose

Annotates the image with label, depth, and size

▶️ Running the Code
bash
复制
编辑
ros2 run your_package your_script.py
Or directly (if it's executable):

bash
复制
编辑
python3 path/to/your_script.py
🧪 Topics Published
Topic Name	Type	Description
/block_pose	geometry_msgs/Pose	Pose of detected blocks
/bin_pose	geometry_msgs/Pose	Pose of detected bins
/color_image	sensor_msgs/Image	Annotated image for debugging
/color_detection (optional)	std_msgs/String	Simple status string
📦 Folder Structure (Example)
arduino
复制
编辑
detect_opencv/
├── detect_opencv/
│   └── 27_Mar.py           # Main detection script
├── package.xml
├── setup.py
└── README.md
🚀 Customization
Add more colors in the COLOR_RANGES dictionary

Modify is_square_shape() to allow rectangles or circles

Tune detection thresholds (size_cm, depth limits) to your environment

Integrate with robot arm to pick and place blocks

🙋‍♀️ Authors
Created by Team 11
Lab: MSC Robotics 2025

