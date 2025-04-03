import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import Pose

class RealSenseColorDetectionNode(Node):
    def __init__(self):
        super().__init__('realsense_color_detection')

        self.publisher_ = self.create_publisher(String, 'color_detection', 10)
        self.image_publisher_ = self.create_publisher(Image, 'color_image', 10)
        self.block_publisher = self.create_publisher(Pose, 'block_pose', 10)
        self.bin_publisher = self.create_publisher(Pose, 'bin_pose', 10)

        self.bridge = CvBridge()

        # ✅ 初始化 RealSense 相机
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # ✅ 相机内参
        self.fx, self.fy = 615.0, 615.0
        self.cx, self.cy = 320, 240

        self.timer = self.create_timer(0.1, self.process_frames)

    def process_frames(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        color_image = cv2.GaussianBlur(color_image, (5, 5), 0)
        hsv_img = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        COLOR_RANGES = {
            'red': (np.array([0, 70, 50]), np.array([10, 255, 255])),
            'yellow': (np.array([18, 100, 100]), np.array([38, 255, 255]))
        }

        COLOR_MAP = {
            'red': (0, 0, 255),
            'yellow': (0, 255, 255)
        }

        for color, (lower, upper) in COLOR_RANGES.items():
            mask = cv2.inRange(hsv_img, lower, upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 300:
                    continue

                # ✅ 可选：判断是否为方块
                #if not self.is_square_shape(cnt):
                #    continue

                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                center_x, center_y = int(rect[0][0]), int(rect[0][1])
                width_pix, height_pix = rect[1]

                # ✅ 获取深度
                depth_value = self.get_stable_depth(depth_image, center_x, center_y)
                if depth_value is None:
                    continue

                # ✅ 像素宽高 ➜ 实际尺寸（cm）
                real_w = (width_pix / self.fx) * depth_value
                real_h = (height_pix / self.fy) * depth_value
                size_cm = max(real_w, real_h) * 100  # 单位转为 cm
                if depth_value < 0.1 or depth_value > 1.0:
                    continue  # 忽略不合理深度

                X, Y, Z = self.pixel_to_world(center_x, center_y, depth_value)
                """
                # ✅ 使用实际尺寸判断类型
                if self.is_square_shape(cnt) and 2 <= size_cm <= 6:
                    self.detect_blocks(color, X, Y, Z)
                    label = f"{color} BLOCK"
                elif 20 <= size_cm <= 35:
                    self.detect_bins(color, X, Y, Z)
                    label = f"{color} BIN"
                else:
                    continue  # 排除不在尺寸范围内的目标
                """

                # ✅ 给 label 一个默认值
                label = None
                # ✅ 判断积木
                is_block = self.is_square_shape(cnt) and 2 <= size_cm <= 6 and Z < 0.5
                is_bin = 20 <= size_cm <= 35 and Z > 0.3

                if is_block:
                    self.detect_blocks(color, X, Y, Z)
                    label = f"{color} BLOCK"

                if is_bin:
                    self.detect_bins(color, X, Y, Z)
                    label = f"{color} BIN"

                # ✅ 显示调试信息
                if label:
                    cv2.drawContours(color_image, [box], 0, COLOR_MAP[color], 2)
                    cv2.circle(color_image, (center_x, center_y), 3, (0, 0, 255), -1)
                    cv2.putText(color_image, f"{label} {size_cm:.1f} {depth_value*100:.1f}cm" , (center_x + 10, center_y + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_MAP[color], 2)
                else:
                    self.get_logger().debug(f"Skipped: size={size_cm:.1f} Z={Z:.2f}")


        # ✅ 显示并发布图像
        cv2.imshow("Color Detection", color_image)
        cv2.waitKey(1)
        img_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        self.image_publisher_.publish(img_msg)

    def get_stable_depth(self, depth_img, x, y):
        h, w = depth_img.shape
        x1, y1 = max(0, x - 5), max(0, y - 5)
        x2, y2 = min(w, x + 5), min(h, y + 5)
        roi = depth_img[y1:y2, x1:x2].astype(np.float32)
        roi = roi[roi > 0]

        if roi.size < 10:
            return None

        median = np.median(roi)
        filtered = roi[np.abs(roi - median) < 0.1 * median]
        return np.median(filtered) / 1000.0 if filtered.size > 0 else None

    def pixel_to_world(self, u, v, depth):
        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        Z = depth
        return X, Y, Z

    def detect_blocks(self, color, X, Y, Z):
        self.get_logger().info(f"[BLOCK] {color} at ({X:.2f}, {Y:.2f}, {Z:.2f})")
        msg = Pose()
        msg.position.x = X
        msg.position.y = Y
        msg.position.z = Z
        msg.orientation.w = 1.0
        self.block_publisher.publish(msg)

    def detect_bins(self, color, X, Y, Z):
        self.get_logger().info(f"[BIN] {color} at ({X:.2f}, {Y:.2f}, {Z:.2f})")
        msg = Pose()
        msg.position.x = X
        msg.position.y = Y
        msg.position.z = Z
        msg.orientation.w = 1.0
        self.bin_publisher.publish(msg)

    def is_square_shape(self, contour):
        rect = cv2.minAreaRect(contour)
        width, height = rect[1]
        if height == 0 or width == 0:
            return False
        ratio = max(width, height) / min(width, height)
        return 0.6 < ratio < 1.6  # 放宽到容忍倾斜、透视畸变

    def shutdown(self):
        self.pipeline.stop()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseColorDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node.')
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
