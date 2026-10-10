#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import Image

import pyrealsense2 as rs

# ========== 以下为d435相机的配置 ========
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipe.start(cfg)
align_to = rs.stream.color
align = rs.align(align_to)  # combine depth map and color image


# =====================================


class NodePublisher(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("运行%s!" % name)


def main(args=None):
    rclpy.init(args=None)

    node = NodePublisher("realsense_pub_image_node")
    image_pub = node.create_publisher(Image, "image_raw", 10)
    image_pub2 = node.create_publisher(Image, "image_data2", 10)

    bridge = CvBridge()

    while True:
        frame = pipe.wait_for_frames()

        # ==== 得到d435的 彩色图像流 ===
        color_frame = align.process(frame).get_color_frame()
        color_frame = np.asanyarray(color_frame.get_data())

        # ==== 得到d435的 深度图像流 ===
        colorizer = rs.colorizer()
        depth_frame = align.process(frame).get_depth_frame()
        colored_depth_frame = np.asanyarray(colorizer.colorize(depth_frame).get_data())
        print(colored_depth_frame[0])

        # ==== 发布 彩色图像流 消息 ====
        # img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg = bridge.cv2_to_imgmsg(color_frame, encoding="bgr8")
        img_msg.header.frame_id = 'camera'
        img_msg.header.stamp = node.get_clock().now().to_msg()
        image_pub.publish(img_msg)

        # ==== 发布 深度图像流 消息 ====
        #img_msg2 = bridge.cv2_to_imgmsg(depth_frame, encoding = "16UC1")
        img_msg2 = bridge.cv2_to_imgmsg(colored_depth_frame, encoding="bgr8")
        image_pub2.publish(img_msg2)


if __name__ == "__main__":
    main()