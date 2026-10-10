#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np

bridge = CvBridge()


class NodeSubscribe(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("运行%s!" % name)

    def callback(self, data):
        cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("color_frame", cv_img)
        cv2.waitKey(1)

    def callback2(self, data):
        cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("color_depth_frame", cv_img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init()
    # 建立一个节点(sub_image_node)用来接受以下两个话题中的图像数据
    node = NodeSubscribe("realsense_sub_image_node")
    # 接收话题image_data中的图像数据，并可视化
    node.create_subscription(Image, 'image_data', node.callback, 10)
    # 接收话题image_data2中的图像数据，并可视化
    node.create_subscription(Image, 'image_data2', node.callback2, 10)
    rclpy.spin(node)
    rclpy.shutdown()