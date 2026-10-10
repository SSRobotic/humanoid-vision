'''
# @author futingxing
# @date 2025-03-20
description='Used to publish realsense images and parameters.',
'''

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import pyrealsense2 as rs
from camera_msgs_v4.msg import InternalParameters

# ========== 以下为d435相机的配置 ========
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipe.start(cfg)
align_to = rs.stream.color
align = rs.align(align_to)  # combine depth map and color image


# =====================================

class PointCloudPublisher(Node):
    rate = 30
    moving = True
    # frame_id = 'map'
    frame_id = 'Camera_Link_1'
    header = Header()
    header.frame_id = frame_id
    bridge = CvBridge()
    realsense_parameters = InternalParameters()

    dtype = PointField.FLOAT32
    point_step = 16
    fields = [PointField(name='x', offset=0, datatype=dtype, count=1),
              PointField(name='y', offset=4, datatype=dtype, count=1),
              PointField(name='z', offset=8, datatype=dtype, count=1),
              PointField(name='intensity', offset=12, datatype=dtype, count=1)]

    def __init__(self):
        super().__init__('pc_publisher')
        self.get_logger().info(f"正在发布图像及参数")
        self.get_logger().info(f"Publishing images and parameters")
        self.publisher_ = self.create_publisher(PointCloud2, 'realsense_cloud_intact', 10)
        self.publisher2_ = self.create_publisher(PointCloud2, 'realsense_cloud', 10)
        self.image_pub = self.create_publisher(Image, "realsense_color_image", 10)
        self.image_pub2 = self.create_publisher(Image, "realsense_depth_image", 10)
        self.Parameters = self.create_publisher(InternalParameters, "realsense/internal_parameters", 10)
        timer_period = 1 / self.rate
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        frame = pipe.wait_for_frames()

        # ==== 得到d435的 彩色图像流 ===
        color_frame = align.process(frame).get_color_frame()
        color_frame = np.asanyarray(color_frame.get_data())

        # ==== 得到d435的 深度图像流 ===
        colorizer = rs.colorizer()
        depth_frame = align.process(frame).get_depth_frame()
        colored_depth_frame = np.asanyarray(colorizer.colorize(depth_frame).get_data())


        if not depth_frame:
            raise RuntimeError("无法获取深度帧")

        # 获取深度传感器的内参
        depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

        # 将深度帧转换为numpy数组
        depth_image = np.asanyarray(depth_frame.get_data())

        # 获取深度图像的宽度和高度
        height, width = depth_image.shape

        # 生成所有像素的坐标网格
        x_idx, y_idx = np.meshgrid(np.arange(width), np.arange(height))

        # 将像素坐标展平为一维数组
        x_idx_flat = x_idx.flatten()
        y_idx_flat = y_idx.flatten()

        # 获取深度值并展平
        depth_values_flat = depth_image.flatten()

        # 过滤无效深度值（深度值为0）
        valid_mask = depth_values_flat != 0
        x_idx_valid = x_idx_flat[valid_mask]
        y_idx_valid = y_idx_flat[valid_mask]
        depth_values_valid = depth_values_flat[valid_mask]


        # 将像素坐标和深度值转换为3D坐标
        # 使用向量化计算
        fx = depth_intrinsics.fx  # 焦距 x
        fy = depth_intrinsics.fy  # 焦距 y
        ppx = depth_intrinsics.ppx  # 主点 x
        ppy = depth_intrinsics.ppy  # 主点 y
        intr_coeffs = depth_intrinsics.coeffs

        # 计算过滤3D坐标
        x_3d = (x_idx_valid - ppx) * depth_values_valid / fx
        y_3d = (y_idx_valid - ppy) * depth_values_valid / fy
        z_3d = depth_values_valid

        # 将结果组合成一个数组
        points_3d = np.vstack((x_3d / 1000, y_3d / 1000, z_3d / 1000, z_3d / 1000)).T
        pc3_msg = point_cloud2.create_cloud(self.header, self.fields, points_3d)

        # 计算3D坐标
        x_3d = (x_idx_flat - ppx) * depth_values_flat / fx
        y_3d = (y_idx_flat - ppy) * depth_values_flat / fy
        z_3d = depth_values_flat

        # 将结果组合成一个数组 单位m
        points_3d = np.vstack((x_3d / 1000, y_3d / 1000, z_3d / 1000, z_3d / 1000)).T
        pc2_msg = point_cloud2.create_cloud(self.header, self.fields, points_3d)

        self.publisher_.publish(pc2_msg) #完整cloud
        self.publisher2_.publish(pc3_msg) #过滤无效数据

        # ==== 发布 彩色图像流 消息 ====
        img_msg = self.bridge.cv2_to_imgmsg(color_frame, encoding="bgr8")
        img_msg.header.frame_id = self.frame_id
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(img_msg)


        # ==== 发布 深度图像流 消息 ====
        # img_msg2 = bridge.cv2_to_imgmsg(depth_frame, encoding = "16UC1")
        img_msg2 = self.bridge.cv2_to_imgmsg(colored_depth_frame, encoding="bgr8")
        img_msg2.header.frame_id = self.frame_id
        img_msg2.header.stamp = self.get_clock().now().to_msg()
        self.image_pub2.publish(img_msg2)

        # ==== 内参发布及畸变系数 ====
        self.realsense_parameters.fx = fx
        self.realsense_parameters.fy = fy
        self.realsense_parameters.ppx =ppx
        self.realsense_parameters.ppy =ppy
        self.realsense_parameters.distortion_coefficient = intr_coeffs
        self.Parameters.publish(self.realsense_parameters)

def main(args=None):
    rclpy.init(args=args)
    pc_publisher = PointCloudPublisher()
    rclpy.spin(pc_publisher)
    pc_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()