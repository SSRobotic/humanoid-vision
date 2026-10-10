#!/usr/bin/env python3
# @author futingxing
# @date 2025-03-01

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import cv2.aruco as aruco
from scipy.spatial.transform import Rotation as R
from camera_msgs_v4.msg import InternalParameters,ArucoPose
bridge = CvBridge()
ARUCO_DICT = {"DICT_4X4_50": cv2.aruco.DICT_4X4_50, "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
              "DICT_4X4_250": cv2.aruco.DICT_4X4_250, "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
              "DICT_5X5_50": cv2.aruco.DICT_5X5_50, "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
              "DICT_5X5_250": cv2.aruco.DICT_5X5_250, "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
              "DICT_6X6_50": cv2.aruco.DICT_6X6_50, "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
              "DICT_6X6_250": cv2.aruco.DICT_6X6_250, "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
              "DICT_7X7_50": cv2.aruco.DICT_7X7_50, "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
              "DICT_7X7_250": cv2.aruco.DICT_7X7_250, "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
              "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
              "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
              "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
              "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
              "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11}
class NodeSubscribe(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("ArUco识别%s" % name)
        self.image_pub = self.create_publisher(Image, "image_aruco_raw", 10)
        self.pub = self.create_publisher(ArucoPose, "aruco_pose", 10)
        self.intr_matrix = []
        self.intr_coeffs = []
        self.msg_aruco_pose = ArucoPose()
    def calculate_center(self,corners): #计算图像中心坐标
        # 提取所有x坐标和y坐标
        x_coords = [point[0] for point in corners]
        y_coords = [point[1] for point in corners]

        # 计算x和y的平均值
        center_x = sum(x_coords) / len(x_coords)
        center_y = sum(y_coords) / len(y_coords)

        # 四舍五入并转换为整数
        center_x = int(round(center_x))
        center_y = int(round(center_y))

        return (center_x, center_y)
    def internal_parameters(self,intr):
        self.intr_matrix = np.array([
            [intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]
        ])
        self.intr_coeffs = np.array(intr.distortion_coefficient)

    def rotation_matrix_to_quaternion(self,R):
        """将旋转矩阵转换为四元数"""
        trace = np.trace(R)
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2  # S=4*qw
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # S=4*qx
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # S=4*qy
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # S=4*qz
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S

        return np.array([qw, qx, qy, qz])
    def rotation_to_euler(self,rotation, order='xyz'):
        """将旋转矩阵转换为欧拉角"""
        return R.from_matrix(rotation).as_euler(order)
    def callback(self, data):
        if len(self.intr_matrix) == 0 and len(self.intr_coeffs) == 0:
            return
        cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
        # 获取dictionary, 4x4的码，指示位50个
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        # 创建detector parameters
        parameters = aruco.DetectorParameters()
        # 输入rgb图, aruco的dictionary, 相机内参, 相机的畸变参数
        
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX  # 亚像素角点细化
        parameters.cornerRefinementWinSize = 7  # 窗口尺寸增大至7x7（默认5）
        parameters.adaptiveThreshWinSizeMin = 5  # 自适应阈值最小窗口
        parameters.adaptiveThreshWinSizeMax = 23  # 自适应阈值最大窗口
        parameters.minMarkerPerimeterRate = 0.03  # 过滤小尺寸误检（默认值）
        parameters.errorCorrectionRate = 0.6  # 提升容错率（默认0.6
        
        
        detector = aruco.ArucoDetector(aruco_dict, parameters, )
        # undistorted_img = cv2.undistort(cv_img, intr_matrix, intr_coeffs)
        # corners11, ids, rejectedImgPoints = detector.detectMarkers(undistorted_img)
        corners, ids, rejectedImgPoints = detector.detectMarkers(cv_img)
        # rvec是旋转向量， tvec是平移向量
        rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners, 0.05, self.intr_matrix, self.intr_coeffs)
        try:
            R_target2cam1, _ = cv2.Rodrigues(rvec)
            self.get_logger().info(f"{self.rotation_to_euler(R_target2cam1)}")
            euler_angles = self.rotation_matrix_to_quaternion(R_target2cam1)
            self.msg_aruco_pose.x = tvec[0][0][0]*1000
            self.msg_aruco_pose.y = tvec[0][0][1]*1000
            self.msg_aruco_pose.z = tvec[0][0][2]*1000
            self.msg_aruco_pose.qw = euler_angles[0]
            self.msg_aruco_pose.qx = euler_angles[1]
            self.msg_aruco_pose.qy = euler_angles[2]
            self.msg_aruco_pose.qz = euler_angles[3]
            aruco.drawDetectedMarkers(cv_img, corners)
            print(tvec)
            # 根据aruco码的位姿标注出对应的xyz轴, 0.05对应length参数，代表xyz轴画出来的长度
            cv2.drawFrameAxes(cv_img, self.intr_matrix, self.intr_coeffs, rvec, tvec, 0.05)
            cv2.putText(cv_img, "X:" + str(round(tvec[0][0][0]*1000,3))+"  Y:"+str(round(tvec[0][0][1]*1000,3))+"  Z:"+ str(round(tvec[0][0][2]*1000,3)), (150, 60), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        [255, 0, 0])

            cv2.imshow('RGB image', cv_img)
        except :
            print("未识别")
            self.msg_aruco_pose.x = 0.0
            self.msg_aruco_pose.y = 0.0
            self.msg_aruco_pose.z = 0.0
            self.msg_aruco_pose.qw =0.0
            self.msg_aruco_pose.qx =0.0
            self.msg_aruco_pose.qy =0.0
            self.msg_aruco_pose.qz = 0.0
            cv2.imshow('RGB image', cv_img)
        img_msg = bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        img_msg.header.frame_id = 'camera'
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(img_msg)
        self.pub.publish(self.msg_aruco_pose)
        key = cv2.waitKey(1)

def main(args=None):
    rclpy.init()
    # 建立一个节点(sub_image_node)用来接受以下两个话题中的图像数据
    node = NodeSubscribe("aruco_pubsub")
    # 接收话题image_data中的图像数据，并可视化
    node.create_subscription(Image, 'realsense_color_image', node.callback, 10)
    node.create_subscription(InternalParameters, 'realsense/internal_parameters', node.internal_parameters, 10)
    rclpy.spin(node)
    rclpy.shutdown()
