#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @author futingxing
# @date 2025-03-25
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
from ultralytics import YOLO
from camera_msgs_v4.msg import InternalParameters,ArucoPose
from dobot_msgs_v4.msg import ToolVectorActual
from scipy.spatial.transform import Rotation as R
from math import *
import math
import threading
from dobot_msgs_v4.srv import *
import time
import os
from camera_msgs_v4.msg import Result,YoloPose
import yaml
def find_directory(dir_name, search_path):
    for root, dirs, files in os.walk(search_path):
        if dir_name in dirs:
            return os.path.join(root, dir_name)
    return None
def euler_to_rotation(rx, ry, rz, order='xyz'):
    """将欧拉角转换为旋转矩阵"""
    return R.from_euler(order, [rx, ry, rz]).as_matrix()

def rotation_to_euler(rotation, order='xyz'):
    """将旋转矩阵转换为欧拉角"""
    return R.from_matrix(rotation).as_euler(order)
def quaternion_to_rotation_matrix(q):
    """将四元数转换为旋转矩阵"""
    qw, qx, qy, qz = q
    R = np.array([
        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
    ])
    return R
bridge = CvBridge()
class NodeSubscribe(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("运行%s!" % name)
        #参数
        self.P_NC = []
        self.result = []
        
        self.tool_z = -10  # 末端工具长度补偿mm ## End tool length compensation mm
        
        # 加载YOLOv11模型（GPU加速）
        # 搜索的起始路径（例如根目录 '/'）
        dir_name = 'vision_resource_7b9c'
        # 搜索的起始路径（例如根目录 '/'）
        search_path = '/home/dobot/dobot_ros2_vision_ws'
        # 查找目录
        directory_path = find_directory(dir_name, search_path)
        if directory_path:
            contents = f"{directory_path}/handeye_calib/hand_on_eye_calib.yaml"
            contents_yolo = f"{directory_path}/yolo/best.pt"
            with open(contents, 'r') as file:
                data = yaml.safe_load(file)
            print(contents)
            print(data)
            t_a = np.array([data['hand_to_eye']["x"], data['hand_to_eye']["y"], data['hand_to_eye']["z"]])
            R_a = quaternion_to_rotation_matrix(np.array(
                [data['hand_to_eye']["qw"], data['hand_to_eye']["qx"], data['hand_to_eye']["qy"],
                 data['hand_to_eye']["qz"]]))
            self.T_C = np.eye(4)
            self.T_C[:3, :3] = R_a
            self.T_C[:3, 3] = t_a.flatten()
            print(self.T_C)
            self.model = YOLO(contents_yolo)  # 使用YOLOv11 Nano模型
            # self.model.to('cuda')  # 将模型加载到GPU
            # 类别标签（YOLOv8默认支持COCO数据集）
            self.class_names = self.model.names
        else:
            print("目录未找到")
        self.dobot = self.create_subscription(ToolVectorActual, "/dobot_msgs_v4/msg/ToolVectorActual", self.dobot_pose,
                                              10)  # 创建订阅者对象（消息类型、话题名、订阅者回调函    数、队列长度）
        self.yolo_pose = self.create_publisher(YoloPose, "youl_pose", 10)

    def dobot_pose(self, msg):
        self.dobot_xyz = [msg.x,msg.y,msg.z,msg.rx,msg.ry,msg.rz]

    def Conversion(self,yolo):
        # self.X = np.array([[0.03777268, 0.99928285, 0.00264903, - 0.11296617],
        #                    [-0.99923195, 0.03774283, 0.01053553, 0.03255997],
        #                    [0.01042799, -0.00304495, 0.99994099, -0.00369701],
        #                    [0., 0., 0., 1.]])
        # self.X = np.array([[ 0.99913467 ,-0.04079958 , 0.00808107, -0.03423476],
        #                    [ 0.04091712 , 0.99905058 ,-0.01495779, -0.1018077 ],
        #                    [-0.00746313 , 0.0152755  , 0.99985547,  0.01518846],
        #                    [ 0.         , 0.         , 0.        ,  1.        ]])
        R_a = euler_to_rotation(self.dobot_xyz[3] * 3.1415 / 180, self.dobot_xyz[4] * 3.1415 / 180,
                                self.dobot_xyz[5] * 3.1415 / 180)
        t_a = np.array([[self.dobot_xyz[0] / 1000], [self.dobot_xyz[1] / 1000], [self.dobot_xyz[2] / 1000]])
        # 构建变换矩阵
        A_B = np.eye(4)
        A_B[:3, :3] = R_a
        A_B[:3, 3] = t_a.flatten()
        self.aruco_xyz = yolo
        R_b = euler_to_rotation(self.aruco_xyz[3] * 3.1415 / 180, self.aruco_xyz[4] * 3.1415 / 180,
                                self.aruco_xyz[5] * 3.1415 / 180)
        t_b = np.array([[self.aruco_xyz[0]], [self.aruco_xyz[1]], [self.aruco_xyz[2]]])
        # print(rotation_to_euler(R_b) * 180 / 3.14)
        C_B = np.eye(4)
        C_B[:3, :3] = R_b
        C_B[:3, 3] = t_b.flatten()
        PP =  A_B@ self.T_C @C_B
        P_N = PP[:3, 3]*1000
        self.P_R = rotation_to_euler(PP[:3, :3]) * 180 / 3.14
        # print(P_N,P_R)
        self.P_NC = P_N
        return P_N,self.P_R

    def callback(self, data):
        cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
        # YOLOv11推理（GPU加速）
        results = self.model(cv_img,verbose=False)  # 输出检测结果
        msg = YoloPose()
        yolo_l = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())  # 获取边界框
                conf = box.conf.item()  # 置信度
                cls_id = int(box.cls.item())  # 类别ID
                cls_name = self.class_names[cls_id]  # 类别名称
                if conf < 0.6  or len(self.result) < 2:
                    continue
                # 获取边界框中心点
                ux, uy = (x1 + x2) // 2, (y1 + y2) // 2
                # print(ux, uy)
                camera_xyz = self.result[uy][ux]
                cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                if self.result[uy][ux][2] == 0:
                    continue
                c_p = list(camera_xyz)
                # print(c_p)
                try:
                     position,posture = self.Conversion(np.array([c_p[0],c_p[1],c_p[2]]+[0,0,0]))    #姿态固定为垂直抓取，需要使用obb识别姿态需要自行替换
                except:
                     continue
                kv1 = Result()
                kv1.name = cls_name
                kv1.id = int(cls_id)
                kv1.points = [float(position[0]),float(position[1]),float(position[2])+self.tool_z,float(posture[0]),float(posture[1]),float(posture[2])]
                yolo_l.append(kv1)
                cv2.circle(cv_img, (ux, uy), radius=5, color=(0, 255, 0), thickness=-1)
                cv2.putText(cv_img, f"{cls_name} ({camera_xyz[0]:.4f}, {camera_xyz[1]:.4f}, {camera_xyz[2]:.4f})",
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        msg.data = yolo_l
        self.yolo_pose.publish(msg)
        cv2.imshow("color_frame", cv_img)
        cv2.waitKey(1)


    def callback3(self, data):
        points = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
        xyz_list = np.array(points)
        self.result = xyz_list.reshape(-1, 640)
        # print(len(result))
        # print(result[130][471])
        # 打印前10个点的坐标
        # self.get_logger().info(f"First 10 points: {xyz_list[:10]}")


def main(args=None):
    rclpy.init()
    # 建立一个节点(sub_image_node)用来接受以下两个话题中的图像数据
    node = NodeSubscribe("yolo_v11")
    # 接收话题image_data中的图像数据，并可视化
    node.create_subscription(Image, 'realsense_color_image', node.callback, 10)
    # 接收话题image_data2中的图像数据，并可视化
    node.create_subscription(PointCloud2, 'realsense_cloud_intact', node.callback3, 10)
    rclpy.spin(node)
    rclpy.shutdown()
