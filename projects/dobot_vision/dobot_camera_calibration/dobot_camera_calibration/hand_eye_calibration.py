#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @author futingxing
# @date 2025-03-05
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
import trajectory_msgs
import os
from camera_msgs_v4.msg import InternalParameters,ArucoPose
from dobot_msgs_v4.msg import ToolVectorActual
import threading
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from math import *
import math
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
def compute_relative_transform(prev_pose, curr_pose):
    """
    计算两个位姿之间的相对变换
    输入格式: [x, y, z, rx, ry, rz]
    返回: 旋转矩阵R, 平移向量t
    """
    # 转换为旋转矩阵和平移向量
    R_prev = euler_to_rotation(*prev_pose[3:])
    t_prev = np.array(prev_pose[:3])
    R_curr = quaternion_to_rotation_matrix(*curr_pose[3:])
    t_curr = np.array(curr_pose[:3])

    # 计算相对变换
    R_rel = R_curr @ R_prev.T
    t_rel = t_curr - R_rel @ t_prev
    return R_rel, t_rel

def hand_eye_calibration(arm_poses, cam_poses):
    """
    执行手眼标定
    参数:
        arm_poses: 机械臂末端位姿列表(N,6)
        cam_poses: 相机检测位姿列表(N,6)
    返回:
        X: 相机到末端的变换矩阵(4x4)
    """
    # 收集所有相对运动
    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []

    for i in range(0, len(arm_poses)):
        # 计算机械臂的相对运动（A矩阵）
        R_a = euler_to_rotation(arm_poses[i][3] * 3.1415 / 180, arm_poses[i][4] * 3.1415 / 180,
                                arm_poses[i][5] * 3.1415 / 180)
        t_a = np.array([[arm_poses[i][0]/1000], [arm_poses[i][1]/1000], [arm_poses[i][2]/1000]])
        R_gripper2base.append(R_a)
        t_gripper2base.append(t_a)

        # 计算相机的相对运动（B矩阵）
        R_b = quaternion_to_rotation_matrix(cam_poses[i][3:])
        t_b = np.array([[cam_poses[i][0]/1000], [cam_poses[i][1]/1000], [cam_poses[i][2]/1000]])
        R_target2cam.append(R_b)
        t_target2cam.append(t_b)

    # 执行手眼标定
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base=R_gripper2base,
        t_gripper2base=t_gripper2base,
        R_target2cam=R_target2cam,
        t_target2cam=t_target2cam,
        method=cv2.CALIB_HAND_EYE_TSAI
    )

    # 构建变换矩阵
    X = np.eye(4)
    X[:3, :3] = R_cam2gripper
    X[:3, 3] = t_cam2gripper.flatten()
    return X
def rotation_matrix_to_quaternion(R):
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

class SubscriberNode(Node):
    def __init__(self, name):
        super().__init__(name)  # ROS2节点父类初始化
        self.arm_poses = []
        self.cam_poses = []
        self.dobot = self.create_subscription(ToolVectorActual, "/dobot_msgs_v4/msg/ToolVectorActual", self.dobot_pose, 10)  # 创建订阅者对象（消息类型、话题名、订阅者回调函    数、队列长度）
        self.aruco = self.create_subscription(ArucoPose, "/aruco_pose",
                                            self.aruco_pose, 10)  # 创建订阅者对象（消息类型、话题名、订阅者回调函    数、队列长度）
        feed_thread = threading.Thread(
            target=self.send_goal)  # 机器状态反馈线程
        feed_thread.daemon = True
        feed_thread.start()


    def dobot_pose(self, msg): 
        self.dobot_xyz = [msg.x,msg.y,msg.z,msg.rx,msg.ry,msg.rz]    #机械臂坐标

    def aruco_pose(self, msg): 
        self.aruco_xyz = [msg.x,msg.y,msg.z,msg.qw,msg.qx,msg.qy,msg.qz]  #aruco码坐标


    def send_goal(self):  # 创建一个发送动作目标的函数
        df = 1
        print("1:采集\n2:执行标定\n3:保存\n4:验证\n5：清空当前数据")
        print("1: Collection\n2: Execute calibration\n3: Save\n4: Verify\n5: Clear current data")
        while True:
            #aa = input("输入指令：")
            aa = input("Please enter the command:")
            if aa == "1":
                self. arm_poses.append(self.dobot_xyz)
                self. cam_poses.append(self.aruco_xyz)
                #print(f"获取：{df}")
                print(f"Git：{df}")
                df += 1
                print(self.dobot_xyz)
                print(self.aruco_xyz)
            elif aa == "2":
                print(self. arm_poses)
                print(self. cam_poses)
                # 执行标定
                self.X = hand_eye_calibration(self.arm_poses, self.cam_poses)
                # 解析结果
                position = self.X[:3, 3]
                rotation_euler = rotation_to_euler(self.X[:3, :3])
                # print("相机到机械臂末端的变换矩阵:")
                print("Transformation matrix from camera to end of robotic arm:")
                print(self.X)
                # print("平移分量 (x, y, z):", position)
                # print("旋转分量 (rx, ry, rz):", np.degrees(rotation_euler))
                print("Translation component (x, y, z):", position)
                print("Rotation component (rx, ry, rz):", np.degrees(rotation_euler))

            elif aa == "3":
                quaternion = rotation_matrix_to_quaternion(self.X[:3, :3])
                # 要搜索的目录名
                dir_name = 'vision_resource_7b9c'
                # 搜索的起始路径（例如根目录 '/'）
                search_path = '/home'
                # 查找目录
                directory_path = find_directory(dir_name, search_path)
                if directory_path:
                    contents = f"{directory_path}/handeye_calib/hand_on_eye_calib.yaml"
                    data = {
                        'time': time.time(),
                        'hand_to_eye': {'x': float(position[0]),
                                        'y': float(position[1]),
                                        'z': float(position[2]),
                                        'qw': float(quaternion[0]),
                                        'qx': float(quaternion[1]),
                                        'qy': float(quaternion[2]),
                                        'qz': float(quaternion[3])}
                    }
                    with open(contents, 'w') as file:
                        yaml.dump(data, file)
                    self.get_logger().info(f"写入: {contents}")
                    self.get_logger().info(f"XYZ: {position},qwxyz: {quaternion}")
                    contents = f"{directory_path}/handeye_calib/aruco_p.yaml"
                    data = {
                        'time': time.time(),
                        'aruco': self.cam_poses
                    }
                    with open(contents, 'w') as file:
                        yaml.dump(data, file)
                    contents = f"{directory_path}/handeye_calib/dobot_p.yaml"
                    data = {
                        'time': time.time(),
                        'dobot': self.arm_poses
                    }
                    with open(contents, 'w') as file:
                        yaml.dump(data, file)
                else:
                    self.get_logger().info("目录未找到")
                    self.get_logger().info("Directory not found")
            elif aa == "4":
                for dobot_xyz,aruco_xyz in zip(self. arm_poses,self. cam_poses):
                     R_a = euler_to_rotation(dobot_xyz[3] * 3.1415 / 180, dobot_xyz[4] * 3.1415 / 180,dobot_xyz[5] * 3.1415 / 180)
                     t_a = np.array([[dobot_xyz[0] / 1000], [dobot_xyz[1] / 1000], [dobot_xyz[2] / 1000]])
                     # 构建变换矩阵
                     A_B = np.eye(4)
                     A_B[:3, :3] = R_a
                     A_B[:3, 3] = t_a.flatten()
                     R_b = euler_to_rotation(aruco_xyz[3] * 3.1415 / 180, aruco_xyz[4] * 3.1415 / 180,
                                             aruco_xyz[5] * 3.1415 / 180)
                     t_b = np.array([[aruco_xyz[0]/1000], [aruco_xyz[1]/1000], [aruco_xyz[2]/1000]])
                     #print(rotation_to_euler(R_b) * 180 / 3.14)
                     C_B = np.eye(4)
                     C_B[:3, :3] = R_b
                     C_B[:3, 3] = t_b.flatten()
                     #print(A_B,C_B)
                     PP = A_B@self.X@C_B
                     print(PP[:3,3])
                     #print(PP[:3,:3])
                     print(rotation_to_euler(PP[:3,:3])*180/3.14)
                   

            elif aa == "5":
                self.arm_poses = []
                self.cam_poses = []
                df = 1

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode("hand_eye_calibration")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
