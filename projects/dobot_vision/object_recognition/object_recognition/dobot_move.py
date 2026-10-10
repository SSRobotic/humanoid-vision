#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @author futingxing
# @date 2025-03-20
import time

import rclpy
from rclpy.node   import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from dobot_msgs_v4.srv import *
from camera_msgs_v4.msg import Result,YoloPose
import threading
from rclpy.executors import SingleThreadedExecutor

class MoveCircleActionServer(Node):
    def __init__(self, name):
        super().__init__(name)
        self.P_NC = []
        self.EnableRobot_l = self.create_client(EnableRobot,'/dobot_bringup_ros2/srv/EnableRobot')
        self.ModbusCreate_l = self.create_client(ModbusCreate,'/dobot_bringup_ros2/srv/ModbusCreate')   #创建modbus主站
        self.SetHoldRegs_l = self.create_client(SetHoldRegs,'/dobot_bringup_ros2/srv/SetHoldRegs')   #写保存寄存器
        self.ModbusClose_l = self.create_client(ModbusClose, '/dobot_bringup_ros2/srv/ModbusClose')  # 写保存寄存器
        self.MovJ_l = self.create_client(MovJ, '/dobot_bringup_ros2/srv/MovJ')  
        self.ToolDO_l = self.create_client(DO, '/dobot_bringup_ros2/srv/DO') 
        self.MovL_l = self.create_client(MovL, '/dobot_bringup_ros2/srv/MovL')  
        self.dobot = self.create_subscription(YoloPose, "/youl_pose", self.yolo_pose,
                                              10)  # 创建订阅者对象（消息类型、话题名、订阅者回调函    数、队列长度）
        while not self.EnableRobot_l.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        feed_thread = threading.Thread(
            target=self.connect_dh)
        feed_thread.daemon = True
        feed_thread.start()

    def yolo_pose(self,msg):
        #yellow
        for ii in msg.data:
            self.P_NC.append(ii.points)
    def gripper(self,state):
        tooldo_c = DO.Request()
        if state == 1:
           tooldo_c.index = 9
           tooldo_c.status = 1
        elif state == 2:
           tooldo_c.index = 9
           tooldo_c.status = 0
        elif state == 3:
           tooldo_c.index = 10
           tooldo_c.status = 1
           #tooldo_c.time = 60000
        elif state == 4:
            tooldo_c.index = 10
            tooldo_c.status = 0
        response = self.ToolDO_l.call_async(tooldo_c)
        time.sleep(0.3)
        #rclpy.spin_until_future_complete(self, response)  # 等待响应
        self.get_logger().info(f"{response.result()}")

    def spin_until_future_complete(self, future):
        # 使用独立的 Executor 等待 Future 完成
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        try:
            executor.spin_until_future_complete(future)
        finally:
            executor.remove_node(self)
            executor.shutdown()
    def movj(self,x,y,z,rx,ry,rz):
        movj_c = MovJ.Request()
        movj_c.mode = bool(0)
        movj_c.a = float(x)
        movj_c.b = float(y)
        movj_c.c = float(z)
        movj_c.d = float(rx)
        movj_c.e = float(ry)
        movj_c.f = float(rz)
        print(movj_c)
        response = self.MovJ_l.call_async(movj_c)
        time.sleep(0.3)
        #self.spin_until_future_complete(response)  # 等待响应
        self.get_logger().info(f"{response.result()}")
    def movl(self, x, y, z, rx, ry, rz):
        movl_c = MovL.Request()
        movl_c.mode = bool(0)
        movl_c.a = float(x)
        movl_c.b = float(y)
        movl_c.c = float(z)
        movl_c.d = float(rx)
        movl_c.e = float(ry)
        movl_c.f = float(rz)
        print(movl_c)
        response = self.MovL_l.call_async(movl_c)
        time.sleep(0.3)
        #self.spin_until_future_complete(response)  # 等待响应
        self.get_logger().info(f"{response.result()}")

    def connect_dh(self):
        #print("1:抓取\n2:查看当前识别结果\n3:运动至初始点\n4:抓取点上方")
        print("1: Grab\n2: View current recognition result\n3: Move to the initial point\n4: Above the grab point")
        while True:
            #aaa = input("请输入指令：")
            aaa = input("Please enter the command:")
            if aaa == "1":
                self.P1 = self.P_NC[0]
                print(self.P1)
                self.movj(214, 55, 300, 180, 0, -111)     #safety point
                self.movj(self.P1[0]-40, self.P1[1], 300, 180, 0, -111)  
                time.sleep(4)
                for i_p in self.P_NC:
                    if abs(i_p[0]-self.P1[0]) < 10 and abs(i_p[1]-self.P1[1]) < 10:
                        self.P2 = i_p
                        break
                self.movj(self.P2[0], self.P2[1], self.P2[2]+80, 180, 0, -111)  #above pick-up
                self.movl(self.P2[0], self.P2[1], self.P2[2], 180, 0, -111)  #pick-up
                time.sleep(1)
                self.gripper(1)       #12-vacuum 34-pressure 
                self.movl(self.P2[0], self.P2[1], self.P2[2]+80, 180, 0, -111)  #above pick-up
                
    
                self.movj(257, -84, 158, 180, 0, -111)  #above place
                self.movl(257, -84, 120, 180, 0, -111)  #place
                time.sleep(1)
                self.gripper(2)
                self.gripper(3)
                self.movl(257, -84, 158, 180, 0, -111) #above place
                self.gripper(4)
                self.movj(214, 55, 300, 180, 0, -111)  #safety point
                
                
            elif aaa == "2":
                print(self.P_NC[0])
                print(self.P_NC[0][2])
            elif aaa == "3":
                self.movj(214, 55, 300, 180, 0, -111)
            elif aaa == "4":  #抓取点上方80
                self.P1 = self.P_NC[0]
                self.movj(self.P1[0], self.P1[1], self.P1[2]+80, 180, 0, -111)
            elif aaa == "5":
                self.P1 = self.P_NC[0]
                self.movj(self.P1[0]-40, self.P1[1], 300, 180, 0, -111)  

def main(args=None):
    rclpy.init(args=args)
    node = MoveCircleActionServer("dh_server")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()  # 在主线程中运行 Executor
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
