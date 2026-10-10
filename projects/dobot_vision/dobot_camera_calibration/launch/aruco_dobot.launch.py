from launch import LaunchDescription         
from launch_ros.actions import Node          

def generate_launch_description():           
    return LaunchDescription([                
        Node(                                  
            package='dobot_camera_calibration',          # 节点所在的功能包
            executable='aruco_pubsub', # 执行aruco识别
        ),
        Node(
            package='dobot_camera_calibration',
            executable='feedback', # 发布机械臂姿态
        ),
        Node(
            package='dobot_camera_calibration',
            executable='ui',  # 发布机械臂姿态
        ),
    ])
